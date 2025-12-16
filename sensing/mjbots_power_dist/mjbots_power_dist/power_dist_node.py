import math
from typing import Dict, Optional

import can
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import UInt8MultiArray


def parse_power_dist_payload(data: bytes) -> Dict[str, Optional[float]]:
    """Parse a power distribution telemetry frame.

    The default layout matches the MJBots power dist r4.5b firmware telemetry:

    * bytes 0-1: high-voltage bus voltage in millivolts (uint16)
    * bytes 2-3: high-voltage bus current in milliamps (int16)
    * bytes 4-5: board temperature in centi-deg C (int16)
    * bytes 6-7: 5V rail voltage in millivolts (uint16)
    * bytes 8-9: 5V rail current in milliamps (int16)
    * bytes 10-11: 12V rail voltage in millivolts (uint16)
    * bytes 12-13: 12V rail current in milliamps (int16)
    * bytes 14-15: status flags (uint16)

    Frames shorter than 16 bytes return any available fields and leave the
    remainder as ``None``. Values are scaled to volts/amps/degrees Celsius.
    """
    fields: Dict[str, Optional[float]] = {
        'bus_voltage_v': None,
        'bus_current_a': None,
        'board_temp_c': None,
        'rail5_voltage_v': None,
        'rail5_current_a': None,
        'rail12_voltage_v': None,
        'rail12_current_a': None,
        'status_flags': None,
    }

    if len(data) >= 2:
        fields['bus_voltage_v'] = int.from_bytes(data[0:2], byteorder='little', signed=False) / 1000.0
    if len(data) >= 4:
        fields['bus_current_a'] = int.from_bytes(data[2:4], byteorder='little', signed=True) / 1000.0
    if len(data) >= 6:
        fields['board_temp_c'] = int.from_bytes(data[4:6], byteorder='little', signed=True) / 100.0
    if len(data) >= 8:
        fields['rail5_voltage_v'] = int.from_bytes(data[6:8], byteorder='little', signed=False) / 1000.0
    if len(data) >= 10:
        fields['rail5_current_a'] = int.from_bytes(data[8:10], byteorder='little', signed=True) / 1000.0
    if len(data) >= 12:
        fields['rail12_voltage_v'] = int.from_bytes(data[10:12], byteorder='little', signed=False) / 1000.0
    if len(data) >= 14:
        fields['rail12_current_a'] = int.from_bytes(data[12:14], byteorder='little', signed=True) / 1000.0
    if len(data) >= 16:
        fields['status_flags'] = int.from_bytes(data[14:16], byteorder='little', signed=False)

    return fields


class PowerDistNode(Node):
    def __init__(self) -> None:
        super().__init__('mjbots_power_dist')

        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('status_id', 0x0500)
        self.declare_parameter('poll_hz', 50.0)
        self.declare_parameter('frame_length_warning', 8)

        self._can_interface = self.get_parameter('can_interface').get_parameter_value().string_value
        self._status_id = int(self.get_parameter('status_id').value)
        poll_hz = float(self.get_parameter('poll_hz').value)
        self._frame_length_warning = int(self.get_parameter('frame_length_warning').value)

        self.get_logger().info(
            f'Opening SocketCAN interface "{self._can_interface}" for power dist telemetry (ID=0x{self._status_id:X})'
        )
        self._bus = can.interface.Bus(channel=self._can_interface, bustype='socketcan')
        self._reader = can.BufferedReader()
        self._notifier = can.Notifier(self._bus, [self._reader], timeout=1.0)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._battery_pub = self.create_publisher(BatteryState, 'power_dist/battery', qos)
        self._diagnostic_pub = self.create_publisher(DiagnosticArray, 'power_dist/diagnostics', qos)
        self._raw_pub = self.create_publisher(UInt8MultiArray, 'power_dist/raw_frame', qos)

        poll_period = 1.0 / poll_hz if poll_hz > 0.0 else 0.1
        self._timer = self.create_timer(poll_period, self._poll_bus)

    def destroy_node(self) -> bool:
        if hasattr(self, '_notifier'):
            self._notifier.stop()
        if hasattr(self, '_bus'):
            self._bus.shutdown()
        return super().destroy_node()

    def _poll_bus(self) -> None:
        while True:
            frame = self._reader.get_message(0.0)
            if frame is None:
                break
            if frame.arbitration_id != self._status_id:
                continue
            self._handle_frame(frame)

    def _handle_frame(self, frame: can.Message) -> None:
        data = bytes(frame.data)
        if len(data) < self._frame_length_warning:
            self.get_logger().warn(
                f'Received short power dist frame (len={len(data)}); expected at least {self._frame_length_warning} bytes'
            )

        fields = parse_power_dist_payload(data)
        now = self.get_clock().now().to_msg()

        battery = BatteryState()
        battery.header.stamp = now
        battery.voltage = fields['bus_voltage_v'] if fields['bus_voltage_v'] is not None else math.nan
        battery.current = fields['bus_current_a'] if fields['bus_current_a'] is not None else math.nan
        battery.temperature = fields['board_temp_c'] if fields['board_temp_c'] is not None else math.nan
        battery.present = True
        battery.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        self._battery_pub.publish(battery)

        diag_status = DiagnosticStatus()
        diag_status.name = 'mjbots_power_dist'
        diag_status.level = DiagnosticStatus.WARN if fields['status_flags'] else DiagnosticStatus.OK
        diag_status.message = 'OK' if diag_status.level == DiagnosticStatus.OK else 'Flags set'
        diag_status.hardware_id = self._can_interface

        def add_kv(key: str, value: Optional[float], suffix: str = '') -> None:
            msg = 'nan' if value is None or math.isnan(value) else f'{value:.3f}{suffix}'
            diag_status.values.append(KeyValue(key=key, value=msg))

        add_kv('bus_voltage_v', fields['bus_voltage_v'])
        add_kv('bus_current_a', fields['bus_current_a'])
        add_kv('board_temp_c', fields['board_temp_c'])
        add_kv('rail5_voltage_v', fields['rail5_voltage_v'])
        add_kv('rail5_current_a', fields['rail5_current_a'])
        add_kv('rail12_voltage_v', fields['rail12_voltage_v'])
        add_kv('rail12_current_a', fields['rail12_current_a'])
        if fields['status_flags'] is not None:
            diag_status.values.append(
                KeyValue(key='status_flags', value=f"0x{int(fields['status_flags']):04X}")
            )

        diag_array = DiagnosticArray()
        diag_array.header.stamp = now
        diag_array.status.append(diag_status)
        self._diagnostic_pub.publish(diag_array)

        raw_msg = UInt8MultiArray()
        raw_msg.data = list(data)
        self._raw_pub.publish(raw_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PowerDistNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
