# mjbots_power_dist

ROS 2 Humble driver that listens to an MJBots power dist r4.5b on a SocketCAN
interface (default: `can0`) and republishes telemetry to ROS topics. The node
extracts bus voltage/current, 5 V and 12 V rail telemetry, board temperature, and
status flags from the board's periodic CAN frames.

## Usage

Build the workspace with `colcon build --symlink-install`, source it, then run:

```bash
ros2 run mjbots_power_dist power_dist_node --ros-args \
  -p can_interface:=can0
```

A launch file is also provided:

```bash
ros2 launch mjbots_power_dist power_dist.launch.py
```

### Parameters

| Name | Default | Description |
| ---- | ------- | ----------- |
| `can_interface` | `can0` | SocketCAN interface that the power dist board is connected to. |
| `status_id` | `0` | Set to the expected telemetry arbitration ID. When left at `0` the node auto-detects the first compatible frame (matching the upstream [mjbots/power_dist](https://github.com/mjbots/power_dist) layout). |
| `status_id_mask` | `0x1FFFFFFF` | Optional mask applied to the arbitration ID match (useful when the firmware encodes a device ID in the CAN identifier). |
| `poll_hz` | `50.0` | How often the node polls buffered CAN frames for new telemetry. |
| `frame_length_warning` | `16` | Minimum payload length expected before logging a warning. |

### Topics

* `/power_dist/battery` (`sensor_msgs/BatteryState`): bus voltage, bus current, and board temperature.
* `/power_dist/diagnostics` (`diagnostic_msgs/DiagnosticArray`): detailed rail telemetry and status flags.
* `/power_dist/raw_frame` (`std_msgs/UInt8MultiArray`): raw payload bytes from the telemetry frame.

### Telemetry layout

The parser expects the 16-byte layout published by the MJBots firmware:

* bytes 0-1: bus voltage (mV)
* bytes 2-3: bus current (mA, signed)
* bytes 4-5: board temperature (centi-deg C, signed)
* bytes 6-7: 5 V rail voltage (mV)
* bytes 8-9: 5 V rail current (mA, signed)
* bytes 10-11: 12 V rail voltage (mV)
* bytes 12-13: 12 V rail current (mA, signed)
* bytes 14-15: status flags

Frames shorter than 16 bytes are accepted; missing fields are published as
`NaN`.
