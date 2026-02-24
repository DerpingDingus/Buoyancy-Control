#!/usr/bin/env python3
"""
ROS2 Node for controlling a servo motor via button inputs on Raspberry Pi.
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray, Bool
from motor_interfaces.msg import MotorState

import gpiozero
from gpiozero import LED

class ButtonMotorNode(Node):
    def __init__(self):
        super().__init__('button_motor_node')

        # Declare parameters
        self.declare_parameter('joint_name', 'joint')
        # Note: gpiozero uses BCM pin numbering by default
        self.declare_parameter('button_pin_cw', 5) 
        self.declare_parameter('button_pin_ccw', 29)
        self.declare_parameter('debounce_time', 0.05)
        self.declare_parameter('led_pin', 26)

        # Get parameters
        self.joint_name = self.get_parameter('joint_name').value
        pin_cw = self.get_parameter('button_pin_cw').value
        pin_ccw = self.get_parameter('button_pin_ccw').value
        debounce = self.get_parameter('debounce_time').value
        led_pin = self.get_parameter('led_pin').value



        self.get_logger().info(
            f"Button Motor Node for joint '{self.joint_name}'\n"
            f"  CW Button Pin: {pin_cw}\n"
            f"  CCW Button Pin: {pin_ccw}\n"
            f"  Led Pin: {led_pin}\n"
            f"  Debounce Time: {debounce} s"
        )

        self.led = LED(led_pin)

        # Publisher for servo commands
        # Topic matches servo_motor_node: /{joint_name}/servo_cmd
        self.pub_cmd = self.create_publisher(
            Float64MultiArray, f'/{self.joint_name}/servo_cmd', 10
        )

        # Current position tracking (updated by motor feedback)
        self.current_pos_deg = 0.0
        self.commanded_pos = 0.0
        self.command_ref_set = False

        # Subscriber for motor state to know where we are
        self.sub_state = self.create_subscription(MotorState, f'/{self.joint_name}/motor_state', self.on_state, 10)

        # Setup leak sensor subscription
        self.leak_status = self.create_subscription(Bool, '/leak_status', self.leak_response, 10)

        # Setup buttons with gpiozero
        try:
            self.button_cw = gpiozero.Button(pin_cw, bounce_time=debounce)
            self.button_ccw = gpiozero.Button(pin_ccw, bounce_time=debounce)
            
            # Assign callbacks
            self.button_cw.when_pressed = self.on_cw_press
            self.button_ccw.when_pressed = self.on_ccw_press
        except Exception as e:
            self.get_logger().error(f"Failed to initialize GPIO: {e}")

    def on_state(self, msg: MotorState):
        # The ServoMotorNode maps position_deg to abs_position in the MotorState message
        self.current_pos_deg = msg.abs_position

        if self.command_ref_set == False:
            self.commanded_pos = self.current_pos_deg
            self.command_ref_set = True

    def leak_response(self, msg):
        self.get_logger().info(f'leak status: {msg.data}')
        if msg.data:
            self.led.on()

    def on_cw_press(self):
        self.commanded_pos += 3600.0
        new_pos = self.commanded_pos
        self.get_logger().info(f"CW Pressed. Target: {new_pos:.1f}°")
        self.send_pos(new_pos)

    def on_ccw_press(self):
        self.commanded_pos -= 3600.0
        new_pos = self.commanded_pos
        self.get_logger().info(f"CCW Pressed. Target: {new_pos:.1f}°")
        self.send_pos(new_pos)

   # def on_state(self, msg: MotorState):
    #    self.get_logger().info(f"Current drawn is: {msg.current:.3f}A")

    def on_state(self, msg: MotorState):
        self.get_logger().info(f"Torque output is: {msg.torque: .3f}Nm")

    def send_pos(self, pos_deg: float):
        """
        Sends a Position Command (Mode 4) to the ServoMotorNode.
        The layout expected is [mode, value0, value1, value2]
        """
        msg = Float64MultiArray()
        # Mode 6 = Position/Velocity/Acceleration Control
        # value0 = target position in degrees
        # v1 and v2 are unused in mode 4 but included for consistency
        msg.data = [6.0, float(pos_deg), 7200.0, 3700.0] 
        self.pub_cmd.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = ButtonMotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
