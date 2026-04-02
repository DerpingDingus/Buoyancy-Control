#!/usr/bin/env python3
"""
ROS2 Node for controlling a servo motor via timers on Raspberry Pi.
"""

import rclpy
from rclpy.node import Node

import time

from std_msgs.msg import Float64MultiArray, Bool, String
from motor_interfaces.msg import MotorState

class ZeroMotorNode(Node):
    def __init__(self):
        super().__init__('zero_motor_node')

        # Declare parameters
        self.declare_parameter('joint_name', 'joint')
        self.declare_parameter('start_time', 5)

        # Get parameters
        self.joint_name = self.get_parameter('joint_name').value
        self.start_time = self.get_parameter('start_time').value

        # Publisher for servo commands
        # Topic matches servo_motor_node: /{joint_name}/servo_cmd
        self.special_cmd = self.create_publisher(
            String, f'/{self.joint_name}/special_cmd', 10
        )

        # Iteration Tracking
        self.shutdown_requested = False

        # Startup Delay
        time.sleep(self.start_time)

        # Send message
        self.send_special("zero")

    def send_special(self,command:str) -> None:
        msg = String()
        msg.data = command
        self.special_cmd.publish(msg)
        print("Published message")

    def countdown(self):
            self.get_logger().info('Motor Origin Set')
            self.shutdown_requested = True

def main(args=None):
    rclpy.init(args=args)
    node = ZeroMotorNode()
    try:
        while rclpy.ok() and not node.shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.1)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

