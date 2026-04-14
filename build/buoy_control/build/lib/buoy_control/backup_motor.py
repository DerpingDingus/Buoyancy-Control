#!/usr/bin/env python3
"""
ROS2 Node for controlling a servo motor via timers on Raspberry Pi.
"""

import rclpy
from rclpy.node import Node

import time

from std_msgs.msg import Float64MultiArray, Bool
from motor_interfaces.msg import MotorState

class BackupMotorNode(Node):
    def __init__(self):
        super().__init__('backup_motor_node')

        # Declare parameters
        self.declare_parameter('joint_name', 'joint')
        self.declare_parameter('start_time', 5)

        # Get parameters
        self.joint_name = self.get_parameter('joint_name').value
        self.start_time = self.get_parameter('start_time').value


        self.get_logger().info(
            f"  Start Delay Time: {self.start_time}\n"
        )

        # Publisher for servo commands
        # Topic matches servo_motor_node: /{joint_name}/servo_cmd
        self.pub_cmd = self.create_publisher(
            Float64MultiArray, f'/{self.joint_name}/servo_cmd', 10
        )

        self.commanded_pos = 0.0

        # Iteration Tracking
        self.shutdown_requested = False

        #Start Delay

        time.sleep(self.start_time)

        # Start Motion
        self.buoyancy_down() 

    def buoyancy_down(self):
        self.commanded_pos = -20000
        new_pos = self.commanded_pos
        self.get_logger().info(f"Moving Motor")
        self.send_pos(new_pos)
        self.countdown()

    def send_pos(self, pos_deg: float):
        msg = Float64MultiArray()

        # Mode 6 = Position/Velocity/Acceleration Control
        # value0 = target position in degrees
        # v1 and v2 are unused in mode 4 but included for consistency
        msg.data = [6.0, float(pos_deg), 10800.0, 5400.0] 
        self.pub_cmd.publish(msg)

    def countdown(self):
            self.get_logger().info('Finished Movement Calibration')
            self.shutdown_requested = True

def main(args=None):
    rclpy.init(args=args)
    node = BackupMotorNode()
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
