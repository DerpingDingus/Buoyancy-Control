#!/usr/bin/env python3
"""
ROS2 Node for controlling a servo motor via timers on Raspberry Pi.
"""

import rclpy
from rclpy.node import Node

import time

from std_msgs.msg import Float64MultiArray, Bool, Int32MultiArray
from motor_interfaces.msg import MotorState

class TimerControlNode(Node):
    def __init__(self):
        super().__init__('timer_control_node')

        # Declare parameters
        self.declare_parameter('joint_name', 'joint')
        self.declare_parameter('start_time', 10)
        self.declare_parameter('delay_time', 60)

        # Get parameters
        self.joint_name = self.get_parameter('joint_name').value
        self.start_time = self.get_parameter('start_time').value
        self.delay_time = self.get_parameter('delay_time').value

        self.get_logger().info(
            f"  Timer Control Node for joint '{self.joint_name}'\n"
            f"  Start Delay Time: {self.start_time}\n"
            f"  Switch Delay: {self.delay_time}\n"
        )

        # Publisher for servo commands
        # Topic matches servo_motor_node: /{joint_name}/servo_cmd
        self.pub_cmd = self.create_publisher(
            Float64MultiArray, f'/{self.joint_name}/servo_cmd', 10
        )

        self.led_cmd = self.create_publisher(
            Int32MultiArray, f'/{self.joint_name}/led_states', 10
        )

        # Current position tracking (updated by motor feedback)
        self.current_pos_deg = 0.0
        self.commanded_pos = 0.0
        self.stopped = True
        self.start = True
        self.state = 'DOWN'

        # Iteration Tracking
        self.counter = 0
        self.shutdown_requested = False

        # Led state tracking
        self.led = 0

        # Subscriber for motor state to know where we are
        self.sub_state = self.create_subscription(MotorState, f'/{self.joint_name}/motor_state', self.motion_state, 10)
        
        # Startup LED state
        self.led_state_change(self.led)

        #Start Delay

        time.sleep(self.start_time)


        # Start Motion
        self.motion_state(MotorState) 

    def buoyancy_down(self):
            self.commanded_pos = 0.0
            self.led = 1
            new_pos = self.commanded_pos
            self.get_logger().info(f"Decreasing Buoyancy")
            self.send_pos(new_pos)
            self.led_state_change(self.led)
            self.countdown()

    def buoyancy_up(self):
            self.commanded_pos = 0.0
            self.led = 1
            new_pos = self.commanded_pos
            self.get_logger().info(f"Increasing Buoyancy")
            self.led_state_change(self.led)
            self.send_pos(new_pos)
            self.countdown()

    def motion_state(self, msg: MotorState):
        self.current_pos_deg = msg.abs_position

        if self.commanded_pos == self.current_pos_deg:
            self.stopped = True

        if self.start == True:
            self.start = False
        elif self.start == False and self.stopped == True and self.state == 'DOWN':
            time.sleep(5)

        if self.state == 'DOWN' and self.stopped == True:
            self.buoyancy_down()
            self.state = 'UP'
        
        if self.state == 'UP':
            time.sleep(self.delay_time)
            self.buoyancy_up()
            self.state = 'DOWN'
            self.stopped = False

    def led_state_change(self, led):
        if led == 0:
            led_state = [1, 0, 0]
        elif led == 1:
            led_state = [0, 1, 0]
        elif led == 2:
            led_state = [0, 0, 1]
        
        self.send_led_state(led_state)
        self.get_logger().info(f'LED state switched')

    def send_pos(self, pos_deg: float):
        msg = Float64MultiArray()

        # Mode 6 = Position/Velocity/Acceleration Control
        # value0 = target position in degrees
        # v1 and v2 are unused in mode 4 but included for consistency
        msg.data = [6.0, float(pos_deg), 10800.0, 5400.0] 
        self.pub_cmd.publish(msg)

    def send_led_state(self, led_state):
        msg = Int32MultiArray()
        msg.data = led_state
        self.led_cmd.publish(msg)

    def countdown(self):
        if self.counter < 1:
            self.counter += 1
        else:
            self.get_logger().info('Finished 2 runs. Stopping node...')
            self.led = 2
            self.led_state_change(self.led)
            self.shutdown_requested = True

def main(args=None):
    rclpy.init(args=args)
    node = TimerControlNode()
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
