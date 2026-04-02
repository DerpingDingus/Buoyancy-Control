#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray # Changed to String for descriptive messaging
from gpiozero import LEDBoard
from signal import pause

class LedNode(Node):
    def __init__(self):
        super().__init__('status_led')
        
        self.declare_parameter("joint_name", "joint")
        self.declare_parameter("led_pin_1", 5)
        self.declare_parameter("led_pin_2", 6)
        self.declare_parameter("led_pin_3", 13)
        
        self.joint_name = self.get_parameter("joint_name").value
        self.led_pin_1 = self.get_parameter("led_pin_1").value
        self.led_pin_2 = self.get_parameter("led_pin_2").value
        self.led_pin_3 = self.get_parameter("led_pin_3").value
        
        # Initializes the 3 LEDs as a board
        self.leds = LEDBoard(self.led_pin_1, self.led_pin_2, self.led_pin_3)

        # Creates state variable to determine LED pattern
        self.state = 'IDLE'

        # Publisher for other ROS 2 nodes to consume the status
        self.subscription = self.create_subscription(
                Int16MultiArray,
                'led_state',
                self.led_callback,
                10
        )

        # Attach event callbacks
        self.get_logger().info(f'Status LEDs Enabled')

    def state_switching(self):
    
    def report_leak(self):
        msg = Bool()
        msg.data = True
        self.publisher_.publish(msg)
        self.get_logger().warn(f'Leak detected at {self.joint_name}!!!')

    def report_clear(self):
        msg = Bool()
        msg.data = False
        self.publisher_.publish(msg)
        self.get_logger().info("Sensor is dry.")

def main(args=None):
    rclpy.init(args=args)
    node = LedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.leds.off()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
