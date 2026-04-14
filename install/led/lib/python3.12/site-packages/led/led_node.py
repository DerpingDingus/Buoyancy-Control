#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
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

        # Subscriber to catch msgs from other nodes
        self.subscription = self.create_subscription(
                Int32MultiArray,
                f"/{self.joint_name}/led_states",
                self.state_switching,
                10
        )

        # Attach event callbacks
        self.get_logger().info(f'Status LEDs Enabled')

    def state_switching(self, msg):
        self.leds.value = tuple(msg.data)
        self.get_logger().info(f'LEDs status switched')
    
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
