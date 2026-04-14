#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray # Changed to String for descriptive messaging
import time

class TimerNode(Node):
    def __init__(self):
        super().__init__('led_timer')
        
        self.declare_parameter("joint_name", "joint")
        self.declare_parameter("time_delay", 5)
        
        self.joint_name = self.get_parameter("joint_name").value
        self.time_delay = self.get_parameter("time_delay").value

        self.pub_cmd = self.create_publisher(
                Int32MultiArray,f'/{self.joint_name}/led_states', 10
        )

        # Attach event callbacks
        self.get_logger().info(f'LED timer started')
        
        time.sleep(self.time_delay)

        self.led_switching()

    def led_switching(self):
        led_state = [1, 0, 0]
        self.send_state(led_state)
        self.get_logger().info(f'LED 1 On')
        time.sleep(self.time_delay)

        led_state = [0, 1, 0]
        self.send_state(led_state)
        self.get_logger().info(f'LED 2 On')
        time.sleep(self.time_delay)

        led_state = [0, 0, 1]
        self.send_state(led_state)
        self.get_logger().info(f'LED 3 On')
        time.sleep(self.time_delay)

    def send_state(self, led_state):
        msg = Int32MultiArray()
        msg.data = led_state
        self.pub_cmd.publish(msg)
    
def main(args=None):
    rclpy.init(args=args)
    node = TimerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
