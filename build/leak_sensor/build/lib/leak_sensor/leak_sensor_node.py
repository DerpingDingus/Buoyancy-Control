#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool # Changed to String for descriptive messaging
from gpiozero import DigitalInputDevice

class LeakSensorNode(Node):
    def __init__(self):
        super().__init__('leak_sensor_monitor')

        self.declare_parameter("gpio_pin", 23)
        self.declare_parameter("pull", "DOWN")

        self.gpio_pin = self.get_parameter("gpio_pin").value

        self.default_status= self.get_parameter("pull")
        
        if self.default_status == "UP":
            pull = True
        else:
            pull = False


        
        # Initialize the sensor on GPIO 23
        # pull_up=False assumes the sensor outputs 3.3V when a leak occurs
        self.sensor = DigitalInputDevice(self.gpio_pin, pull_up= pull)
        
        # Publisher for other ROS 2 nodes to consume the status
        self.publisher_ = self.create_publisher(Bool, 'leak_status', 10)
        
        # Attach event callbacks
        self.sensor.when_activated = self.report_leak
        self.sensor.when_deactivated = self.report_clear

        self.get_logger().info('Leak detection system initialized on GPIO 23.')

    def report_leak(self):
        msg = Bool()
        msg.data = True
        self.publisher_.publish(msg)
        # This prints to your terminal via ROS 2 logging
        self.get_logger().warn("Leak detected!")

    def report_clear(self):
        msg = Bool()
        msg.data = False
        self.publisher_.publish(msg)
        self.get_logger().info("Status: Sensor is dry.")

def main(args=None):
    rclpy.init(args=args)
    node = LeakSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

