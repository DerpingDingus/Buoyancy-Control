import adafruit_dht
import board
import gpiozero
from gpiozero import LED
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class HumiditySensor(Node):
    def __init__(self):
        super().__init__("humidity_node")
        self.declare_parameter(
            "led_pin",16
        )
        self.declare_parameter(
            "dht_pin",12
        )
        self.declare_parameter("humidity_threshold", 60.0)
        self.declare_parameter("name","humidity_1")

        self.led_pin = self.get_parameter("led_pin").value
        self.dht_pin = self.get_parameter("dht_pin").value
        self.name = self.get_parameter("name").value
        self.humidity_threshold = self.get_parameter("humidity_threshold").value

        self.led = LED(self.led_pin)

        self.pub_status = self.create_publisher(Float32, f"/{self.name}/humidity", 10)

        self.humidity = 0.0
        pin_name = f"D{self.dht_pin}"
        self.dht_device = adafruit_dht.DHT22(getattr(board, pin_name))

        self.get_logger().info(f"Humidity Sensor: {self.name} Started")

        self.create_timer(0.1, self.update_loop)

    def update_loop(self):
        try:
            # Read Humidity
            humidity = self.dht_device.humidity

            if humidity is not None:
                msg = Float32()
                msg.data = float(humidity)
                self.pub_status.publish(msg)

                if humidity > self.humidity_threshold:
                    self.led.on()
                    self.get_logger().info(
                        f"Warning Humidity Exceeds Threshold\nHumidity: {humidity:.1f}%"
                    )

                else:
                    self.led.off()
            else:
                # Optional: log a debug message if reading is None
                pass

        except RuntimeError as error:
            # DHT sensors often throw RuntimeErrors during normal operation 
            # (checksum failures, etc.). We just catch and continue.
            self.get_logger().warn(f"Reading error: {error.args[0]}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = HumiditySensor()
    try:
        rclpy.spin(node)  # Keep the node running
    except KeyboardInterrupt:
        # Handle graceful shutdown on Ctrl+C
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

