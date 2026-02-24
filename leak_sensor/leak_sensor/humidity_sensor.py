import time
import board
import adafruit_dht
from std_msgs.msg import Float32
import gpiozero
from gpiozero import LED

import rclpy
import rclpy.node import Node


class HumiditySensor(Node):
    def __init__(self):
        super().__init__('humidity_node')
        self.declare_paramater('led_pin',)
        self.declare_parameter('dht_pin',)
        self.declare_paramter('humidity_threshold',60.0)
        self.declare_paramter('name')

        led_pin = self.get_parameter('led_pin')
        dht_pin = self.get_parameter('dht_pin')
        name = self.get_parameter('name')
        humidity_threshold = self.get_parameter('humidity_threshold')

    self.led = LED(led_pin)

    self.pub_status = self.create_publisher(
            Float32, f'/{name}/humidity',10
            )

    self.humidity = 0.0
    self.dht_device = adafruit_dht.DHT22(DHT_PIN)

    self.get_logger().info(f"Humidity Sensor: {name} Started")











# --- Configuration ---
DHT_PIN = board.D4  # GPIO 4
LED_PIN = 18        # GPIO 18
HUMIDITY_THRESHOLD = 60.0  # Percentage to trigger LED

# Setup LED
GPIO.setmode(GPIO.BCM)
GPIO.setup(LED_PIN, GPIO.OUT)

# Initialize DHT sensor (Change to DHT11 if needed)
dht_device = adafruit_dht.DHT22(DHT_PIN)

print("Starting Humidity Monitor...")

try:
    while True:
        try:
            # Read values
            humidity = dht_device.humidity
            temp_c = dht_device.temperature

            if humidity is not None:
                print(f"Temp: {temp_c:.1f}C | Humidity: {humidity:.1f}%")

                # Logic for LED
                if humidity > HUMIDITY_THRESHOLD:
                    GPIO.output(LED_PIN, GPIO.HIGH)
                    print("--- High Humidity Detected! LED ON ---")
                else:
                    GPIO.output(LED_PIN, GPIO.LOW)
            
        except RuntimeError as error:
            # DHT sensors are notoriously glitchy; just keep retrying
            print(f"Reading error: {error.args[0]}")
            time.sleep(2.0)
            continue
        except Exception as error:
            dht_device.exit()
            raise error

        time.sleep(2.0)  # DHT sensors need ~2s between readings

except KeyboardInterrupt:
    print("Program stopped by user")
finally:
    GPIO.output(LED_PIN, GPIO.LOW)
    GPIO.cleanup()

