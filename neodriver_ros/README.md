# =========================
# README.md
# =========================
# ROS 2 Humble – Adafruit NeoDriver (I2C → NeoPixel) node for Jetson Orin Nano (Ubuntu 22.04)
#
# This package talks to the Adafruit **NeoDriver (I2C to NeoPixel)** board and drives WS2812/WS2811 LEDs.
# It targets two LEDs by default but supports any strip length.
#
# ✨ Features
# - Services to set an individual pixel or fill the whole strip
# - Optional topics to drive each pixel with ColorRGBA
# - Parameters for I2C bus, address, pixel count, brightness, and pixel order
# - Clean shutdown (turns off the LEDs)
#
# 📦 Dependencies (system)
#   sudo apt update && sudo apt install -y python3-pip python3-smbus i2c-tools
#
# 🧰 Python libs (Blinka + Seesaw):
#   pip3 install --upgrade adafruit-blinka adafruit-circuitpython-seesaw
#
# 🧩 Enable I2C on Jetson Orin Nano
#   - Use **Jetson-IO** or your board’s config tool to enable I2C on your chosen header (SDA/SCL)
#   - Confirm device (often /dev/i2c-1):  ls /dev/i2c*
#   - Find the NeoDriver address (default is often 0x60, but verify):  sudo i2cdetect -y 1
#
# 🏁 Build & Run
#   cd ~/ros2_ws/src
#   git clone <this_package> neodriver_ros
#   cd .. && colcon build --symlink-install
#   source install/setup.bash
#
#   # Launch with 2 LEDs at I2C addr 0x60 on bus 1
#   ros2 launch neodriver_ros neodriver.launch.py i2c_bus:=1 i2c_addr:=0x60 num_pixels:=2
#
# 🔧 Services
#   ros2 service call /neodriver/set_pixel neodriver_ros/SetPixel "{index: 0, r: 255, g: 0, b: 0, brightness: -1.0}"
#   ros2 service call /neodriver/fill      neodriver_ros/Fill     "{r: 0, g: 0, b: 255, brightness: -1.0}"
#
# 🎨 Topic control (optional; enabled by param use_topics:=true)
#   ros2 topic pub /neodriver/pixel0 std_msgs/ColorRGBA '{r: 0.0, g: 1.0, b: 0.0, a: 1.0}'
#   ros2 topic pub /neodriver/pixel1 std_msgs/ColorRGBA '{r: 1.0, g: 1.0, b: 0.0, a: 1.0}'
#
# ⚠️ Notes
# - The NeoPixel output pin on the NeoDriver is fixed on the board; in the seesaw driver this is typically **pin 18**.
#   If your board variant uses a different internal pin, adjust the `neopixel_pin` parameter accordingly.
# - The default pixel order here is **RGB**; if your LEDs require GRB (common) or another order, change `pixel_order`.
