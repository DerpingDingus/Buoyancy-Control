import rclpy
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from neodriver_ros.srv import SetPixel, Fill

import time

try:
    import board
    import busio
    from adafruit_seesaw.seesaw import Seesaw
    from adafruit_seesaw.neopixel import NeoPixel
except Exception as e:
    board = None
    busio = None
    Seesaw = None
    NeoPixel = None
    _import_error = e
else:
    _import_error = None

PIXEL_ORDERS = {
    'RGB': (0, 1, 2),
    'GRB': (1, 0, 2),
    'BRG': (2, 0, 1),
    'GBR': (1, 2, 0),
    'RBG': (0, 2, 1),
    'BGR': (2, 1, 0),
}

class NeoDriverNode(Node):
    def __init__(self):
        super().__init__('neodriver')

        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_addr', '0x60')
        self.declare_parameter('num_pixels', 2)
        self.declare_parameter('brightness', 0.3)
        self.declare_parameter('pixel_order', 'RGB')
        self.declare_parameter('neopixel_pin', 18)
        self.declare_parameter('use_topics', True)

        if _import_error is not None:
            self.get_logger().fatal(f"Failed to import Blinka/Seesaw libs: {_import_error}")
            raise _import_error

        i2c_bus = int(self.get_parameter('i2c_bus').value)
        i2c_addr_param = self.get_parameter('i2c_addr').value
        i2c_addr = int(i2c_addr_param, 16) if isinstance(i2c_addr_param, str) else int(i2c_addr_param)
        self.num_pixels = int(self.get_parameter('num_pixels').value)
        init_brightness = float(self.get_parameter('brightness').value)
        order_key = str(self.get_parameter('pixel_order').value).upper()
        neopixel_pin = int(self.get_parameter('neopixel_pin').value)
        use_topics = bool(self.get_parameter('use_topics').value)

        if order_key not in PIXEL_ORDERS:
            self.get_logger().warn(f"Unknown pixel_order '{order_key}', defaulting to RGB")
            order_key = 'RGB'

        try:
            self.get_logger().info(f"Opening NeoDriver at I2C addr 0x{i2c_addr:02X} on bus {i2c_bus}")
            i2c = busio.I2C(board.SCL, board.SDA)
            self._ss = Seesaw(i2c, addr=i2c_addr)
            self._pixels = NeoPixel(self._ss, neopixel_pin, self.num_pixels, bpp=3, brightness=init_brightness, auto_write=False, pixel_order=order_key)
        except Exception as e:
            self.get_logger().fatal(f"Failed to init NeoDriver: {e}")
            raise

        self.get_logger().info(f"NeoDriver ready: {self.num_pixels} pixels, order={order_key}, pin={neopixel_pin}")

        self._buf = [(0, 0, 0)] * self.num_pixels

        self._srv_set_pixel = self.create_service(SetPixel, 'set_pixel', self._on_set_pixel)
        self._srv_fill = self.create_service(Fill, 'fill', self._on_fill)

        if use_topics:
            self._sub0 = self.create_subscription(ColorRGBA, 'pixel0', lambda msg: self._on_color_msg(0, msg), 10)
            if self.num_pixels > 1:
                self._sub1 = self.create_subscription(ColorRGBA, 'pixel1', lambda msg: self._on_color_msg(1, msg), 10)

        self._dirty = True
        self._timer = self.create_timer(0.05, self._refresh)

    def _clamp8(self, v):
        return max(0, min(255, int(v)))

    def _on_color_msg(self, index, msg: ColorRGBA):
        if 0 <= index < self.num_pixels:
            r = self._clamp8(msg.r * 255.0)
            g = self._clamp8(msg.g * 255.0)
            b = self._clamp8(msg.b * 255.0)
            self._buf[index] = (r, g, b)
            self._dirty = True

    def _on_set_pixel(self, req, res):
        try:
            if not (0 <= req.index < self.num_pixels):
                res.ok = False
                res.message = f"index {req.index} out of range [0,{self.num_pixels-1}]"
                return res
            r, g, b = self._clamp8(req.r), self._clamp8(req.g), self._clamp8(req.b)
            self._buf[req.index] = (r, g, b)
            if req.brightness >= 0.0:
                self._pixels.brightness = max(0.0, min(1.0, float(req.brightness)))
            self._apply()
            res.ok = True
            res.message = ""
        except Exception as e:
            res.ok = False
            res.message = str(e)
        return res

    def _on_fill(self, req, res):
        try:
            r, g, b = self._clamp8(req.r), self._clamp8(req.g), self._clamp8(req.b)
            if req.brightness >= 0.0:
                self._pixels.brightness = max(0.0, min(1.0, float(req.brightness)))
            for i in range(self.num_pixels):
                self._buf[i] = (r, g, b)
            self._apply()
            res.ok = True
            res.message = ""
        except Exception as e:
            res.ok = False
            res.message = str(e)
        return res

    def _apply(self):
        for i, (r, g, b) in enumerate(self._buf):
            self._pixels[i] = (r, g, b)
        self._pixels.show()
        self._dirty = False

    def _refresh(self):
        if self._dirty:
            self._apply()

    def destroy_node(self):
        try:
            for i in range(self.num_pixels):
                self._pixels[i] = (0, 0, 0)
            self._pixels.show()
        except Exception:
            pass
        super().destroy_node()

def main():
    rclpy.init()
    node = NeoDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
