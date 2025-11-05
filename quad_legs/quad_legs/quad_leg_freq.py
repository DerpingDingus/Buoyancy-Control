#!/usr/bin/env python3
"""
Headless frequency controller for 4 legs (ROS 2 Humble)

- Rotates all legs with average frequency 'frequency_hz' (± allowed).
- Inside ±30° about down (0 rad), angular speed = 6× ω0; outside = 1× ω0.
- Motors 1 & 4 share a phase; Motors 2 & 3 share a phase offset by 'group_offset_deg'.

Publishes velocity-only MIT commands (p=0, kp, kd, torque_ff=0) to:
  /motor{1..4}/mit_cmd
Optionally sends special "start"/"zero" on startup (auto_start/auto_zero).
"""

import math, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String


def wrap_to_pi(x: float) -> float:
    return (x + math.pi) % (2.0 * math.pi) - math.pi

def sgn(x: float) -> float:
    return -1.0 if x < 0 else (1.0 if x > 0 else 0.0)


class QuadLegFreq(Node):
    def __init__(self):
        super().__init__('quad_leg_freq')

        # ---- Parameters ----
        self.declare_parameter('frequency_hz', 0.5)       # average frequency (±)
        self.declare_parameter('group_offset_deg', 180.0) # phase offset between (1&4) and (2&3)
        self.declare_parameter('fast_band_deg', 30.0)     # +/- around "down"
        self.declare_parameter('kp', 0.0)
        self.declare_parameter('kd', 1.0)
        self.declare_parameter('max_vel_abs', 30.0)       # rad/s clamp
        self.declare_parameter('control_hz', 100.0)       # publish rate
        self.declare_parameter('auto_start', True)        # send "start" once
        self.declare_parameter('auto_zero', False)        # optionally "zero" once

        self.freq_hz = float(self.get_parameter('frequency_hz').value)
        self.group_offset = math.radians(float(self.get_parameter('group_offset_deg').value))
        self.band_rad = math.radians(float(self.get_parameter('fast_band_deg').value))
        self.KP = float(self.get_parameter('kp').value)
        self.KD = float(self.get_parameter('kd').value)
        self.max_vel_abs = float(self.get_parameter('max_vel_abs').value)
        hz = float(self.get_parameter('control_hz').value)
        self.dt = 1.0 / max(1e-6, hz)

        # ---- Publishers ----
        self.cmd_pubs = {
            1: self.create_publisher(Float64MultiArray, '/motor1/mit_cmd', 10),
            2: self.create_publisher(Float64MultiArray, '/motor2/mit_cmd', 10),
            3: self.create_publisher(Float64MultiArray, '/motor3/mit_cmd', 10),
            4: self.create_publisher(Float64MultiArray, '/motor4/mit_cmd', 10),
        }
        self.special_pubs = {
            1: self.create_publisher(String, '/motor1/special_cmd', 10),
            2: self.create_publisher(String, '/motor2/special_cmd', 10),
            3: self.create_publisher(String, '/motor3/special_cmd', 10),
            4: self.create_publisher(String, '/motor4/special_cmd', 10),
        }

        # ---- State ----
        self.theta_base = 0.0
        self.last_time = time.monotonic()

        # Startup specials
        self._maybe_specials()

        # Timer
        self.create_timer(self.dt, self._tick)
        self.get_logger().info(
            f"QuadLegFreq running: f={self.freq_hz:+.3f} Hz, "
            f"offset={math.degrees(self.group_offset):.1f}°, band=±{math.degrees(self.band_rad):.1f}°"
        )

    def _maybe_specials(self):
        if bool(self.get_parameter('auto_start').value):
            self._broadcast_special("start")
        if bool(self.get_parameter('auto_zero').value):
            self._broadcast_special("zero")

    def _broadcast_special(self, text: str):
        msg = String(); msg.data = text
        for p in self.special_pubs.values():
            p.publish(msg)

    # average frequency derivation: ω0 = (31/36) * 2π |f|
    def _omega0(self, f_abs: float) -> float:
        return (31.0 / 36.0) * 2.0 * math.pi * f_abs

    def _region_speed(self, theta: float, f_hz: float) -> float:
        """Piecewise speed with 6× in central band."""
        if f_hz == 0.0:
            return 0.0
        s = sgn(f_hz)
        omega0 = self._omega0(abs(f_hz))
        omega_mag = 6.0 * omega0 if abs(wrap_to_pi(theta)) <= self.band_rad else omega0
        omega = s * omega_mag
        return max(-self.max_vel_abs, min(self.max_vel_abs, omega))

    def _tick(self):
        now = time.monotonic()
        dt = max(0.0, min(now - self.last_time, 0.05))
        self.last_time = now

        f = float(self.get_parameter('frequency_hz').value)  # allow live param change
        self.freq_hz = f

        # Integrate base phase
        omega_base = self._region_speed(self.theta_base, f)
        self.theta_base += omega_base * dt

        # Group phases
        theta_a = self.theta_base                  # motors 1 & 4
        theta_b = self.theta_base + self.group_offset  # motors 2 & 3

        # Compute per-leg velocities
        v1 = self._region_speed(theta_a, f)
        v4 = self._region_speed(theta_a, f)
        v2 = self._region_speed(theta_b, f)
        v3 = self._region_speed(theta_b, f)

        # Publish MIT velocity frames [p, v, kp, kd, tff]
        self._pub(1, v1); self._pub(2, v2); self._pub(3, v3); self._pub(4, v4)

    def _pub(self, idx: int, vel: float):
        msg = Float64MultiArray()
        msg.data = [0.0, float(vel), self.KP, self.KD, 0.0]
        self.cmd_pubs[idx].publish(msg)


def main():
    rclpy.init()
    node = QuadLegFreq()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

