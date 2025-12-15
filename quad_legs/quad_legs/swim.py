#!/usr/bin/env python3
"""
J. Vranicar
Beuhler Clock control of legs

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
        super().__init__('quad_leg_freq') ######## SHOULD THIS BE SOMETHING ELSE?

        # ---- Parameters ----
        self.declare_parameter('frequency_hz', 0.5)       # average frequency (±)
        self.declare_parameter('sweep_angle_deg', 180.0) # phase offset between (1&4) and (2&3)
        self.declare_parameter('centerline_angle_Q3_deg', 30.0)     # +/- around "down"
        self.declare_parameter('kp', 0.0)
        self.declare_parameter('kd', 1.0)
        self.declare_parameter('max_vel_abs', 30.0)       # rad/s clamp
        self.declare_parameter('control_hz', 100.0)       # publish rate
        self.declare_parameter('auto_start', True)        # send "start" once
        self.declare_parameter('auto_zero', False)        # optionally "zero" once

        self.freq_hz = float(self.get_parameter('frequency_hz').value)
        self.sweep_angle = math.radians(float(self.get_parameter('sweep_angle_deg').value))
        self.centerline = math.radians(float(self.get_parameter('centerline_angle_Q3_deg').value))
        self.KP = float(self.get_parameter('kp').value)
        self.KD = float(self.get_parameter('kd').value)
        self.max_vel_abs = float(self.get_parameter('max_vel_abs').value)
        hz = float(self.get_parameter('control_hz').value)
        self.dt = 1.0 / max(1e-6, hz)
        self.cur_direction = 1

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
    def _omegaRange(self) -> tuple[float, float]:
        # return (31.0 / 36.0) * 2.0 * math.pi * f_abs, 1
        midline = self.centerline
        max_theta = midline + self.sweep_angle / 2
        min_theta = midline - self.sweep_angle / 2
        return min_theta, max_theta

    def _tick(self):
        # For now, going to program this as if there is no p measurement. Fix later
        now = time.monotonic()
        dt = max(0.0, min(now - self.last_time, 0.05))
        self.last_time = now

        f = float(self.get_parameter('frequency_hz').value)  # allow live param change
        self.freq_hz = f
        
        cur_omega = self.freq_hz * self.cur_direction
        self.theta_base += cur_omega * dt
        
        # Check if we've made it
        theta = self.theta_base
        min_theta, max_theta = self._omegaRange()
    
        # Compute per-leg velocities
        for i in range(1, 5):
            self._pub(i, cur_omega)
            
        # Now, check if we need to flip directions for next time
        # CW and CCW from the perspective of looking at starboard side, selqie moving to the right
        # -1 is CW 1 is CCW
        if self.cur_direction == 0:
            des_theta = min_theta
        else:
            des_theta = max_theta

        # If close enough, flip directions
        if abs(theta - des_theta) < 2:
            self.cur_direction *= -1
        
        

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

