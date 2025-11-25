#!/usr/bin/env python3
"""
Quad Leg UI (ROS 2 Humble + PyQt5)

- Select a rotation frequency (Hz, negative reverses direction) for all four legs.
- Inside [-30°, +30°] about "down" (0°), legs move 6× faster than outside that band.
- Motors 1 & 4 share a phase; Motors 2 & 3 share a phase; the two groups are phase-offset
  by a user-set value (default 180°), so 1&4 are equally offset from 2&3.

Publishes velocity-only MIT commands (p=0, kp=0, kd=1, t=0) to:
  /motor{1..4}/mit_cmd

Sends special commands ("start", "exit", "zero") to:
  /motor{1..4}/special_cmd

Requires:
  sudo apt install python3-pyqt5
"""

import os
import sys, math, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from PyQt5 import QtCore, QtWidgets


# ------------------------------ Helpers ---------------------------------

def wrap_to_pi(x):
    """Wrap angle (rad) to [-pi, pi]."""
    return (x + math.pi) % (2.0 * math.pi) - math.pi

def sgn(x):
    return -1.0 if x < 0 else (1.0 if x > 0 else 0.0)


# ------------------------------ ROS Node --------------------------------

class LegCommandPublisher(Node):
    """
    Publishes MIT velocity commands to four motors, computing per-leg velocity
    from a non-uniform speed profile over angle:
      - fast region: |theta| <= 30°  -> 6 * ω0
      - outside     : else           -> 1 * ω0
    with ω0 chosen so the average cycle frequency equals the selected f (Hz).

    Phase layout:
      Group A: motors 1 & 4  (phase = theta_base)
      Group B: motors 2 & 3  (phase = theta_base + group_offset_rad)
    """
    def __init__(self):
        super().__init__('quad_leg_ui_node')

        # Publishers
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

        # MIT control constants for velocity mode
        self.KP = 0.0
        self.KD = 1.0
        self.TFF = 0.0
        self.POS_HOLD = 0.0  # position term ignored when KP=0

        # Speed-profile band (±30°)
        self.band_rad = math.radians(30.0)

        # Timing / phase state
        self.theta_base = 0.0
        self.last_time = time.monotonic()

        # User-controlled parameters
        self.freq_hz = 0.5
        self.group_offset_rad = math.radians(180.0)  # 1&4 vs 2&3

        # Safety clamp for rad/s
        self.max_vel_abs = 30.0

        # Run at 100 Hz for smooth updates
        self.timer = self.create_timer(0.01, self._tick)

        # Pause flag (Stop button sets it)
        self.paused = False

    # ---------- External setters (called from UI) ----------
    def set_frequency_hz(self, f_hz: float):
        self.freq_hz = float(f_hz)

    def set_group_offset_deg(self, deg: float):
        self.group_offset_rad = math.radians(float(deg))

    def set_paused(self, paused: bool):
        self.paused = bool(paused)

    # ---------- Special cmds ----------
    def send_special(self, text: str):
        msg = String(); msg.data = text
        for pub in self.special_pubs.values():
            pub.publish(msg)

    # ---------- Core kinematics/velocity profile ----------
    def _omega0_for_average_frequency(self, f_hz_abs: float) -> float:
        """
        Choose ω0 so that the *average* rotation frequency equals |f|.
        Derivation:
         - fast angle = π/3  (±30° total)
         - slow angle = 5π/3
         - ω_fast = 6 ω0, ω_slow = ω0
         - T = (slow/ω0) + (fast/(6ω0)) = (31π/18)/ω0
         - avg_omega = 2π / T = (36/31) * ω0
         => ω0 = (31/36) * 2π f
        """
        return (31.0 / 36.0) * 2.0 * math.pi * f_hz_abs

    def _region_speed(self, theta: float, f_hz: float) -> float:
        """Compute ω(θ) with correct sign and 6×/1× scaling."""
        if f_hz == 0.0:
            return 0.0
        s = sgn(f_hz)
        f_abs = abs(f_hz)
        omega0 = self._omega0_for_average_frequency(f_abs)
        theta_wrapped = wrap_to_pi(theta)
        omega_mag = (6.0 * omega0) if abs(theta_wrapped) <= self.band_rad else omega0
        omega = s * omega_mag
        # Safety clamp
        if abs(omega) > self.max_vel_abs:
            omega = self.max_vel_abs * s
        return omega

    def _tick(self):
        """Integrate base phase and publish per-leg velocities."""
        now = time.monotonic()
        dt = max(0.0, min(now - self.last_time, 0.05))
        self.last_time = now

        # If paused, publish zeros and don't advance phase
        if self.paused or abs(self.freq_hz) < 1e-9:
            self._publish_all(0.0, 0.0, 0.0, 0.0)
            return

        # Advance base phase using its own region speed
        omega_base = self._region_speed(self.theta_base, self.freq_hz)
        self.theta_base += omega_base * dt

        # Group phases
        theta_a = self.theta_base                      # motors 1 & 4
        theta_b = self.theta_base + self.group_offset_rad  # motors 2 & 3

        # Compute per-leg velocities from their phase (region decision)
        v1 = self._region_speed(theta_a, self.freq_hz)
        v4 = self._region_speed(theta_a, self.freq_hz)
        v2 = self._region_speed(theta_b, self.freq_hz)
        v3 = self._region_speed(theta_b, self.freq_hz)

        self._publish_all(v1, v2, v3, v4)

    def _publish_all(self, v1, v2, v3, v4):
        self._publish(1, v1)
        self._publish(2, v2)
        self._publish(3, v3)
        self._publish(4, v4)

    def _publish(self, motor_idx: int, vel_rad_s: float):
        msg = Float64MultiArray()
        msg.data = [self.POS_HOLD, float(vel_rad_s), self.KP, self.KD, self.TFF]
        pub = self.cmd_pubs.get(motor_idx)
        if pub:
            pub.publish(msg)


# ------------------------------ Qt UI -----------------------------------

class MainWindow(QtWidgets.QWidget):
    def __init__(self, ros_node: LegCommandPublisher):
        super().__init__()
        self.setWindowTitle("Quadruped Leg Frequency UI")
        self.ros = ros_node

        # --- Widgets ---
        self.freq_spin = QtWidgets.QDoubleSpinBox()
        self.freq_spin.setDecimals(3)
        self.freq_spin.setRange(-3.0, 3.0)   # Hz
        self.freq_spin.setSingleStep(0.05)
        self.freq_spin.setValue(0.5)

        self.freq_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.freq_slider.setRange(-300, 300)  # maps to -3.00 .. +3.00 Hz
        self.freq_slider.setValue(int(self.freq_spin.value() * 100))

        self.offset_spin = QtWidgets.QDoubleSpinBox()
        self.offset_spin.setDecimals(1)
        self.offset_spin.setRange(0.0, 360.0)  # degrees between groups
        self.offset_spin.setSingleStep(5.0)
        self.offset_spin.setValue(180.0)

        self.status_label = QtWidgets.QLabel("Ready")

        self.start_btn = QtWidgets.QPushButton("Start (send 'start')")
        self.stop_btn  = QtWidgets.QPushButton("Stop (send 'exit')")
        self.zero_btn  = QtWidgets.QPushButton("Zero encoders")

        self.pause_chk = QtWidgets.QCheckBox("Pause output (hold zero)")

        # --- Layout ---
        form = QtWidgets.QFormLayout()
        form.addRow("Frequency (Hz):", self.freq_spin)
        form.addRow("Frequency Slider:", self.freq_slider)
        form.addRow("Group Offset (deg)  (1&4 vs 2&3):", self.offset_spin)

        btns = QtWidgets.QHBoxLayout()
        btns.addWidget(self.start_btn)
        btns.addWidget(self.stop_btn)
        btns.addWidget(self.zero_btn)
        btns.addWidget(self.pause_chk)

        v = QtWidgets.QVBoxLayout(self)
        v.addLayout(form)
        v.addLayout(btns)
        v.addWidget(self.status_label)

        # --- Signals ---
        self.freq_spin.valueChanged.connect(self._on_freq_spin)
        self.freq_slider.valueChanged.connect(self._on_freq_slider)
        self.offset_spin.valueChanged.connect(self._on_offset)
        self.start_btn.clicked.connect(self._on_start)
        self.stop_btn.clicked.connect(self._on_stop)
        self.zero_btn.clicked.connect(self._on_zero)
        self.pause_chk.stateChanged.connect(self._on_pause_changed)

        # Initialize ROS side
        self._apply_freq(self.freq_spin.value())
        self._apply_offset(self.offset_spin.value())

        # UI timer to refresh status text
        self.ui_timer = QtCore.QTimer(self)
        self.ui_timer.timeout.connect(self._refresh_status)
        self.ui_timer.start(100)

        # Periodically spin ROS so timers fire
        self.ros_timer = QtCore.QTimer(self)
        self.ros_timer.timeout.connect(lambda: rclpy.spin_once(self.ros, timeout_sec=0.0))
        self.ros_timer.start(10)  # ~100 Hz

    # ---- Slots ----
    def _on_freq_spin(self, val):
        self.freq_slider.blockSignals(True)
        self.freq_slider.setValue(int(val * 100))
        self.freq_slider.blockSignals(False)
        self._apply_freq(val)

    def _on_freq_slider(self, sval):
        val = sval / 100.0
        self.freq_spin.blockSignals(True)
        self.freq_spin.setValue(val)
        self.freq_spin.blockSignals(False)
        self._apply_freq(val)

    def _on_offset(self, deg):
        self._apply_offset(deg)

    def _on_start(self):
        self.ros.send_special("start")

    def _on_stop(self):
        self.ros.send_special("exit")

    def _on_zero(self):
        self.ros.send_special("zero")

    def _on_pause_changed(self, state):
        self.ros.set_paused(state == QtCore.Qt.Checked)

    # ---- Helpers ----
    def _apply_freq(self, f_hz):
        self.ros.set_frequency_hz(float(f_hz))

    def _apply_offset(self, deg):
        self.ros.set_group_offset_deg(float(deg))

    def _refresh_status(self):
        f = self.ros.freq_hz
        off = math.degrees(self.ros.group_offset_rad)
        f_abs = abs(f)
        if f_abs > 0.0:
            omega0 = self.ros._omega0_for_average_frequency(f_abs)
            omega_fast = 6.0 * omega0
            omega_slow = omega0
        else:
            omega_fast = omega_slow = 0.0

        status = (
            f"freq = {f:+.3f} Hz | group offset = {off:.1f}° | "
            f"ω_slow = {omega_slow:+.2f} rad/s | ω_fast = {omega_fast:+.2f} rad/s | "
            f"{'PAUSED' if self.ros.paused else 'RUNNING'}"
        )
        self.status_label.setText(status)


# ------------------------------ Main ------------------------------------

def main():
    # Ensure Qt can start even in headless sessions (e.g., tmux without DISPLAY)
    if not os.environ.get("DISPLAY") and not os.environ.get("QT_QPA_PLATFORM"):
        os.environ["QT_QPA_PLATFORM"] = "offscreen"

    rclpy.init()
    node = LegCommandPublisher()

    app = QtWidgets.QApplication(sys.argv)
    win = MainWindow(node)
    win.resize(640, 200)
    win.show()

    try:
        sys.exit(app.exec_())
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

