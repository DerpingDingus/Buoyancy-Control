#!/usr/bin/env python3
"""Interactive command console tailored for the SELQIE quadruped."""

import math
import threading
import time
from cmd import Cmd
from typing import Iterable, List
import numpy as np

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

from motor_interfaces.msg import MotorState


class MotorConsole(Node):
    """ROS 2 helper that publishes commands and captures motor state."""

    MOTOR_IDS = (1, 2, 3, 4)

    def __init__(self) -> None:
        super().__init__('selqie_motor_console')

        # Publishers
        self._cmd_pubs = {
            motor_id: self.create_publisher(
                Float64MultiArray, f'/motor{motor_id}/mit_cmd', 10
            )
            for motor_id in self.MOTOR_IDS
        }
        self._special_pubs = {
            motor_id: self.create_publisher(
                String, f'/motor{motor_id}/special_cmd', 10
            )
            for motor_id in self.MOTOR_IDS
        }

        # Subscriptions (state + error) with local caches for printing
        self._state_cache: dict[int, MotorState] = {}
        self._error_cache: dict[int, str] = {}
        self._lock = threading.Lock()

        for motor_id in self.MOTOR_IDS:
            self.create_subscription(
                MotorState,
                f'/motor{motor_id}/motor_state',
                lambda msg, mid=motor_id: self._on_state(mid, msg),
                10,
            )
            self.create_subscription(
                String,
                f'/motor{motor_id}/error_code',
                lambda msg, mid=motor_id: self._on_error(mid, msg),
                10,
            )

        # Background executor
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()
        self._shutdown = False

    # ------------------------------------------------------------------
    # ROS callbacks / caches
    # ------------------------------------------------------------------
    def _on_state(self, motor_id: int, msg: MotorState) -> None:
        with self._lock:
            self._state_cache[motor_id] = msg

    def _on_error(self, motor_id: int, msg: String) -> None:
        with self._lock:
            self._error_cache[motor_id] = msg.data

    # ------------------------------------------------------------------
    # Command helpers
    # ------------------------------------------------------------------
    def send_special(self, command: str, targets: Iterable[int]) -> None:
        msg = String()
        msg.data = command
        for motor_id in targets:
            pub = self._special_pubs.get(motor_id)
            if pub:
                pub.publish(msg)

    def send_cmd(self, target: int, position: float, velocity: float, kp: float, kd: float, torque: float) -> None:
        msg = Float64MultiArray()
        msg.data = [float(position), float(velocity), float(kp), float(kd), float(torque)]
        pub = self._cmd_pubs.get(target)
        if pub:
            pub.publish(msg)

    def send_velocity(self, targets: Iterable[int], velocity: float, kp: float = 0.0, kd: float = 1.0, torque: float = 0.0) -> None:
        for motor_id in targets:
            self.send_cmd(motor_id, 0.0, velocity, kp, kd, torque)

    def snapshot_states(self) -> dict[int, MotorState]:
        with self._lock:
            return dict(self._state_cache)

    def snapshot_errors(self) -> dict[int, str]:
        with self._lock:
            return dict(self._error_cache)

    def shutdown(self) -> None:
        if self._shutdown:
            return
        self._shutdown = True
        self._executor.shutdown()
        self.destroy_node()
        self._spin_thread.join(timeout=1.0)
        

#######################################################
########### BEUHLER CLOCK IMPLEMENTATION ##############
#######################################################

## Helper Functions
def _wrap_to_pi(x: float) -> float:
    return (x + math.pi) % (2.0 * math.pi) - math.pi


def _sgn(x: float) -> float:
    return -1.0 if x < 0 else (1.0 if x > 0 else 0.0)


## Beuhler Class
class BeuhlerClock:
    """Threaded implementation of the Beuhler clock velocity pattern."""

    def __init__(self, console: MotorConsole):
        self._console = console

        # Tunables (defaults mirror the standalone beuhler_clock node)
        self.gait_frequency_hz = 0.5 # 1 / total gait period
        self.alpha = 6.0 # how much faster fast portion is
        self.fast_band_deg = 30.0 # d_theta in stance

        # Constants
        self.group_offset_deg = 180.0
        self.control_hz = 100.0
        self.kp = 0.0
        self.kd = 1.0
        self.max_vel_abs = 2.0
        
        # Internal state
        self._theta_base = 0.0
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._cfg_lock = threading.Lock()

    # ---- core math ---------------------------------------------------
    def _calcOmegaSlow(self, f_abs: float, alpha: float) -> float:
        return (f_abs / (2 * math.pi * (1.0 + alpha)))

    def _region_speed(self, theta: float, f_hz: float, fast_band_deg: float, alpha: float) -> float:
        if f_hz == 0.0:
            return 0.0
     
        omegaSlow = self._calcOmegaSlow(abs(f_hz), alpha)
        if abs(_wrap_to_pi(theta) <= math.radians(fast_band_deg)):
            omega_mag = alpha * omegaSlow
        else:
            omega_mag = omegaSlow
            
        if omega_mag <= self.max_vel_abs:
            return _sgn(f_hz) * omega_mag
        else:
            return _sgn(f_hz) * self.max_vel_abs

    # ---- lifecycle ---------------------------------------------------
    def _apply_config(
        self,
        *,
        gait_frequency_hz: float | None = None,
        alpha: float | None = None,
        fast_band_deg: float | None = None,
    ) -> None:
        with self._cfg_lock:
            if gait_frequency_hz is not None:
                self.gait_frequency_hz = gait_frequency_hz
            if alpha is not None:
                self.alpha = alpha
            if fast_band_deg is not None:
                self.fast_band_deg = fast_band_deg
                
    def start(
        self,
        gait_frequency_hz: float | None = None,
        alpha: float | None = None,
        fast_band_deg: float | None = None,
    ) -> None:
        self._apply_config(
            gait_frequency_hz=gait_frequency_hz,
            alpha = alpha,
            fast_band_deg=fast_band_deg
        )

        if self._thread and self._thread.is_alive():
            # Update in-place without phase reset for real-time tuning.
            return

        self._stop_event.clear()
        self._theta_base = 0.0
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=1.0)
        self._thread = None

    def is_running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    # ---- runner ------------------------------------------------------
    def _run(self) -> None:
        last_time = time.monotonic()

        while not self._stop_event.is_set():
            with self._cfg_lock:
                gait_frequency_hz = self.gait_frequency_hz
                alpha = self.alpha
                fast_band_deg = self.fast_band_deg

            dt = 1.0 / max(1e-6, self.control_hz)
            now = time.monotonic()
            step = max(0.0, min(now - last_time, 0.05))
            last_time = now

            f = gait_frequency_hz
            omega_base = self._region_speed(self._theta_base, f, fast_band_deg, alpha)
            self._theta_base += omega_base * step

            theta_a = self._theta_base
            theta_b = self._theta_base + math.radians(self.group_offset_deg)
            
            vA = self._region_speed(theta_a, f, fast_band_deg, alpha)
            vB = self._region_speed(theta_b, f, fast_band_deg, alpha)
            
            motorOrder = np.array([1, 4, 2, 3])
            
            for i, motor in enumerate(motorOrder):
                if i <= 2:
                    curV = vA
                else: 
                    curV = vB
                self._console.send_velocity((motor,), curV, kp=self.kp, kd=self.kd)

            sleep_time = dt - (time.monotonic() - now)
            if sleep_time > 0:
                time.sleep(sleep_time)


#######################################################
###################### TERMINAL #######################
#######################################################

class SELQIETerminal(Cmd):
    """Cmd-based shell that speaks directly to the quad_legs motor topics."""

    intro = 'Welcome to the SELQIE terminal. Type help or ? to list commands.\n'
    prompt = 'SELQIE> '

    def __init__(self) -> None:
        super().__init__()
        self._console = MotorConsole()
        self._beuhler = BeuhlerClock(self._console)

    # ---- helpers ------------------------------------------------------
    def _parse_targets(self, text: str, default_all: bool = False) -> List[int]:
        tokens = text.split()
        if not tokens:
            return list(MotorConsole.MOTOR_IDS) if default_all else []

        token = tokens[0].lower()
        if token in ('*', 'all'):
            return list(MotorConsole.MOTOR_IDS)

        try:
            motor_id = int(token)
        except ValueError:
            print("Motor id must be 1-4, 'all', or '*'")
            return []

        if motor_id not in MotorConsole.MOTOR_IDS:
            print('Motor id must be between 1 and 4')
            return []
        return [motor_id]

    # ---- lifecycle ----------------------------------------------------
    def do_exit(self, line: str) -> bool:
        """Exit the terminal."""
        print('Exiting...')
        self._beuhler.stop()
        self._console.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
        return True

    # ---- special commands --------------------------------------------
    def do_start_motors(self, line: str) -> None:
        """Send the 'start' special command. Usage: start_motors [motor_id|all]"""
        targets = self._parse_targets(line, default_all=True)
        if targets:
            self._console.send_special('start', targets)

    def do_stop_motors(self, line: str) -> None:
        """Send the 'exit' special command to stop MIT mode. Usage: stop_motors [motor_id|all]"""
        targets = self._parse_targets(line, default_all=True)
        if targets:
            self._console.send_special('exit', targets)

    def do_zero(self, line: str) -> None:
        """Zero encoders. Usage: zero [motor_id|all]"""
        targets = self._parse_targets(line, default_all=True)
        if targets:
            self._console.send_special('zero', targets)

    def do_clear(self, line: str) -> None:
        """Clear commands and hold zeros. Usage: clear [motor_id|all]"""
        targets = self._parse_targets(line, default_all=True)
        if targets:
            self._console.send_special('clear', targets)

    def do_beuhler(self, line: str) -> None:
        """Start or stop the Beuhler clock pattern.

        Usage:
          beuhler stop
          beuhler <frequency_hz> [group_offset_deg] [alpha]
        """

        parts = line.split()
        if not parts:
            print('Usage: beuhler stop | <frequency_hz> [fast_band_deg] [alpha]')
            return

        if parts[0].lower() == 'stop':
            if self._beuhler.is_running():
                self._beuhler.stop()
                self.do_stop_motors("All")
                print('Beuhler clock and motors stopped.')
            else:
                print('Beuhler clock was not running.')
            return

        try:
            values = [float(p) for p in parts]
        except ValueError:
            print('All parameters must be numeric. Usage: beuhler <frequency_hz> [fast_band_deg] [alpha]')
            return

        params = {
            'gait_frequency_hz': values[0],
            'fast_band_deg': values[1] if len(values) > 1 else None,
            'alpha': values[2] if len(values) > 2 else None,
        }

        was_running = self._beuhler.is_running()
        self._beuhler.start(**params)
        print(
            ('Beuhler clock updated: ' if was_running else 'Beuhler clock running: '),
            f"f={self._beuhler.gait_frequency_hz:+.3f} Hz, "
            f"band=±{self._beuhler.fast_band_deg:.1f}°, alpha={self._beuhler.alpha}"
        )

    # ---- MIT command helpers -----------------------------------------
    def do_set_cmd(self, line: str) -> None:
        """Send a full MIT command. Usage: set_cmd <motor_id|all> <p> <v> <kp> <kd> <torque>"""
        parts = line.split()
        if len(parts) != 6:
            print('Usage: set_cmd <motor_id|all> <p> <v> <kp> <kd> <torque>')
            return

        targets = self._parse_targets(parts[0])
        if not targets:
            return
        try:
            p_val, v_val, kp_val, kd_val, t_val = map(float, parts[1:])
        except ValueError:
            print('Position, velocity, kp, kd, and torque must be numeric')
            return

        for target in targets:
            self._console.send_cmd(target, p_val, v_val, kp_val, kd_val, t_val)

    def do_set_vel(self, line: str) -> None:
        """Convenience velocity command. Usage: set_vel <motor_id|all> <vel> [kp] [kd] [torque]"""
        parts = line.split()
        if len(parts) < 2:
            print('Usage: set_vel <motor_id|all> <vel> [kp] [kd] [torque]')
            return

        targets = self._parse_targets(parts[0])
        if not targets:
            return

        try:
            vel = float(parts[1])
            kp = float(parts[2]) if len(parts) > 2 else 0.0
            kd = float(parts[3]) if len(parts) > 3 else 1.0
            torque = float(parts[4]) if len(parts) > 4 else 0.0
        except ValueError:
            print('Velocity, kp, kd, and torque must be numeric')
            return

        self._console.send_velocity(targets, vel, kp=kp, kd=kd, torque=torque)

    # ---- inspection ---------------------------------------------------
    def do_status(self, line: str) -> None:
        """Print the latest motor states received."""
        states = self._console.snapshot_states()
        if not states:
            print('No motor state messages received yet.')
            return

        for motor_id in sorted(states):
            state = states[motor_id]
            print(
                f"motor{motor_id}: pos={state.position:.3f} rad, "
                f"abs={state.abs_position:.3f} rad, vel={state.velocity:.3f} rad/s, "
                f"torque={state.torque:.3f} Nm, current={state.current:.3f} A, "
                f"temp={state.temperature} C"
            )

    def do_errors(self, line: str) -> None:
        """Print the latest error strings from each motor."""
        errors = self._console.snapshot_errors()
        if not errors:
            print('No error messages received yet.')
            return

        for motor_id in sorted(errors):
            print(f'motor{motor_id}: {errors[motor_id]}')


def main(argv: list[str] | None = None) -> int:
    rclpy.init(args=argv)
    terminal = SELQIETerminal()
    try:
        terminal.cmdloop()
    finally:
        terminal._console.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':  # pragma: no cover - CLI entry point
    raise SystemExit(main())
