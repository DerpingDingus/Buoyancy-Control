#!/usr/bin/env python3
"""Interactive command console tailored for the SELQIE quadruped."""

import threading
from cmd import Cmd
from typing import Iterable, List

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


class SELQIETerminal(Cmd):
    """Cmd-based shell that speaks directly to the quad_legs motor topics."""

    intro = 'Welcome to the SELQIE terminal. Type help or ? to list commands.\n'
    prompt = 'SELQIE> '

    def __init__(self) -> None:
        super().__init__()
        self._console = MotorConsole()

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
