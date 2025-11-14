#!/usr/bin/env python3
"""Utility to spin up a tmux session that launches SELQIE nodes."""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
import textwrap
from typing import Optional

DEFAULT_LAUNCH_COMMAND = 'ros2 launch quad_legs quad_full.launch.py'


def _default_console_command() -> str:
    banner = textwrap.dedent(
        """
        SELQIE command console ready.\n\n"
        "Use this pane to send additional ROS 2 commands (ros2 topic pub, ros2 action send_goal, etc.).\n"
        "Detach from the session with Ctrl+b d, or exit the pane with Ctrl+D.\n"
        """
    ).strip()
    escaped = banner.replace("'", "'\\''")
    return f"printf '{escaped}\n'; exec bash -i"


def _run_tmux_command(args: list[str]) -> None:
    subprocess.run(['tmux', *args], check=True)


def _session_exists(name: str) -> bool:
    result = subprocess.run(
        ['tmux', 'has-session', '-t', name],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    return result.returncode == 0


def _ensure_tmux_available() -> None:
    """Ensure tmux exists before attempting to launch anything."""

    if shutil.which('tmux') is None:
        raise RuntimeError(
            'tmux is not installed or not on PATH. Install it with "sudo apt install tmux" '
            'or run "rosdep install selqie_tmux_ui" before launching the UI.'
        )


def _stdio_supports_attach() -> bool:
    """Return True if stdin/stdout look like interactive terminals."""

    return sys.stdin.isatty() and sys.stdout.isatty()


def launch_tmux(session_name: str, left_command: str, right_command: str, recreate: bool, attach: bool) -> None:
    _ensure_tmux_available()

    if _session_exists(session_name):
        if recreate:
            _run_tmux_command(['kill-session', '-t', session_name])
        else:
            print(f"Attaching to existing tmux session '{session_name}'.")
            if attach:
                if _stdio_supports_attach():
                    _run_tmux_command(['attach-session', '-t', session_name])
                else:
                    print(
                        'Current environment has no interactive TTY; cannot attach automatically.\n'
                        f"Attach manually with: tmux attach -t {session_name}"
                    )
            return

    # left pane - launch command
    _run_tmux_command([
        'new-session', '-d', '-s', session_name, 'bash', '-lc', left_command,
    ])

    # right pane - interactive console (45% of window width)
    _run_tmux_command([
        'split-window', '-h', '-p', '45', '-t', f'{session_name}:0.0',
        'bash', '-lc', right_command,
    ])

    _run_tmux_command(['select-pane', '-t', f'{session_name}:0.0'])

    if attach:
        if _stdio_supports_attach():
            _run_tmux_command(['attach-session', '-t', session_name])
        else:
            print(
                'Current environment has no interactive TTY; leaving the tmux session detached.\n'
                f"Attach manually with: tmux attach -t {session_name}"
            )
            print(f"tmux session '{session_name}' started in detached mode.")
    else:
        print(f"tmux session '{session_name}' started in detached mode.")


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Start a tmux layout that launches SELQIE quadruped nodes.',
    )
    parser.add_argument(
        '--session-name',
        default='selqie_ui',
        help='Name of the tmux session to create or attach to.',
    )
    parser.add_argument(
        '--launch-command',
        default=DEFAULT_LAUNCH_COMMAND,
        help='Command executed in the left pane to start all nodes.',
    )
    parser.add_argument(
        '--console-command',
        default=None,
        help='Command executed in the right pane (defaults to an interactive bash console).',
    )
    parser.add_argument(
        '--recreate',
        action='store_true',
        help='Kill any existing tmux session with the same name before starting a new layout.',
    )
    parser.add_argument(
        '--detach',
        action='store_true',
        help='Do not automatically attach to the tmux session after creating it.',
    )
    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> int:
    args = parse_args(argv)
    right_command = args.console_command or _default_console_command()

    try:
        launch_tmux(
            session_name=args.session_name,
            left_command=args.launch_command,
            right_command=right_command,
            recreate=args.recreate,
            attach=not args.detach,
        )
    except (subprocess.CalledProcessError, RuntimeError) as exc:  # pragma: no cover - CLI path
        print(f'Failed to start tmux UI: {exc}', file=sys.stderr)
        return 1

    return 0


if __name__ == '__main__':  # pragma: no cover - CLI entry point
    sys.exit(main())
