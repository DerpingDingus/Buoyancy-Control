# SELQIE tmux UI

This ROS 2 Humble Python package creates a repeatable tmux layout that starts all
quadruped control nodes on the left pane (via `ros2 launch quad_legs
quad_full.launch.py`) and exposes a dedicated console pane on the right for
manual commands.

## Prerequisites

Install tmux (the UI backend) if it is not already available:

```bash
sudo apt update && sudo apt install tmux
# or via rosdep
rosdep install selqie_tmux_ui
```

## Usage

```bash
# Launch directly as a ROS node
ros2 run selqie_tmux_ui tmux_ui

# Or through the included launch file
ros2 launch selqie_tmux_ui tmux_session.launch.py
```

> **Note**
> `ros2 launch` (and other supervisor tools) often run without an interactive
> TTY. The UI automatically stays detached in that scenario and prints the
> exact `tmux attach -t <session>` command so you can connect from a real
> terminal.

Useful command-line flags:

* `--session-name` – tmux session identifier (defaults to `selqie_ui`).
* `--launch-command` – shell command for the left pane; override to use a
  different launch file or composed command.
* `--console-command` – optional custom right pane command. Leave empty to get
  an interactive Bash shell with quick instructions.
* `--recreate` – kill any existing session with the same name before building
  the layout.
* `--detach` – start everything but leave tmux detached.

Once the layout is up you can:

* Watch the left pane bring up all motors, UI, and joystick nodes via the
  existing `quad_legs` launch file.
* Type arbitrary ROS 2 commands on the right pane (e.g. `ros2 topic echo`,
  `ros2 action send_goal`, etc.).
* Detach from tmux with `Ctrl+b d` and reattach with `tmux attach -t selqie_ui`.
