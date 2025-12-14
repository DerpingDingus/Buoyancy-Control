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

# Or through the included launch file (works great over SSH as well)
ros2 launch selqie_tmux_ui tmux_session.launch.py
```

> **Note**
> The launch file explicitly requests a pseudo-TTY so you can start the UI from
> an SSH session and immediately land inside the tmux panes. In the rare case a
> supervisor refuses to allocate a TTY, the UI still prints the exact
> `tmux attach -t <session>` command so you can connect manually.

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
* Drive the robot from the right pane using the interactive `selqie_terminal`
  shell. The console now talks directly to the `/motor{1..4}` topics provided
  by `quad_legs`, so you can:
  - `start_motors`, `stop_motors`, `zero`, or `clear` (optionally targeting a
    single motor or `all`).
  - Send MIT commands with `set_cmd <motor|all> <p> <v> <kp> <kd> <torque>`.
  - Send quick velocity commands with `set_vel <motor|all> <vel> [kp] [kd]
    [torque]`.
  - Inspect feedback with `status` (latest MotorState) and `errors` (last error
    string per motor).
  The tool exits cleanly with `exit`. If ROS 2 is unavailable in the environment,
  the pane falls back to a standard Bash shell so you can still run manual
  commands like `ros2 topic echo`.
* Detach from tmux with `Ctrl+b d` and reattach with `tmux attach -t selqie_ui`.
