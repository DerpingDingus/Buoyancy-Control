# Buoyancy Control

A repository for the motor control and sensor packages used in the Buoyancy Control project. Useful for those controlling CubeMars AK-series motors with Raspberry Pis using an RS485 CAN Hat. 

## Repository layout

| Path | Type | Purpose |
| ---- | ---- | ------- |
| `servo/` | Python ROS 2 package | Controls Motors. |
| `buoy_control/` | Python ROS 2 package | Houses the control nodes for the motors |
| `sensors/` | Python ROS 2 package | Houses all of the sensing nodes |

## Prerequisites

* Ubuntu 22.04 with ROS 2 Jazzy (desktop or ros-base install).
* Updated Python3 and Python can dependencies
* A CAN interface configured as `can0` (e.g., `sudo ip link set can0 up type can bitrate 1000000`).

## Core packages

Servo: Holds the actual code to turn positional, velocity, and acceleration commands into movement of the motor

Buoy Control: Holds the code that sends positional, velocity, and acceleration commands to the servo package

Sensors: Holds the code for the leak sensors used in the project, as well as code to detect the current and torque being used by each motor.

## License

Each package keeps its own license (MIT for `quad_legs` and `ws2812b_ros`,
Apache-2.0 for `motor_interfaces`, vendor licenses for the submodules). See the
respective subdirectories for details.
