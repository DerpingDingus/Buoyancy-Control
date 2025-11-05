#!/usr/bin/env python3
"""
Main Control Loop for Quadruped (ROS 2 Humble)

Reads joystick input (sensor_msgs/Joy) and commands four motors
via MIT control protocol topics exposed by motor_node.py.

Controls:
- Left stick up/down -> forward/backward velocity
- A button -> re-start motors after shutdown
- B button -> stop motors (sends 'exit' special command)
- Automatically initializes motors on first joystick input

Publishes:
  /motor{1..4}/mit_cmd       (Float64MultiArray [p,v,kp,kd,t])
  /motor{1..4}/special_cmd   (String "start"/"exit"/"zero"/"clear")
Subscribes:
  /motor{1..4}/motor_state   (MotorState)
  /motor{1..4}/error_code    (String)
  /joy                       (sensor_msgs/Joy)
"""

import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray, String
from motor_interfaces.msg import MotorState


class MainControlLoop(Node):
    def __init__(self):
        super().__init__("main_ctrl_node")
        self.get_logger().info("Main Control Node has been started")

        qos_profile = QoSProfile(depth=10)

        # --- Parameters ---
        self.declare_parameter("max_motor_vel", 8.0)
        self.declare_parameter("hz", 20.0)
        self.declare_parameter("motor1_kp", 0.0)
        self.declare_parameter("motor1_kd", 1.0)
        self.declare_parameter("motor2_kp", 0.0)
        self.declare_parameter("motor2_kd", 1.0)

        # Retrieve parameters
        self.max_motor_vel = self.get_parameter("max_motor_vel").value
        self.motor1_kp = self.get_parameter("motor1_kp").value
        self.motor2_kp = self.get_parameter("motor2_kp").value
        self.motor1_kd = self.get_parameter("motor1_kd").value
        self.motor2_kd = self.get_parameter("motor2_kd").value
        self.dt = 1.0 / self.get_parameter("hz").value

        # --- State Variables ---
        self.shutdown_triggered = False
        self.motors_initialized = False

        # per-motor states
        self.motor_states = {i: dict(pos=0.0, vel=0.0, torque=0.0) for i in range(1, 5)}

        # --- ROS Subscribers ---
        self.joystick_subscriber = self.create_subscription(
            Joy, "joy", self.joy_callback, qos_profile
        )

    # ------------------------------------------------------------------
    # Initialization and setup
    # ------------------------------------------------------------------
    def initialize_motors(self):
        qos_profile = QoSProfile(depth=10)
        if self.motors_initialized:
            return
        self.get_logger().info("Initializing Motors...")

        # Publishers
        self.motor_cmd_pubs = {
            i: self.create_publisher(Float64MultiArray, f"/motor{i}/mit_cmd", qos_profile)
            for i in range(1, 5)
        }
        self.motor_special_pubs = {
            i: self.create_publisher(String, f"/motor{i}/special_cmd", qos_profile)
            for i in range(1, 5)
        }

        # Subscribers
        sub_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        for i in range(1, 5):
            self.create_subscription(
                String, f"/motor{i}/error_code",
                lambda msg, i=i: self._error_callback(msg, i),
                sub_qos,
            )
            self.create_subscription(
                MotorState, f"/motor{i}/motor_state",
                lambda msg, i=i: self._state_callback(msg, i),
                sub_qos,
            )

        time.sleep(0.2)
        self.start_motors()
        self.reset_pos()
        self.motors_initialized = True
        self.get_logger().info("Motors started and zeroed.")

    # ------------------------------------------------------------------
    # Joystick callback
    # ------------------------------------------------------------------
    def joy_callback(self, msg: Joy):
        try:
            a_button = msg.buttons[1]  # (A)
            b_button = msg.buttons[2]  # (B)
            left_stick_ud = msg.axes[1]

            if not self.motors_initialized:
                self.initialize_motors()

            if self.shutdown_triggered and a_button:
                self.get_logger().info("A pressed — Restarting motors")
                self.shutdown_triggered = False
                self.start_motors()

            if self.motors_initialized and not self.shutdown_triggered:
                vel = np.clip(left_stick_ud * self.max_motor_vel,
                              -self.max_motor_vel, self.max_motor_vel)

                # mirror left/right sides
                vel1, vel2, vel3, vel4 = vel, -vel, vel, -vel

                cmd_template = lambda v, kp, kd: [0.0, v, kp, kd, 0.0]
                cmd1 = Float64MultiArray(data=cmd_template(vel1, self.motor1_kp, self.motor1_kd))
                cmd2 = Float64MultiArray(data=cmd_template(vel2, self.motor2_kp, self.motor2_kd))
                cmd3 = Float64MultiArray(data=cmd_template(vel3, self.motor1_kp, self.motor1_kd))
                cmd4 = Float64MultiArray(data=cmd_template(vel4, self.motor2_kp, self.motor2_kd))

                for i, msg_cmd in enumerate([cmd1, cmd2, cmd3, cmd4], 1):
                    self.motor_cmd_pubs[i].publish(msg_cmd)

                if b_button:
                    self.get_logger().info("B pressed — Stopping motors")
                    self.kill_motors()
                    self.shutdown_triggered = True

        except Exception as e:
            self.get_logger().error(f"Error in joy_callback: {e}")

    # ------------------------------------------------------------------
    # Motor command helpers
    # ------------------------------------------------------------------
    def start_motors(self):
        msg = String(); msg.data = "start"
        for pub in self.motor_special_pubs.values():
            pub.publish(msg)

    def kill_motors(self):
        msg = String(); msg.data = "exit"
        for pub in self.motor_special_pubs.values():
            pub.publish(msg)

    def reset_pos(self):
        msg = String(); msg.data = "zero"
        for pub in self.motor_special_pubs.values():
            pub.publish(msg)
        self.get_logger().info("Zeroing motor encoders...")

    # ------------------------------------------------------------------
    # State & Error callbacks
    # ------------------------------------------------------------------
    def _state_callback(self, msg: MotorState, idx: int):
        try:
            self.motor_states[idx]["pos"] = msg.abs_position
            self.motor_states[idx]["vel"] = msg.velocity
            self.motor_states[idx]["torque"] = msg.torque
        except Exception as e:
            self.get_logger().error(f"Error updating motor{idx} state: {e}")

    def _error_callback(self, msg: String, idx: int):
        try:
            if "0" not in msg.data:
                self.get_logger().error(f"Motor{idx} error: {msg.data}")
                self.kill_motors()
                self.shutdown_triggered = True
        except Exception as e:
            self.get_logger().error(f"Error parsing motor{idx} error: {e}")


# ----------------------------------------------------------------------
# Main entry
# ----------------------------------------------------------------------
def main():
    rclpy.init()
    node = MainControlLoop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down main control node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

