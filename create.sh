#!/bin/bash
source /opt/ros/jazzy/setup.bash

ros2 pkg create buoy_control \
  --build-type ament_python \
  --dependencies rclpy std_msgs motor_interfaces \
  --description "ROS 2 node that controls motors using buttons" \
  --license Apache-2.0
