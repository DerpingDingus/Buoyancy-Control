source install/setup.bash


ros2 topic pub /joint1/servo_cmd std_msgs/msg/Float64MultiArray "{data: [4.0, 270.0, 0.0, 0.0]}"
