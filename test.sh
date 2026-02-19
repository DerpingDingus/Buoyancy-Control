source install/setup.bash

ros2 run servo servo_motor_node --ros-args \
  -p can_interface:=can0 \
  -p can_id:=2 \
  -p motor_type:=AK40-10 \
  -p control_hz:=20.0 \
  -p joint_name:=joint1 \
  -p auto_start:=true \
  -p reverse_polarity:=false
