#!/bin/bash

export HOME=/home/selqie
export USER=selqie
export GPIOZERO_PIN_FACTORY=lgpio 

# Wait until can0 appears in ip link
while ! ip link show can0 > /dev/null 2>&1; do
  echo "Waiting for can0 interface..."
  sleep 2
done

source /opt/ros/jazzy/setup.bash
source /home/selqie/buoyancy/install/setup.bash
source /home/selqie/buoyancy/venv/bin/activate

ros2 launch buoy_control button_control.launch.py 
