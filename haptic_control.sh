#!/bin/bash
clear
colcon build --packages-select haptic_control
. install/local_setup.bash 
ros2 launch haptic_control haptic_control.launch.py haption_ws_path:=$(pwd)/
