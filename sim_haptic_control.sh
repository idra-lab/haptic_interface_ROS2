#!/bin/bash
colcon build --packages-select haptic_control
. install/local_setup.bash 
ros2 launch haptic_control sim_haptic_control.launch.py haption_ws_path:=/home/robotics/haption_ws/
