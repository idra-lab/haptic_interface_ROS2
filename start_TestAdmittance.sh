#!/bin/bash
colcon build --packages-select test_admittance
. install/local_setup.bash 
ros2 run test_admittance test_admittance --ros-args --params-file /home/robotics/haption_ws/src/test_admittance/parameters.yaml
