#!/bin/bash
colcon build --packages-select test_calibration
. install/local_setup.bash 
ros2 run test_calibration test_calibration --ros-args --params-file ./src/test_calibration/config/parameters.yaml
