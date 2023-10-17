#!/bin/bash
colcon build --packages-select haption_raptor_api
. install/setup.bash 
ros2 run haption_raptor_api raptor_api_wrapper 
