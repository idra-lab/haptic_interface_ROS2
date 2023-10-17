This project includes a basic implementation of the test programs (TestCalibration, TestImpedance and TestAdmittance) using ROS2 and RaptorAPI.
This guide is written for Linux, Windows users need to adapt it in the appropriate way.

## Installation and setup
1. Installation of ros2: 
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
2. Installation of colcon: 
https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html#install-colcon
3. Clone as a workspace:
If you want to follow the ros2 way for creating a workspace, you can clone this repository under ~/ros2_ws/, or visit the documentation page for creating a worspace https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html
4. RaptorAPI headers are already included under src/haption_raptor_api/Dependencies/RaptorAPI
## Usage
0. For newbies: run "local_setup.bash" first! (typically under "/opt/ros/<version>")
1. Copy the RaptorAPI shared libraries to "src/haption_raptor_api/Dependencies/RaptorAPI/bin/Linux/glibc-<version>"
2. Make sure that the ".param" file for your device is accessible
3. Prepare the raptor_api_interfaces:
 colcon build --packages-select raptor_api_interfaces
4. Set LD_LIBRARY_PATH so that it points to "src/haption_raptor_api/Dependencies/RaptorAPI/bin/Linux/glibc-<version>"
5. Run "local_setup.bash" again
6. Start the RaptorAPIWrapper node by calling ./start_RaptorAPIWrapper.sh
7. Calibrate the robot if it was not already calibrated:
    - Edit the test_calibration/parameters.yaml according to your network setup
    - Run the calibration node by calling ./start_TestCalibration
8. Run the impedance node:
    - Edit the test_impedance/parameters.yaml according to your network setup
    - Call ./start_TestImpedance
9. Or run the admittance node:
    - Edit the test_admittance/parameters.yaml according to your network setup
    - Call ./start_TestAdmittance
    
    
    

