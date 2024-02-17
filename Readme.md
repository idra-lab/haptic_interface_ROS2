*The following instructions are the one that were distribuited with the vendor's code with minor clarifications. You must follow the instruction in the haptic documentation first (having the right files under /etc/Haption/). Documentation and libraries are [here](https://drive.google.com/drive/folders/1g4NHb75PtUcHunHAImuzkCfoDhdFXWoR?usp=drive_link). I uploaded the binaries in the correct folder so ***you do not need to follow point #1***. Also you can avoid points #0 and #4 by running the 'entrypoint.sh'*

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
0. Source ROS environment
    ```bash
    source /opt/ros/$ROS-DISTRO/local_setup.bash
    ```
1. Copy the RaptorAPI shared libraries to "src/haption_raptor_api/Dependencies/RaptorAPI/bin/Linux/glibc-<version>"
2. Make sure that the ".param" file for your device is accessible under /etc/Haption/Connector
3. Prepare the raptor_api_interfaces:
    ```bash
    colcon build --packages-select raptor_api_interfaces
    ```
4. Update LD_LIBRARY_PATH so that it includes the path to `src/haption_raptor_api/Dependencies/RaptorAPI/bin/Linux/glibc-<version>`  
    ```
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:<path_to_ws>/src/haption_raptor_api/Dependencies/RaptorAPI/bin/Linux/glibc-<version>
    ```
5. Source the workspace:
    ```bash
    source <path_to_ws>/install/local_setup.bash
    ```
6. Start the RaptorAPIWrapper node by calling 
    ```bash
    ./start_RaptorAPIWrapper.sh
    ```
7. Calibrate the robot if it was not already calibrated:
    - Edit the test_calibration/parameters.yaml according to your network setup
    - Run the calibration node by calling 
        ```bash
        ./start_TestCalibration
        ```
8. Run the impedance node:
    - Edit the test_impedance/parameters.yaml according to your network setup
    - Run 
        ```bash
        ./start_TestImpedance
        ```
9. Or run the admittance node:
    - Edit the test_admittance/parameters.yaml according to your network setup
    - Run 
        ```bash
        ./start_TestAdmittance
        ```
    
    
    

