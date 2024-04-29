*The following instructions are the one that were distribuited with the vendor's code with minor clarifications. You must follow the instruction in the haptic documentation first (having the right files under /etc/Haption/). Documentation and libraries are [here](https://drive.google.com/drive/folders/1g4NHb75PtUcHunHAImuzkCfoDhdFXWoR?usp=drive_link). I uploaded the binaries in the correct folder so ***you do not need to follow point #1***. Also you can avoid points #0 and #4 by running the 'entrypoint.sh'*

This project includes a basic implementation of the test programs (TestCalibration, TestImpedance and TestAdmittance) using ROS2 and RaptorAPI.
This guide is written for Linux, Windows users need to adapt it in the appropriate way.

## Installation and setup
1. Installation of ros2: 
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
2. Clone as a workspace:
If you want to follow the ros2 way for creating a workspace, you can clone this repository under ~/ros2_ws/, or visit the documentation page for creating a worspace https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html
3. RaptorAPI headers are already included under src/haption_raptor_api/Dependencies/RaptorAPI
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
    and build the workspace
    ```bash
    colcon build --symlink-install
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
        ./start_TestCalibration.sh
        ```
8. Run the impedance node:
    - Edit the test_impedance/parameters.yaml according to your network setup
    - Run 
        ```bash
        ./start_TestImpedance.sh
        ```
9. Or run the admittance node:
    - Edit the test_admittance/parameters.yaml according to your network setup
    - Run 
        ```bash
        ./start_TestAdmittance.sh
        ```
## Ethernet configuration
The computer must be connected via ethernet to the haptic device black box. The network interface used must have the `192.168.100.50` IP address. In Ubuntu you can set this IP from `Settings -> Network -> Select the right network interface -> Click the gear -> IPv4` and set
- Address: 192.168.100.50
- Netmask: 255.255.255.0
## Robot teleoperation
The haptic interface can be used to command a target pose and the resulting force measured by the robot can be
exerted by the haptic interface through the `haptic_control` node. Also a safe XYZ cartesian position zone limit is implemented. Limits can be changes modifyng the parameters in `haptic_control/parameters.yaml`.
Finally run
```bash
./haptic_control.sh
```


    
    
    

