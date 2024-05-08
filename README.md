*The following instructions are the one that were distribuited with the vendor's code with minor clarifications. You must follow the instruction in the haptic documentation first (having the right files under /etc/Haption/). Documentation and libraries are [here](https://drive.google.com/drive/folders/1g4NHb75PtUcHunHAImuzkCfoDhdFXWoR?usp=drive_link). I uploaded the binaries in the correct folder so ***you do not need to follow point #1***. Also you can avoid points #0 and #4 by running the 'entrypoint.sh'*

This project includes a basic implementation of the test programs (TestCalibration, TestImpedance and TestAdmittance) using ROS2 and RaptorAPI.
This guide is written for Linux, Windows users need to adapt it in the appropriate way.

## Installation and setup
1. Installation of ros2: 
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
2. Clone as a workspace named `haption_ws`:
    ```bash
    git clone https://github.com/idra-lab/haptic_interface_ROS2 haption_ws
    ```
3. RaptorAPI headers are already included under src/haption_raptor_api/Dependencies/RaptorAPI
## Usage
0. Source ROS environment and copy shell scripts to the workspace root:
    ```bash
    source /opt/ros/$ROS-DISTRO/local_setup.bash
    ```
    ```bash
    cp  <path_to_ws>/src/haptic_interface_ROS2/*.sh  <path_to_ws>
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
6. Enter **sudo mode** and run the entrypoint.sh script:
    ```bash
    sudo su
    source entrypoint.sh
    ```
7. Start the RaptorAPIWrapper node by calling 
    ```bash
    ./start_RaptorAPIWrapper.sh
    ```
8. Calibrate the robot if it was not already calibrated (in another sudo shell):
    - Edit the test_calibration/parameters.yaml according to your network setup
    - Check that the green `calibration/force feedback` button is not pressed otherwise press it again.
    - Run the calibration node by calling 
        ```bash
        ./start_TestCalibration.sh
        ```
    - When you read `C_WAITINGFORPOWER: P_NOPOWER` in the RaptorAPIWrapper terminal, press the green `calibration/force feedback` button on the haptic device to power it on.
    You will see the haptic device moving to the calibration position.
9. Run the impedance node (in another sudo shell):
    - Edit the test_impedance/parameters.yaml according to your network setup
    - Run 
        ```bash
        ./start_TestImpedance.sh
        ```
    Or run the admittance node (in another sudo shell):
    - Edit the test_admittance/parameters.yaml according to your network setup
    - Run 
        ```bash
        ./start_TestAdmittance.sh
        ```
## Ethernet configuration
The computer must be connected via ethernet to the haptic device black box. The network interface used must have the `192.168.100.50` IP address. In Ubuntu you can set this IP from `Settings -> Network -> Select the right network interface -> Click the gear -> IPv4` and set
- Address: `192.168.100.50`
- Netmask: `255.255.255.0`
## Robot teleoperation
The haptic interface can be used to command a target pose and the resulting force measured by the robot can be
exerted by the haptic interface through the `haptic_control` node. Also a safe XYZ cartesian position zone limit is implemented. Limits can be changes modifyng the parameters in `haptic_control/parameters.yaml`.
Finally run
```bash
./haptic_control.sh
```


    
    
    

