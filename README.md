[![Humble CI](https://github.com/idra-lab/haptic_interface_ROS2/actions/workflows/main.yml/badge.svg)](https://github.com/idra-lab/haptic_interface_ROS2/actions/workflows/main.yml)  


*The following instructions are the one that were distribuited with the vendor's code with some clarifications. You must follow the instruction in the haptic documentation first (having the right files under /etc/Haption/). Documentation and libraries are [here](https://drive.google.com/drive/folders/1g4NHb75PtUcHunHAImuzkCfoDhdFXWoR?usp=drive_link). You can avoid points #0 and #4 by running the 'entrypoint.sh'*

This project includes a basic implementation of the test programs (TestCalibration, TestImpedance and TestAdmittance) distribuited with the haptic interface and the IDRA team's implementation for the teleoperation (under `haptic_control`) using ROS2 and RaptorAPI.
This guide is written for Linux, Windows users need to adapt it in the appropriate way.

## Requirements
- Ubuntu 22.04 (API binaries for MacOS and Windows are also present but it did not tested them).
- [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
- PCI access
   ```bash
   sudo apt-get install libpciaccess-dev
   ``` 
The next requirements is needed only if the virtual fixtures are used.
- OpenMP
  ```
  sudo apt install libomp-dev
  ```
<!-- - Open3D [from source](https://www.open3d.org/docs/release/compilation.html) -->

## Installation
0. Clone as a workspace named `haption_ws`:
    ```bash
    git clone https://github.com/idra-lab/haptic_interface_ROS2 haption_ws
    ```
1. Source ROS environment:
    ```bash
    source /opt/ros/$ROS-DISTRO/local_setup.bash
    ```
3. Copy the `desktop_6D_n65.param` in `/etc/Haption/Connector`. Create the missing folders if needed. The parameters file is stored in the IDRA drive folder [here](https://drive.google.com/drive/folders/1g4NHb75PtUcHunHAImuzkCfoDhdFXWoR?usp=sharing), request access to the drive if you need to use the haptic interface.  
4. Build the workspace
    ```bash
    colcon build --symlink-install
    ```
5. Enter **sudo mode** and run the entrypoint.sh script:
    ```bash
    sudo su
    source entrypoint.sh
    ```
6. Start the RaptorAPIWrapper node by calling 
    ```bash
    ros2 run haption_raptor_api raptor_api_wrapper 
    ```
## Ethernet configuration
The computer must be connected via ethernet to the haptic device black box. The network interface used must have the `192.168.100.50` IP address. In Ubuntu you can set this IP from `Settings -> Network -> Select the right network interface -> Click the gear -> IPv4` and set
- Address: `192.168.100.50`
- Netmask: `255.255.255.0`

## Run the examples
#### Calibration
Calibrate the robot if it was not already calibrated (in another sudo shell):
  - Edit the `test_calibration/parameters.yaml` according to your network setup
  - Check that the green `calibration/force feedback` button is not pressed otherwise press it again.
  - Run the calibration node by calling 
    ```bash
    ros2 run test_calibration test_calibration --ros-args --params-file ./src/test_calibration/config/parameters.yaml
    ```
  - When you read `C_WAITINGFORPOWER: P_NOPOWER` in the RaptorAPIWrapper terminal, press the green `calibration/force feedback` button on the haptic device to power it on.
    You will see the haptic device moving to the calibration position.
#### Impedance control
. Run the impedance node (in another sudo shell):
- Edit the `test_impedance/parameters.yaml` according to your network setup
- Run 
    ```bash
    ros2 run test_impedance test_impedance --ros-args --params-file ./src/test_impedance/config/parameters.yaml
    ```
#### Admittance control
Or run the admittance node (in another sudo shell):
- Edit the `test_admittance/parameters.yaml` according to your network setup
- Run 
    ```bash
    ros2 run test_impedance test_impedance --ros-args --params-file ./src/test_impedance/config/parameters.yaml
    ```



## Robot teleoperation sample
The haptic interface can be used to command a target pose and the resulting force measured by the robot can be
exerted by the haptic interface through the `haptic_control` package that I implemented. Check `haptic_control/parameters.yaml` for the list of parameters.
```bash
ros2 launch haptic_control sample_teleoperation.launch.py 
```
## Mesh Virtual Fixtures control
An implementation of the algorithm presented [here](https://ieeexplore.ieee.org/document/9341590/) is implemented.
### Requirements
- Open3D C++
### Run the teleoperation with mesh virtual fixtures
```
ros2 launch haptic_control haptic_control.launch.py use_fixtures:=true
``` 
### Delay simulation
A delay simulation is implemented and can be tested with
```
ros2 launch haptic_control haptic_control.launch.py use_fixtures:=true delay:=0.4
```
# Trouble shooting
- If you get this error `HAPTION::RaptorAPI::StartLogging() failed with error 13` you probably did not sourced `entrypoint.sh`
- If you get `HAPTION::RaptorAPI::StartLogging() failed with error 1` the haptic interface is probably shutted down
    
    
    

