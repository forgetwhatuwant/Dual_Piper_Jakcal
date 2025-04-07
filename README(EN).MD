# Dual Piper Robot (humble)

[CN](README.MD)

![ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange.svg)

python_version >= "3.10"

Test:

|PYTHON |STATE|
|---|---|
|![humble](https://img.shields.io/badge/ros-humble-blue.svg)|![Pass](https://img.shields.io/badge/Pass-blue.svg)|

## 1 Installation

### 1.1 Install Dependencies

Note: python-can version should be higher than 4.3.1

```shell
pip3 install python-can
pip3 install scipy
```

```shell
pip3 install piper_sdk
```

```shell
sudo apt install ros-$ROS_DISTRO-ros2-control
sudo apt install ros-$ROS_DISTRO-ros2-controllers
sudo apt install ros-$ROS_DISTRO-controller-manager
```

## 2 Quick Start

### 2.1 Enable CAN Module

#### 2.1.1 Activate Single CAN Module, **using `can_activate.sh` script**

##### 1) PC with only one USB to CAN module

Execute directly

```bash
bash can_activate.sh can0 1000000
```

##### 2) PC with multiple USB to CAN modules, but activate one at a time

Note: This is used when using both robot arm and chassis

(1) Check the hardware address of the CAN module in the USB port. Unplug all CAN modules, insert only the CAN module connected to the robot arm into the PC, and execute

```shell
sudo ethtool -i can0 | grep bus
```

Record the value of `bus-info`, for example `1-2:1.0`

Note: **Generally, the first inserted CAN module will be can0 by default. If you can't find the can, you can use `bash find_all_can_port.sh` to check the can name corresponding to the USB address**

(2) Activate the CAN device. Assuming the `bus-info` value recorded above is `1-2:1.0`, execute:

```bash
bash can_activate.sh can_piper 1000000 "1-2:1.0"
```

Note: **The CAN device inserted in the USB port with hardware code 1-2:1.0 is renamed to can_piper, baud rate is 1000000, and activated**

(3) Check if activation is successful

Execute `ifconfig` to see if there is `can_piper`, if yes, the CAN module is set successfully

#### 2.1.2 Activate Multiple CAN Modules Simultaneously, **using `can_config.sh` script**

##### 1) Plug and unplug CAN modules one by one and record the USB port hardware address for each module

In `can_config.sh`, the `EXPECTED_CAN_COUNT` parameter is the number of CAN modules you want to activate, now assume it's 2

Then insert one of the CAN modules into the PC alone, execute

```shell
sudo ethtool -i can0 | grep bus
```

Record the value of `bus-info`, for example `1-2:1.0`

(2) Then insert the next CAN module, note that it **cannot** be the same USB port as the last CAN module, then execute

```shell
sudo ethtool -i can1 | grep bus
```

Note: **Generally, the first inserted CAN module will be can0, the second will be can1. If you can't find the can, you can use `bash find_all_can_port.sh` to check the can name corresponding to the USB address**

##### 2) Predefine USB Ports, Target Interface Names and Their Bit Rates

Assuming the `bus-info` values recorded above are `1-2:1.0` and `1-4:1.0`, then replace the parameters inside the double quotes in `USB_PORTS["1-9:1.0"]="can_left:1000000"` with `1-2:1.0` and `1-4:1.0`.

Final result:

`USB_PORTS["1-2:1.0"]="can_right:1000000"`

`USB_PORTS["1-4:1.0"]="can_right:1000000"`

Note: **The CAN device inserted in the USB port with hardware code 1-2:1.0 is renamed to can_left, baud rate is 1000000, and activated**

##### 3) Check if activation is successful

Execute `bash can_config.sh`

##### 4) Check if CAN is set successfully

Execute `ifconfig` to see if there are `can_left` and `can_right`

### 2.2 Run Nodes

Build

```shell
colcon build
```

#### 2.2.1 Single Robot Arm

Node name `piper_single_ctrl`

Parameters

```shell
can_port: CAN route name to open
auto_enable: Whether to enable automatically, True means enable automatically when starting
# Note that if this is set to False, after interrupting the program and restarting the node, the robot arm will maintain the state from the last program start
# If the robot arm was enabled in the last program start, after interrupting the program and restarting the node, the robot arm will still be enabled
# If the robot arm was disabled in the last program start, after interrupting the program and restarting the node, the robot arm will still be disabled
gripper_exist: Whether there is an end gripper, True means there is an end gripper, will enable gripper control
rviz_ctrl_flag: Whether to use rviz to send joint angle messages, True means receive joint angle messages from rviz
gripper_val_mutiple: Set gripper control multiple
# Since the range of joint7 in rviz is [0,0.04], and the actual gripper stroke is 0.08m, opening rviz to control the actual gripper requires setting the gripper to double
```

There are two launch files for starting a single arm: `start_single_piper_rviz.launch.py` and `start_single_piper.launch.py`

The former can start rviz together, and then you can control the robot arm by dragging the slider

Start control node

```shell
# Start node
ros2 run piper piper_single_ctrl --ros-args -p can_port:=can0 -p auto_enable:=false -p gripper_exist:=true -p gripper_val_mutiple:=2
# Start launch
ros2 launch piper start_single_piper.launch.py can_port:=can0 auto_enable:=false gripper_exist:=false gripper_val_mutiple:=2
# Or, will run with default parameters
ros2 launch piper start_single_piper.launch.py
# You can also use rviz to start control, parameters to change as above
ros2 launch piper start_single_piper_rviz.launch.py
```

`ros2 topic list`

```shell
/arm_status #Robot arm status, see below for details
/enable_flag #Enable flag, send to node, send true to enable
/end_pose #Robot arm end pose status feedback
/joint_states #Subscribe to joint messages, sending joint positions to this message can control robot arm movement
/joint_states_single #Robot arm joint status feedback
/pos_cmd #End control message
```

ros2 service list

```shell
/enable_srv #Robot arm enable server
```

Enable robot arm

```shell
# Call server
ros2 service call /enable_srv piper_msgs/srv/Enable enable_request:\ true\ 
# Pub topic
ros2 topic pub /enable_flag std_msgs/msg/Bool data:\ true\ 
```

Disable robot arm

```shell
# Call server
ros2 service call /enable_srv piper_msgs/srv/Enable enable_request:\ false\ 
# Pub topic
ros2 topic pub /enable_flag std_msgs/msg/Bool data:\ false\ 
```

Publish joint messages

Note, the robot arm will lift up, please ensure there are no obstacles in the robot arm's working range

The robot arm will move at 10% of the default speed, gripper torque is set to 0.5N

Speed range is limited to 1-100, less than 1 is treated as 1

Position as follows

```shell
ros2 topic pub /joint_states sensor_msgs/msg/JointState "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'piper_single'}, name: ['joint1', 'joint2','joint3','joint4','joint5','joint6','joint7'], position: [0.2,0.2,-0.2,0.3,-0.2,0.5,0.01], velocity: [0,0,0,0,0,0,10], effort: [0,0,0,0,0,0,0.5]}"
```

#### 2.2.2 Dual Robot Arm Control

The dual robot arm system consists of two independent Piper robot arms, which can be started in the following way:

```shell
# Start dual robot arm visualization
ros2 launch dual_piper_description display_dual_piper.launch.py
```

The URDF description file for the dual robot arms is located at:
```
src/dual_piper_description/urdf/dual_piper.urdf.xacro
```

You can view the robot arm model in the following way:
```shell
# View dual robot arm model in RViz
ros2 launch dual_piper_description display_dual_piper.launch.py
```

## 3 Notes

- You need to activate the CAN device first and set the correct baud rate before you can read robot arm messages or control the robot arm
- If you see

  ```shell
  Enable status: False
  <class 'can.exceptions.CanError'> Message NOT sent
  <class 'can.exceptions.CanError'> Message NOT sent
  ```

  It means the robot arm is not connected to the CAN module. Unplug the USB, restart the robot arm, activate the CAN module again, and try to restart the node

- If auto-enable is turned on, it will automatically exit the program if enabling is not successful after 5s

### 3.1 Piper Custom Messages

ROS package `piper_msgs`

Robot arm status feedback message, corresponding to the feedback message with `id=0x2A1` in the CAN protocol

`PiperStatusMsg.msg`

```c
uint8 ctrl_mode
/*
0x00 Standby mode  
0x01 CAN command control mode
0x02 Teaching mode
0x03 Ethernet control mode
0x04 WiFi control mode
0x05 Remote control mode
0x06 Linked teaching input mode
0x07 Offline trajectory mode*/
uint8 arm_status
/*
0x00 Normal
0x01 Emergency stop
0x02 No solution
0x03 Singular point
```

End-effector pose control: Note that some singularities may be unreachable.

`PosCmd.msg`

```c
float64 x
float64 y
float64 z
float64 roll
float64 pitch
float64 yaw
float64 gripper
int32 mode1
int32 mode2
```

## MoveIt Planner and Simulation Environment

| Description | Documentation |
|---|---|
| MoveIt | [MoveIt README](src/piper_moveit/README(EN).md) |
| Simulation | [Simulation README](src/piper_sim/README(EN).md) |

## Q&A

1.  

```shell
colcon build
Starting >>> piper   
Starting >>> piper_control
Starting >>> piper_description
Starting >>> piper_msgs
Finished <<< piper [0.31s]                                                                                                              
--- stderr: piper_description                                                                                            
CMake Error at CMakeLists.txt:5 (find_package):
  By not providing "Findament_cmake.cmake" in CMAKE_MODULE_PATH this project
  has asked CMake to find a package configuration file provided by
  "ament_cmake", but CMake did not find one.

  Could not find a package configuration file provided by "ament_cmake" with
  any of the following names:

    ament_cmakeConfig.cmake
    ament_cmake-config.cmake

  Add the installation prefix of "ament_cmake" to CMAKE_PREFIX_PATH or set
  "ament_cmake_DIR" to a directory containing one of the above files.  If
  "ament_cmake" provides a separate development package or SDK, be sure it
  has been installed.


---
Failed   <<< piper_description [0.45s, exited with code 1]
Aborted  <<< piper_control [0.45s]
Aborted  <<< piper_msgs [0.45s]

Summary: 1 package finished [0.55s]
  1 package failed: piper_description
  2 packages aborted: piper_control piper_msgs
  3 packages had stderr output: piper_control piper_description piper_msgs
```

```shell
source /opt/ros/foxy/setup.bash
```

2. 

```shell
Starting >>> piper   
Starting >>> piper_control
Starting >>> piper_description
Starting >>> piper_msgs
Finished <<< piper [0.33s]                                                                                                              
Finished <<< piper_description [0.42s]                                                                                     
--- stderr: piper_control                                                               
CMake Error at CMakeLists.txt:7 (find_package):
  By not providing "Findcontroller_manager.cmake" in CMAKE_MODULE_PATH this
  project has asked CMake to find a package configuration file provided by
  "controller_manager", but CMake did not find one.

  Could not find a package configuration file provided by
  "controller_manager" with any of the following names:

    controller_managerConfig.cmake
    controller_manager-config.cmake

  Add the installation prefix of "controller_manager" to CMAKE_PREFIX_PATH or
  set "controller_manager_DIR" to a directory containing one of the above
  files.  If "controller_manager" provides a separate development package or
  SDK, be sure it has been installed.


---
Failed   <<< piper_control [0.80s, exited with code 1]
Aborted  <<< piper_msgs [3.28s]                               

Summary: 2 packages finished [3.38s]
  1 package failed: piper_control
  1 package aborted: piper_msgs
  1 package had stderr output: piper_control
```

```shell
sudo apt install ros-$ROS_DISTRO-ros2-control
sudo apt install ros-$ROS_DISTRO-ros2-controllers
sudo apt install ros-$ROS_DISTRO-controller-manager
```

3. 

```shell
 "package 'joint_state_publisher_gui' not found
```

```shell
sudo apt install ros-$ROS_DISTRO-joint-state-publisher-gui
```

4. 

```shell
sudo apt install ros-$ROS_DISTRO-robot-state-publisher
```

5. 

```shell
sudo apt install ros-foxy-xacro
```
