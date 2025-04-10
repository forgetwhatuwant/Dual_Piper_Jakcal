# Dual Piper Robot Setup (ROS 2 Humble)

![ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange.svg)
![humble](https://img.shields.io/badge/ros-humble-blue.svg)

This document provides instructions specifically for setting up and launching the dual Piper arm configuration using `ros2_control` and MoveIt 2.

## 1. Prerequisites

### 1.1 ROS 2 Humble & Dependencies

Ensure you have ROS 2 Humble installed. Install necessary dependencies:

```bash
# General ROS 2 Control & MoveIt
sudo apt update
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-controller-manager ros-humble-moveit ros-humble-joint-state-publisher-gui ros-humble-robot-state-publisher ros-humble-xacro

# Python dependencies (if using real hardware with CAN)
pip3 install python-can>=4.3.1 scipy piper_sdk
```

### 1.2 Workspace Setup

Clone this repository into your ROS 2 workspace (e.g., `~/ros2_ws/src`) and build it:

```bash
cd ~/ros2_ws/src
# git clone <your-repo-url> . # Uncomment and replace if needed
cd ..
colcon build --symlink-install
source install/setup.bash
```

## 2. Hardware Setup (Real Robots Only)

If you are using real dual Piper arms connected via CAN:

### 2.1 CAN Interface Naming & Activation

You need two separate CAN interfaces, one for each arm. It's recommended to assign persistent names to avoid issues if USB port assignments change.

1.  **Identify Hardware Addresses:** Plug in the CAN adapter for the *left* arm *only*. Find its `bus-info`:
    ```bash
    sudo ethtool -i can0 | grep bus-info
    ```
    Record the address (e.g., `usb-0000:00:14.0-1:1.0`). Unplug it.
2.  **Identify Hardware Addresses:** Plug in the CAN adapter for the *right* arm *only* (use a different physical USB port). Find its `bus-info`:
    ```bash
    # It might appear as can0 again or can1
    sudo ethtool -i can0 | grep bus-info # or ethtool -i can1 ...
    ```
    Record the address (e.g., `usb-0000:00:14.0-2:1.0`).
3.  **Configure udev Rule (Recommended):** Create a udev rule to assign persistent names. Create a file `/etc/udev/rules.d/99-dual-can.rules` with content like this (replace addresses and desired names):
    ```udev
    # Rule for Left Arm CAN adapter
    SUBSYSTEM=="net", ACTION=="add", DRIVERS=="?*", ATTR{address}=="<MAC_ADDRESS_LEFT>", ATTR{dev_id}=="0x0", ATTRS{bus_id}=="<BUS_INFO_LEFT>", KERNEL=="can*", NAME="can_left"

    # Rule for Right Arm CAN adapter
    SUBSYSTEM=="net", ACTION=="add", DRIVERS=="?*", ATTR{address}=="<MAC_ADDRESS_RIGHT>", ATTR{dev_id}=="0x0", ATTRS{bus_id}=="<BUS_INFO_RIGHT>", KERNEL=="can*", NAME="can_right"
    ```
    *   Replace `<BUS_INFO_LEFT>` and `<BUS_INFO_RIGHT>` with the addresses you recorded (e.g., `1-1:1.0`).
    *   You might need to find the MAC addresses (`ATTR{address}`) using `ip link show can0` when each adapter is plugged in individually.
    *   Reload rules: `sudo udevadm control --reload-rules && sudo udevadm trigger`
4.  **Activate Interfaces:** Bring up the interfaces with the correct bitrate (typically 1000000 for Piper):
    ```bash
    sudo ip link set can_left type can bitrate 1000000
    sudo ip link set can_left up

    sudo ip link set can_right type can bitrate 1000000
    sudo ip link set can_right up
    ```
5.  **Verify:** Check `ifconfig` or `ip a` to see `can_left` and `can_right`.

**(Alternative: Manual Activation)** You can use the `can_activate.sh` or `can_config.sh` scripts from the main README, but ensure you modify them to use distinct names (e.g., `can_left`, `can_right`) and target the correct hardware addresses.

## 3. Launching the Dual Arm Setup

### 3.1 Simulation with RViz and MoveIt

This launch file starts the `robot_state_publisher`, `controller_manager` (with mock hardware using `mock_components`), MoveIt (`move_group`), and RViz configured for planning.

```bash
# Use the MoveIt config from dual_piper_moviet_3
ros2 launch dual_piper_moviet_3 demo.launch.py
```

*   **RViz:** You should see the dual-arm robot model.
*   **MoveIt:** The MotionPlanning panel should be available to plan for `left_arm`, `right_arm`, `left_gripper`, and `right_gripper`.
*   **Controllers:** The controllers (`left_arm_controller`, `right_arm_controller`, etc.) are loaded and managed by `ros2_control` using simulated hardware interfaces.

### 3.2 Real Hardware

Launching with real hardware requires:
1.  **Correct CAN Setup:** Ensure Section 2 is completed.
2.  **Hardware Interface Plugin:** The `dual_piper.ros2_control.xacro` needs to be modified to use your actual hardware interface plugin instead of `mock_components/GenericSystem`.
3.  **Control Node:** You likely need a dedicated node (like `piper_single_ctrl` adapted for dual arms) that communicates over CAN using `piper_sdk` and implements the `ros2_control` `SystemInterface`.
4.  **Modified Launch:** Your launch file will need to start the hardware interface node and potentially configure the `ros2_control` controllers differently (e.g., specifying the actual hardware CAN ports).

*(Detailed instructions for real hardware depend on your specific `ros2_control` hardware interface implementation.)*

## 4. Configuration Files Overview

Key configuration files for the `dual_piper_moviet_3` package:

*   `config/dual_piper.urdf.xacro`: Main robot description file, includes other Xacros.
*   `config/dual_piper.ros2_control.xacro`: Defines joints and interfaces for `ros2_control`. Specifies the hardware plugin (`mock_components` for simulation).
*   `config/ros2_controllers.yaml`: Configures the controllers (types, joints, interfaces) loaded by `controller_manager`.
*   `config/moveit_controllers.yaml`: Maps MoveIt planning groups to `ros2_control` controllers and specifies action namespaces (e.g., `follow_joint_trajectory`).
*   `config/kinematics.yaml`: Defines the kinematics solver for MoveIt.
*   `config/dual_piper.srdf`: Semantic Robot Description File for MoveIt (planning groups, end-effectors, collision pairs).
*   `launch/demo.launch.py`: Main launch file using `moveit_configs_utils` to start simulation/MoveIt/RViz.
*   `launch/rsp.launch.py`: Loads the robot description and starts `robot_state_publisher`.

## 5. Troubleshooting

*   **Controller Spawner Failure (`[FATAL] [spawner_...]`):**
    *   Check `ros2_controllers.yaml` for correct controller names, types, joints, and interface names.
    *   Verify joint names and interfaces match exactly between `ros2_controllers.yaml` and `dual_piper.ros2_control.xacro`.
    *   Ensure the `robot_description` parameter loaded by `robot_state_publisher` correctly includes the `dual_piper.ros2_control.xacro` definitions.
    *   Check `initial_positions.yaml` for missing joint entries if used by the hardware interface.
*   **MoveIt Errors (`No action namespace specified`, `Controller ... does not exist`):**
    *   Ensure controllers are successfully loaded by `controller_manager` (check terminal output).
    *   Verify `config/moveit_controllers.yaml` correctly maps MoveIt controller names to `ros2_control` controller names.
    *   Make sure `action_ns` (e.g., `follow_joint_trajectory`) is specified correctly in `moveit_controllers.yaml` for trajectory controllers.
*   **Real Hardware CAN Errors (`Message NOT sent`):**
    *   Double-check CAN wiring and termination.
    *   Verify CAN interfaces (`can_left`, `can_right`) are active (`ip a`) and configured with the correct bitrate.
    *   Ensure your hardware control node is correctly configured with the CAN interface names.

See the main [README.md](README.md) for more general troubleshooting tips.