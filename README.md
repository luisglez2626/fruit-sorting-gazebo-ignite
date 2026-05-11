# Collaborative Arm - Fruit Sorting Simulation

A comprehensive robotics simulation project built with **ROS 2** and **Gazebo Sim**. This project features a **UR5e** collaborative robot arm equipped with a custom parallel gripper and a stereo camera setup. It demonstrates manual Cartesian/Joint control, custom Inverse Kinematics (IK), computer vision-based object detection, and a fully automated pick-and-place sorting sequence for red and green apples.

## 🛠️ System Requirements

This project is tested and guaranteed to work on the following stack:

* **OS:** Ubuntu 22.04 LTS (Jammy Jellyfish)
* **ROS 2 Version:** ROS 2 Humble Hawksbill
* **Gazebo Version:** Gazebo Sim 6.16.0 (Ignition Fortress)
* **Language:** Python 3.10 & C++17

---

## 📦 Project Architecture

The workspace is divided into six specialized ROS 2 packages to ensure modularity and maintainability:

### 1. `collaborative_arm_bringup`
The master launch package responsible for bringing up the entire robotic system.
* **`launch/system.launch.py`**: The core executable script. It concurrently launches the Gazebo simulation, spawns the robot, loads controllers, and sequentially boots up the Inverse Kinematics, Vision, and GUI nodes with appropriate delays to ensure a stable startup.

### 2. `collaborative_arm_application`
A Python-based package containing the Graphical User Interface (GUI) and high-level logic.
* **`cartesian_jogger.py`**: A robust Tkinter UI node. It handles manual joint jogging, Cartesian movements (calculating required poses), reads Ground Truth from Gazebo via TF, and executes the full automated sorting choreography.
* **`config.py`**: A centralized configuration dictionary allowing easy modification of UI layouts, colors, physical limits, and target coordinates without altering the main logic.

### 3. `collaborative_arm_description`
Contains the physical definition, visual assets, and physical limits of the robot.
* **`urdf/collaborative_arm.urdf.xacro`**: The primary robot blueprint. It imports the base UR5e macro and attaches the custom pedestal, parallel gripper, stereo camera (`camera_left_link`, `camera_right_link`), and the `ign_ros2_control` hardware interfaces.

### 4. `collaborative_arm_gazebo`
Houses the simulation environment and Gazebo-specific configurations.
* **`worlds/fruit_sorting.world`**: The Gazebo SDF world file containing the lighting, ground plane, sorting table, and the red/green apples.
* **`config/arm_controllers.yaml`**: Configuration for `ros2_control`. Defines the `joint_trajectory_controller` instances for the 6-DOF arm and the parallel gripper.
* **`launch/start_gazebo.launch.py`**: Handles starting the Gazebo server, spawning the robot entity, and setting up the `ros_gz_bridge` to pass camera and TF data between Gazebo and ROS 2.

### 5. `collaborative_arm_kinematics`
A C++ package dedicated to mathematical pose calculations.
* **`src/ik_solver_node.cpp`**: An Inverse Kinematics node built with the **Orocos Kinematics and Dynamics Library (KDL)**. It parses the URDF dynamically, tracks real-time joint states, and converts incoming Cartesian target requests (X, Y, Z, R, P, Y) into a 6-joint trajectory array. It includes fallback seed logic to escape local minima limits.

### 6. `collaborative_arm_vision`
A Python package for processing simulated stereo camera feeds.
* **`vision_detector.py`**: Uses **OpenCV** and `cv_bridge` to subscribe to the left and right camera feeds. It applies HSV masking to identify red and green apples, calculates their stereo depth using pixel disparity, and broadcasts their calculated 3D coordinates to the ROS 2 TF tree.

---

## 🚀 Installation & Usage

### 1. Build the workspace
Navigate to the root of your ROS 2 workspace (e.g., `~/ros2_ws`) and compile the packages:

```bash
colcon build --symlink-install
```

### 2. Source the workspace
source install/setup.bash

### 3. Launch the System
Start the entire simulation, controllers, and UI using the `bringup` package:

```bash
ros2 launch collaborative_arm_bringup system.launch.py
```

---

## 🎮 Features
