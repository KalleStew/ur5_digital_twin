# Relevant External Documentation

This document serves as a centralized index for the official documentation, tutorials, and API references relevant to the tools powering this digital twin. When modifying the system or writing new nodes, refer to these specific versions (Humble) to avoid compatibility issues.

---

## 1. ROS 2 Core (Humble Hawksbill)

* **[ROS 2 Humble Official Documentation](https://docs.ros.org/en/humble/index.html)**
  * *Why it's relevant:* The master guide for ROS 2 concepts. Useful for understanding the underlying architecture of topics, services, and actions used in our python control scripts.
* **[rclcpp API Reference (C++)](https://docs.ros2.org/latest/api/rclcpp/)**
  * *Why it's relevant:* Crucial when modifying the C++ trajectory planners (`multi_waypoint_planner.cpp`). Contains the syntax for node creation, parameter injection, and publishing.
* **[rclpy API Reference (Python)](https://docs.ros2.org/latest/api/rclpy/)**
  * *Why it's relevant:* The backbone of the AI sandbox. Use this when expanding `ai_torque_controller.py` or modifying the 100Hz telemetry subscribers.

## 2. MoveIt 2 & Kinematics

* **[MoveIt 2 Humble Tutorials](https://moveit.picknik.ai/humble/index.html)**
  * *Why it's relevant:* Explains how to modify the `ur5_moveit_config` package, tune PID gains, and adjust the SRDF (Semantic Robot Description Format).
* **[MoveGroupInterface C++ API](https://moveit.picknik.ai/main/api/html/classmoveit_1_1planning__interface_1_1MoveGroupInterface.html)**
  * *Why it's relevant:* The specific C++ class utilized by our conventional planner to calculate Inverse Kinematics, avoid collision tables, and execute safe Cartesian paths.

## 3. Simulation & Physics (Gazebo)

* **[Gazebo Documentation (Fortress/Harmonic)](https://gazebosim.org/docs/all/getstarted)**
  * *Why it's relevant:* The physics engine running the digital twin. Useful for understanding mass/inertia tags, gearbox friction, and collision geometry rendering.
* **[ros_gz Bridge Documentation](https://github.com/gazebosim/ros_gz)**
  * *Why it's relevant:* Explains how ROS 2 topics map to Gazebo's internal topics. Essential if you want to add custom sensors (like cameras or force/torque sensors) to the simulation later.

## 4. Control Systems & Hardware-in-the-Loop

* **[ros2_control Framework](https://control.ros.org/humble/index.html)**
  * *Why it's relevant:* The architecture that makes our "Control Handoff" possible. Documents how the `controller_manager` works and how to write custom hardware interfaces for the Platinum Maestro.
* **[ros2_controllers Repository](https://github.com/ros-controls/ros2_controllers)**
  * *Why it's relevant:* Contains the mathematical implementations of the `joint_trajectory_controller` (our C++ position lock) and the `forward_command_controller` (our raw AI torque injector).
* **[Ignition / Gazebo ROS 2 Control Plugin](https://github.com/ros-controls/gz_ros2_control)**
  * *Why it's relevant:* The specific bridge that allows the `controller_manager` to send virtual voltage to Gazebo's simulated motors. 

## 5. Middleware & Networking

* **[Eclipse Cyclone DDS](https://cyclonedds.io/docs/cyclonedds/latest/)**
  * *Why it's relevant:* The recommended ROS Middleware (RMW) for this project. If you experience dropped packets or lag while pushing 100Hz torque arrays during the AI handoff, refer here for network tuning.