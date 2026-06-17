# UR5 7-DOF Digital Twin & HIL Research Platform

A comprehensive digital twin and Hardware-in-the-Loop (HIL) simulation platform for a custom 7-DOF UR5 robotic arm. Built on **ROS 2 Humble** and **Ubuntu 22.04**, this system is tailored for advanced control systems, space application research, and dynamic trajectory validation.

## System Overview

This repository provides a highly accurate, physics-enabled simulation of a UR5 arm, complete with accurate mass/inertia tags, gearbox friction dynamics, and custom collision boundaries. It acts as a dual-purpose environment:
1. **Kinematic Trajectory Planning:** Safely navigate complex Cartesian paths and avoid rigid obstacles using MoveIt 2.
2. **Dynamic Torque Control & AI Sandbox:** Instantly bypass position controllers to inject raw electrical torque arrays directly into the joint motors, allowing for real-time fault detection and AI model testing.

### Key Features
* **Split-Level Architecture:** Seamlessly hand off control between a conventional C++ kinematic brain (MoveIt 2) and a high-level Python AI torque-control script.
* **On-the-Fly Controller Switching:** Dynamically toggle between `ur_manipulator_controller` (position) and `forward_effort_controller` (torque) via the ROS 2 Controller Manager without dropping the simulation.
* **Real-Time Telemetry Logging:** Intercept and record position, velocity, and effort data at 100Hz into synchronized CSV exports for post-run analysis.
* **Force Injection:** Mathematical simulation of external disturbances (e.g., wind gusts, physical bumps) via custom wrench topics.

## Repository Structure

The workspace is organized into three primary ROS 2 colcon packages:

* `ur5_description`: The physical blueprint. Contains the URDF, STL meshes, collision tables, and Gazebo plugins.
* `ur5_moveit_config`: The kinematic configuration. Contains the SRDF, joint limits, and controller configurations generated for MoveIt 2.
* `ur5_controller`: The system brain. Contains the split-level C++ path planners and Python AI dynamic torque controllers.

## Documentation Overview

Comprehensive documentation for setting up, operating, and extending this system can be found in the `docs/` directory:

1. [System Architecture](docs/architecture.md) - Deep dive into node communication, data flow, and ROS 2 topic routing.
2. [Environment Setup](docs/environment_setup.md) - Instructions for configuring Ubuntu, WSL2, or VM environments. *(Coming Soon)*
3. [Operation & Deployment](docs/operation_and_deployment.md) - Commands for launching the sim, executing trajectories, and switching controllers. *(Coming Soon)*
4. [Development Workflow](docs/development_workflow.md) - Guidelines for branching, testing, and modifying the codebase. *(Coming Soon)*

---
**Maintainer Note:** This repository is currently targeting ROS 2 Humble. Future migrations to ROS 2 Lyrical Luthien will be documented in a separate migration guide.