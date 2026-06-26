# UR5 7-DOF Digital Twin & HIL Research Platform

A comprehensive digital twin and Hardware-in-the-Loop (HIL) simulation platform for a custom 7-DOF UR5 robotic arm. Built on **ROS 2 Lyrical Luthien (LL)** and **Ubuntu 26.04**, this system is tailored for advanced control systems, space application research, and dynamic trajectory validation.

> **Migration note:** This branch was migrated from ROS 2 Humble / Ubuntu 22.04. See [docs/migration_UB22_to_UB26.md](docs/migration_UB22_to_UB26.md) for the complete technical changelog.

## System Overview

This repository provides a highly accurate, physics-enabled simulation of a UR5 arm, complete with accurate mass/inertia tags, gearbox friction dynamics, and custom collision boundaries (1.525 m × 0.76 m × 0.99 m table). It acts as a dual-purpose environment:
1. **Kinematic Trajectory Planning:** Safely navigate complex Cartesian paths and avoid rigid obstacles using MoveIt 2.
2. **Dynamic Torque Control & AI Sandbox:** Instantly bypass position controllers to inject raw electrical torque arrays directly into the joint motors, allowing for real-time fault detection and AI model testing.

### Key Features
* **Dual-Brain Architecture:** Seamlessly hand off control between a conventional C++ kinematic brain (MoveIt 2) and a high-level Python torque-control script. Both run simultaneously; authority is swapped via a single service call.
* **Eclipse Zenoh Middleware:** FastDDS replaced with `rmw_zenoh_cpp` as the Tier 1 RMW. Eliminates DDS participant-discovery overhead on high-frequency torque array channels (100 Hz effort streams).
* **On-the-Fly Controller Switching:** Dynamically toggle between `ur_manipulator_controller` (position/effort) and `forward_effort_controller` (raw torque injection) via `/controller_manager/switch_controller` without dropping the simulation.
* **Real-Time HDF5 Telemetry Logging:** Intercept and record position, velocity, and effort data at 100 Hz into synchronized `.h5` files and CSV exports for post-run analysis.
* **Deadlock-Free Threading:** Python HIL scripts run control logic in a background `threading.Thread` while `rclpy.spin()` holds the main thread — preventing `SingleThreadedExecutor` deadlocks on the `FollowJointTrajectory` action server.
* **Safe Start Pose (Candle Pose):** `shoulder_lift_joint = -1.5708 rad` baked into the SRDF as a named group state, ensuring the arm spawns pointing straight up and clear of the table surface on every boot.

## Repository Structure

The workspace is organized into three primary ROS 2 colcon packages:

* `ur5_description`: The physical blueprint. Contains the URDF, STL meshes, the `gz_ros2_control/GazeboSystem` plugin, accurate physics parameters, and the collision table.
* `ur5_moveit_config`: The kinematic configuration. Contains the SRDF, joint limits, kinematics solver config, controller gains (v3 PID), and the master `gazebo_sim.launch.py` built around `MoveItConfigsBuilder`.
* `ur5_controller`: The system brain. Contains the C++ Cartesian waypoint planner (`multi_waypoint_planner`) and Python HIL scripts with HDF5 telemetry export.

## Quick Start

```bash
# 1. Source the environment
source /opt/ros/ll/setup.bash
source ~/ros2_ws/install/setup.bash

# 2. Build
cd ~/ros2_ws
colcon build --symlink-install

# 3. Launch the full simulation stack
ros2 launch ur5_moveit_config gazebo_sim.launch.py

# 4. (Optional) Launch the C++ waypoint planner
ros2 launch ur5_controller planner.launch.py
```

## Documentation

| Document | Description |
|---|---|
| [docs/architecture.md](docs/architecture.md) | Node graph, topic routing, dual-brain data flow |
| [docs/environment_setup.md](docs/environment_setup.md) | Ubuntu 26.04 setup for WSL2 (Windows) and ARM64 VM (Apple Silicon) |
| [docs/development_workflow.md](docs/development_workflow.md) | Threaded Action Client pattern, controller swap procedure, Git workflow |
| [docs/operation_and_deployment.md](docs/operation_and_deployment.md) | Launch commands, trajectory execution, telemetry export |
| [docs/troubleshooting_and_reset.md](docs/troubleshooting_and_reset.md) | Zombie process kill sequence, cache nuke, DDS daemon reset |
| [docs/migration_UB22_to_UB26.md](docs/migration_UB22_to_UB26.md) | Technical changelog: Humble → Lyrical Luthien migration |
