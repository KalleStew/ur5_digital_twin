# UR5 7-DOF Digital Twin & HIL Research Platform

A comprehensive digital twin and Hardware-in-the-Loop (HIL) simulation platform for a custom 7-DOF UR5 robotic arm, supporting Master's research at the ICE Lab for an MDA Space project. Built on **ROS 2 Jazzy Jalisco** and **Ubuntu 24.04 LTS Noble Numbat**, with **Eclipse Zenoh** as the default middleware and **Gazebo Harmonic** as the physics engine.

> **Platform:** Natively targets Windows WSL2 (Ubuntu 24.04) and Apple Silicon (ARM64) virtual machines.

---

## System Overview

This repository provides a physics-enabled digital twin of a UR5 arm with accurate mass/inertia tags, gearbox friction dynamics, and a 1.525 m × 0.76 m × 0.99 m rigid workbench collision boundary. The system operates in two primary modes:

1. **Kinematic Trajectory Planning** — Navigate complex Cartesian paths around rigid obstacles using MoveIt 2 and the `ur_manipulator_controller` (effort-based PID position control).
2. **Dynamic Torque Injection & Research Sandbox** — Bypass position controllers entirely and push raw torque arrays at 100 Hz directly into the Gazebo physics engine via `forward_effort_controller`, enabling fault detection, disturbance simulation, and AI model validation.

---

## Two-Brain Architecture

The system implements a **Dual-Brain Hybrid Control** pattern. Physical safety constraints prevent two controllers from commanding the same motor simultaneously, so control authority is explicitly transferred via a ROS 2 service call:

```
┌──────────────────────────────────────────────────────────────────┐
│  BRAIN A — Kinematic Brain (C++)                                  │
│  Node:    multi_waypoint_planner                                  │
│  Path:    RViz click → MoveIt IK → JointTrajectory →             │
│           ur_manipulator_controller → Gazebo joints               │
└────────────────────────┬─────────────────────────────────────────┘
                         │  /controller_manager/switch_controller
                         │  (deactivate A, activate B)
┌────────────────────────▼─────────────────────────────────────────┐
│  BRAIN B — Research Brain (Python, threaded)                      │
│  Nodes:   conventional_research.py / ai_research.py              │
│  Path:    Python loop → Float64MultiArray →                       │
│           forward_effort_controller → raw joint torques           │
│  Logging: /joint_states @100 Hz → HDF5 (.h5) telemetry files     │
└──────────────────────────────────────────────────────────────────┘
```

The Python scripts use a **background-thread + main-thread spin** pattern to prevent `SingleThreadedExecutor` deadlocks: `rclpy.spin()` runs on the main thread; all blocking control logic (IK, service calls, HDF5 writes) runs in a `threading.Thread`.

---

## Key Features

| Feature | Detail |
|---|---|
| **Middleware** | Eclipse Zenoh (`rmw_zenoh_cpp`) — zero-broker, DDS-free, ideal for high-frequency torque arrays |
| **Physics Engine** | Gazebo Harmonic (`gz_ros2_control/GazeboSimSystem`) |
| **Position Control** | `ur_manipulator_controller` — effort-command JTC with tuned PID gains |
| **Torque Control** | `forward_effort_controller` — raw effort pass-through, inline joint array format |
| **Telemetry** | 100 Hz `/joint_states` logged to HDF5 (`.h5`) via `h5py` |
| **Startup Safety** | `candle_pose` baked into `ur5.srdf` — arm vertical at boot, clears table |
| **VM Tolerance** | All controller spawners use `--controller-manager-timeout 30` |

---

## Repository Structure

```
ur5_digital_twin/
├── ur5_description/         # Physical model: URDF, STL meshes, Gazebo plugin tags
│   ├── urdf/ur5.urdf        # URDF 1.2 with gz_ros2_control, physics gains, accel/jerk limits
│   ├── meshes/              # Visual and collision STL geometry
│   └── launch/sim.launch.py # Standalone Gazebo boot (no MoveIt)
│
├── ur5_moveit_config/       # MoveIt 2 configuration (Jazzy MoveItConfigsBuilder)
│   ├── config/
│   │   ├── ur5.srdf         # Semantic description + candle_pose group state
│   │   ├── ros2_controllers.yaml  # Controller definitions + forward_effort_controller
│   │   ├── kinematics.yaml  # KDL solver config
│   │   └── joint_limits.yaml
│   └── launch/
│       └── gazebo_sim.launch.py  # Single master launch file (MoveItConfigsBuilder)
│
├── ur5_controller/          # Control logic
│   ├── src/
│   │   ├── multi_waypoint_planner.cpp   # C++ Cartesian waypoint planner
│   │   ├── unified_control.py           # Threaded Python HIL controller
│   │   └── conventional_cartesian_ik.py # Conventional IK research script
│   └── launch/planner.launch.py
│
└── docs/
    ├── architecture.md
    ├── environment_setup.md       # Ubuntu 24.04 / WSL2 / ARM64 / Zenoh setup
    ├── development_workflow.md    # Threaded Python pattern, controller switching
    ├── troubleshooting_and_reset.md
    ├── operation_and_deployment.md
    └── migration_UB22_to_UB24.md  # Humble → Jazzy migration changelog
```

---

## Quick Start

```bash
# 1. Source ROS 2 Jazzy and Zenoh middleware
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp

# 2. Build the workspace
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

# 3. Launch the full simulation (Gazebo + MoveIt + RViz)
ros2 launch ur5_moveit_config gazebo_sim.launch.py

# 4. (Separate terminal) Launch the C++ waypoint planner
ros2 launch ur5_controller planner.launch.py
```

---

## Documentation

| Document | Description |
|---|---|
| [docs/architecture.md](docs/architecture.md) | Node graph, topic routing, dual-brain data flow |
| [docs/environment_setup.md](docs/environment_setup.md) | Ubuntu 24.04, WSL2, ARM64 VM, Zenoh installation |
| [docs/development_workflow.md](docs/development_workflow.md) | Threaded Python pattern, controller switching, HDF5 logging |
| [docs/troubleshooting_and_reset.md](docs/troubleshooting_and_reset.md) | Cache purge, zombie node kill, Zenoh reset |
| [docs/operation_and_deployment.md](docs/operation_and_deployment.md) | Launch commands, trajectory execution, controller swap |
| [docs/migration_UB22_to_UB24.md](docs/migration_UB22_to_UB24.md) | Humble → Jazzy migration technical changelog |

---

**Maintainer:** Kalle Stewart — ICE Lab, MDA Space Research Project