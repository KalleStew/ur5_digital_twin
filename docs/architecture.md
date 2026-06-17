# System Architecture

The UR5 Digital Twin operates on a **Split-Level Control Architecture**. Because physical safety protocols dictate that only one controller can send voltage/commands to a motor at a time, the system features entirely independent "brains" that interact indirectly by competing for authority over the ROS 2 Controller Manager.

## 1. The Dual-Brain Control Flow

The core system consists of a C++ planner for safe positioning and a Python script for experimental AI control. They communicate via a sequential **"Handoff"**.

### Phase A: The Kinematic Brain (C++)
* **Role:** Safe navigation and positioning.
* **Node Example:** `multi_waypoint_planner.cpp`
* **Pathway:** Utilizes the MoveIt 2 framework and `MoveGroupInterface`.
* **Execution:** MoveIt calculates Inverse Kinematics (IK) and dispatches a trajectory to the `ur_manipulator_controller`. This controller commands the virtual motors to rigidly follow positional waypoints, successfully navigating around tracked collision objects (e.g., the 1.525m x 0.76m workbench).

### Phase B: The AI Sandbox (Python)
* **Role:** High-level AI, fault-detection, and raw torque manipulation.
* **Node Example:** `ai_torque_controller.py`
* **The Handoff (Service Client):** To take control, the Python script calls the `/controller_manager/switch_controller` service. It explicitly disables the `ur_manipulator_controller` (breaking the C++ planner's position lock) and enables the `forward_effort_controller`.
* **Execution:** Once enabled, the Python script bypasses MoveIt entirely, pushing raw numerical arrays (representing joint torques) directly to the motors. 

## 2. Core ROS 2 Topic Network

The system relies on the following primary topics to bridge MoveIt, Gazebo, and the custom scripts:

### Telemetry & State Monitoring
* `/joint_states`: The core telemetry topic. Published by `joint_state_broadcaster` at 100Hz. Subscribed to by the Python AI scripts for live encoder, velocity, and effort data.
* `/robot_description`: Broadcasts the compiled URDF/Xacro XML string. Consumed by `gz_ros2_control` and MoveIt to establish kinematics and collision limits.

### Low-Level Hardware Command
* `/forward_effort_controller/commands`: The active intake for dynamic control. Python scripts publish `Float64MultiArray` messages here, which are routed directly into the Gazebo physics engine.

### Trajectory Planning & Validation
* `/display_planned_path`: MoveIt publishes calculated, collision-free trajectories here. Subscribed to by `trajectory_exporter.cpp` to intercept timestamped joint angles for CSV export prior to execution.
* `/planning_scene`, `/planning_scene_world`, `/monitored_planning_scene`: MoveIt environments that dynamically synchronize rigid 3D obstacle geometry to prevent collisions.

### Simulation Control
* `/clock`: Published by Gazebo. All nodes run with `use_sim_time:=true`, ensuring controllers do not desynchronize or crash if the physics engine is paused.
* **Force Injection (gazebo_ros_force_system):** A dedicated topic allowing the publication of Wrench vectors to specific robot links to mathematically simulate random disturbances.

## 3. Directory Structure and Module Responsibilities

```text
src/
├── my_arm_description/           # Physical Data & Physics Engine Setup
│   ├── config/controllers.yaml   # Hardware interface mapping for Gazebo
│   ├── launch/sim.launch.py      # Master launch file for Gazebo + URDF spawning
│   ├── meshes/                   # STL files (visual and collision geometry)
│   └── urdf/ur5.urdf             # The master URDF with accurate mass/inertia tags
│
├── my_arm_moveit_config/         # MoveIt 2 Setup Assistant Outputs
│   ├── config/ros2_controllers.yaml # MoveIt controller tunings (PID gains)
│   ├── config/ur57_arm.srdf      # Semantic description (groups, end effectors)
│   └── launch/demo.launch.py     # MoveIt planning execution environment
│
└── my_arm_controller/            # Custom Control Logic (The "Brains")
    ├── src/
    │   ├── multi_waypoint_planner.cpp  # C++ Kinematic Brain (MoveIt API)
    │   ├── trajectory_exporter.cpp     # Intercepts /display_planned_path to CSV
    │   ├── ai_torque_controller.py     # Python AI Sandbox (Direct torque injection)
    │   ├── mode_switcher.py            # Dedicated service client for the "Handoff"
    │   └── research_control.py         # 100Hz /joint_states telemetry logger
    └── launch/planner.launch.py        # Initializes the custom control nodes