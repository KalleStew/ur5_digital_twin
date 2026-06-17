# Operation Guide

This document outlines the standard operating procedures for launching the UR5 Digital Twin, running kinematic path planning, and executing the dynamic torque-control AI handoff.

Because ROS 2 relies on a decentralized node architecture, you will need to run these processes in separate terminal windows. 

**Prerequisite:** Every new terminal must source the workspace before running any ROS 2 commands.
```bash
cd ~/ros2_ws
source install/setup.bash
```

---

## 1. Launching the Master Simulation Environment (Terminal 1)

The first step is to boot the core physics engine, the visualization environment, and the controller manager. 

**Command:**
```bash
ros2 launch ur5_moveit_config gazebo_sim.launch.py
```

**What this does:**
* Spawns the physical URDF model inside the Gazebo physics engine.
* Launches RViz2 for 3D trajectory visualization.
* Initializes the `controller_manager` and loads both the `ur_manipulator_controller` (position) and `forward_effort_controller` (torque).
* Synchronizes the simulation clock (`use_sim_time:=true`) across the ROS network to prevent node desynchronization.

---

## 2. Running the C++ Waypoint Planner (Terminal 2)

Once the simulation is running, open a second terminal to launch the conventional kinematic planner.

**Command:**
```bash
ros2 launch ur5_controller planner.launch.py
```

**What this does:**
* Initializes the `MoveGroupInterface` to calculate Inverse Kinematics (IK) and avoid collisions.
* Uses a dedicated launch file rather than a standard `ros2 run` command to properly inject required IK parameters into the node.
* Bypasses a known ROS 2 terminal-hanging bug, ensuring your keyboard inputs remain active for interactive Cartesian planning.

---

## 3. Executing the Custom Control & AI Handoff (Terminal 3)

For advanced research, fault-detection, or Hardware-in-the-Loop (HIL) testing, open a third terminal to run the unified control pipeline.

**Command:**
```bash
python3 unified_control.py
```

**What this does:**
* **The Handoff:** Calls the `/controller_manager/switch_controller` service to instantly disable the C++ position controller and enable raw torque control.
* **Execution:** Pumps customized trajectories or electrical torque arrays directly into the Gazebo motors (or physical hardware) bypassing MoveIt entirely.
* **Real-Time Logging:** Subscribes to `/joint_states` and records high-frequency telemetry (Position, Velocity, and PID Effort) at 100Hz.
* **Auto-Export:** Automatically formats and exports the telemetry to a localized CSV file the exact millisecond the arm completes its movement for post-run analysis.

---

## Expected Outputs & Validation

* **Successful Handoff:** When running `unified_control.py`, watch Terminal 1. You should see a green terminal output from the `controller_manager` confirming `ur_manipulator_controller` stopped and `forward_effort_controller` started.
* **Simulation Pausing:** If Gazebo is paused (the play button in the bottom left is not active), the `/clock` topic will stop ticking. Your planners will wait indefinitely until the simulation is unpaused.