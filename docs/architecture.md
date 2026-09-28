# System Architecture

The UR5 Digital Twin uses a **split-level (dual-brain) control** design: only one controller may command the simulated joints at a time. Brains hand off via `/controller_manager/switch_controller`.

**Runtime stack:** ROS 2 Jazzy · Gazebo Harmonic · `rmw_zenoh_cpp` · helper CLIs in `scripts/ur5` / `ur5_env.sh`

## 1. Dual-brain control flow

### Brain A — Kinematic / trajectory (default)

* **Controller:** `ur_manipulator_controller` (`JointTrajectoryController`, effort command interfaces)
* **Nodes:** `multi_waypoint_planner.cpp`, `interactive_control.py`, MoveIt `move_group`
* **Path:** Plan or replay joints → FollowJointTrajectory → Gazebo via `gz_ros2_control`
* **Spawn pose:** Candle (upright) — URDF `initial_value` + SRDF `candle_pose` (`shoulder_lift_joint = -1.5708`)

### Brain B — Effort / research (loaded inactive)

* **Controller:** `forward_effort_controller` (`JointGroupEffortController`)
* **Spawned:** inactive at bring-up (ready for intentional activation)
* **Handoff:** `switch_controllers` deactivates JTC, activates FEC; publish `Float64MultiArray` on `/forward_effort_controller/commands`
* **Active production scripts:** torque handoff is **not** in the default `interactive_control` path (archive / research only)

## 2. Core ROS 2 topics

### Telemetry
* `/joint_states` — `joint_state_broadcaster` (~100 Hz)
* `/robot_description` — URDF for RSP / MoveIt / gz plugin

### Commands
* `/ur_manipulator_controller/follow_joint_trajectory` — Brain A
* `/forward_effort_controller/commands` — Brain B (when active)

### Planning
* `/display_planned_path`, `/planning_scene`, `/monitored_planning_scene`
* `/clicked_point` — RViz Publish Point waypoints (current planner definition path)

### Simulation
* `/clock` — Gazebo → ROS bridge; all nodes `use_sim_time:=true`

## 3. Launch topology

```text
gazebo_sim.launch.py          # master (Zenoh RMW, GUI+RViz defaults)
  ├─ sim.launch.py            # gz + spawn + JSB→JTC→FEC(inactive) spawners
  ├─ clock bridge
  ├─ move_group.launch.py     # Timer ~6 s
  └─ moveit_rviz.launch.py    # Timer ~8 s (if start_rviz:=true)
```

| Mode | Command |
|---|---|
| GUI + RViz (default) | `ur5 simgui` / `run_sim` |
| Headless | `ur5 sim` |
| Candle reset | `ur5 reset` or `ur5 candle` |

## 4. Directory structure (active)

```text
ur5_digital_twin/
├── docs/                      # Operator + repair docs (this folder)
├── scripts/                   # ur5, ur5_env.sh, run_sim
├── ur5_description/           # URDF, meshes, sim.launch.py, ensure_controllers.py
├── ur5_moveit_config/         # SRDF, ros2_controllers.yaml, gazebo_sim.launch.py
└── ur5_controller/            # multi_waypoint_planner, interactive_control, planner_cli, waypoints/
```

See `OPERATOR_WALKTHROUGH.md` for day-to-day commands and `SYSTEM_MAP.md` for deeper static analysis.
