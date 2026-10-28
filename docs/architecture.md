# System Architecture

The platform is three ROS packages. `ur5_description` is the robot. `ur5_moveit_config` is MoveIt and the Gazebo bring-up. `ur5_controller` is both brains and the operator entry point.

## 1. The Dual-Brain Control Flow

`controller_manager` runs at 1 kHz. Exactly one of `ur_manipulator_controller` and `forward_effort_controller` is active. `joint_state_broadcaster` stays active in both modes.

### Phase A: The Kinematic Brain

`position_control.py` is Brain A. It switches on `ur_manipulator_controller`, sends a `FollowJointTrajectory` goal, and records HDF5 while the controller's PID writes effort. Cartesian CSVs are solved with `/compute_ik` and rejected if `/check_state_validity` reports a collision.

`multi_waypoint_planner` is the MoveIt node behind `ur5 plan` and `ur5 planner`. It exports joint or Cartesian CSV into `$UR5_DATA_DIR/waypoints/`.

The Candle pre-flight lives here: switch to Brain A, move to `candle_pose`, then hold for `candle.settle_sec` and require `max_error_rad` and `max_rms_velocity` from `platform.yaml`.

### Phase B: Torque control

`torque_control.py` is Brain B. After an atomic switch to `forward_effort_controller`, a worker thread computes one torque command per physics-stamped state sample. The sample stream is decimated to `topics.joint_states_rate_hz` (100 Hz). The ROS executor stays on the main thread.

### Handoff contract

`Brains.switch_to` calls `/controller_manager/switch_controller` with `STRICT` and checks the result: the requested controller is active and the other is not. Brain B is never left in authority without a torque stream. `ur5 brain b` holds the current pose with PD + gravity, then returns to Brain A. A torque trial returns to Candle under Brain A when the trial ends.

### Loop rates

The controller manager update rate is 1000 Hz (`ros2_controllers.yaml`). Brain B does not run a free 1 kHz timer. It wakes on each kept `/ur5/gz/joint_states` sample at 100 Hz. `use_sim_time` comes from the launch file, not from a literal in a node constructor.

### Control laws

`control_laws.py` registers `pd_gravity`, `pd_id_ff`, `computed_torque`, and `learned_id`. Each implements `compute(t, q, qd, q_ref, qd_ref, qdd_ref)`. Model terms are `rigid_body`, `viscous`, and `coulomb`. Viscous and Coulomb use `q̇_ref`. The computed-torque Coriolis term uses measured `q̇`. Coulomb is `tanh(q̇_ref / ε)`. Computed-torque gains use `joint_gain_match_at_candle`: the N·m gains are divided by `M_jj(q_candle)` once, before any perturbation.

## 2. Core ROS 2 Topic Network

### Telemetry & State Monitoring

| Topic | Who publishes | Who reads |
|---|---|---|
| `/joint_states` | `joint_state_broadcaster` | MoveIt, Brain A goal checks |
| `/ur5/gz/joint_states` | Gazebo joint-state publisher, physics stamp | Brain B and HDF5 |
| `/clock` | Gazebo bridge | every node (`use_sim_time:=true`) |

`/joint_states` order is not controller order. Every consumer reorders by joint name.

### Low-Level Hardware Command

Brain A: `ur_manipulator_controller` action `/ur_manipulator_controller/follow_joint_trajectory`. The command interface is effort. Brain B: `Float64MultiArray` on `/forward_effort_controller/commands`. Names are in `platform.yaml` so a hardware bring-up can retarget both brains without editing Python.

### Trajectory Planning & Validation

`/compute_ik` and `/check_state_validity` come from `move_group`. A waypoint file that fails the collision check or the joint limits never starts a motion. After a run, `telemetry.py validate` checks the HDF5 file. A Nyquist ratio above 8.5 marks the trial invalid. The torque runner reruns it, up to three attempts, and moves failures to `rejected/`.

### Simulation Control

`ros2 launch ur5_moveit_config gazebo_sim.launch.py` starts Gazebo Harmonic, `robot_state_publisher`, the controller spawners, `move_group`, optional RViz, and the Candle pre-flight (`run_preflight:=true`). Headless is `gz_headless:=true start_rviz:=false`.

## 3. Directory Structure and Module Responsibilities

| Path | Role |
|---|---|
| `ur5_description/urdf/ur5.urdf` | Geometry and inertias. `initial_value` is a `__CANDLE_<joint>__` placeholder. |
| `ur5_description/launch/sim.launch.py` | Injects `candle_pose` and the controller YAML, spawns the arm, starts the spawners. |
| `ur5_moveit_config/config/ur57_arm.srdf` | `candle_pose` is the only named group state. |
| `ur5_moveit_config/launch/gazebo_sim.launch.py` | The one bring-up command. |
| `ur5_controller/config/platform.yaml` | Topic names, controller names, Candle criteria. |
| `ur5_controller/config/experiment_*.yaml` | Brain B experiments. Internal names: `nominal_check`, `slice`, `benchmark`. |
| `ur5_controller/src/ur5_common.py` | Config, model, state cache, trajectory I/O, exit codes, data directory. |
| `ur5_controller/src/dynamics.py` | KDL `g`, `M`, `C`. |
| `ur5_controller/src/control_laws.py` | The four torque laws. |
| `ur5_controller/src/position_control.py` | Brain A, Candle, controller handoff. |
| `ur5_controller/src/torque_control.py` | Brain B loop, experiments, perturbations. |
| `ur5_controller/src/telemetry.py` | HDF5 writer and `validate`. |
| `ur5_controller/src/interactive_control.py` | Menu and flags. |
| `ur5_controller/src/planner_cli.py` | Planner menu. |
| `ur5_controller/src/multi_waypoint_planner.cpp` | MoveIt planner. |
| `ur5_controller/src/waypoints/` | Shipped trajectories. Each starts and ends at Candle. |
| `ur5_controller/src/data/` | Default `$UR5_DATA_DIR`. Contents are gitignored. |
| `scripts/ur5` | Command dispatcher. |
