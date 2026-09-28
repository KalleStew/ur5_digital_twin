# SYSTEM_MAP — Phase 1 Static Model

**Generated:** 2026-09-22 · **Updated:** 2026-09-24 (restore / Candle / GUI defaults)  
**Branch / commit:** `v1-dev`  
**Host snapshot:** `aarch64`, Ubuntu 24.04.4 noble, `ROS_DISTRO=jazzy`, `RMW_IMPLEMENTATION=rmw_zenoh_cpp`

This document started as the Phase 1 static model. Sections marked **Updated** reflect post-repair code (Candle IC, Zenoh default, GUI restore, dual-brain FEC inactive). Historical discrepancy rows that are fixed keep a **FIXED** mark for audit trail.

---

## 0. File inventory (Phase 1 read set)

Every non-binary, non-`.git` source/config/doc file below was read. Meshes were inventoried by path only (STL binaries not parsed).

### Root
| Path | Role |
|---|---|
| `README.md` | Project overview / quick start |
| `.gitignore` | Ignore rules |

### `docs/`
| Path | Role |
|---|---|
| `docs/OPERATOR_WALKTHROUGH.md` | Day-to-day `ur5` commands (authoritative ops) |
| `docs/SYSTEM_MAP.md` | Static model + discrepancy audit |
| `docs/REPAIR_LOG.md` | Repair iterations + restore notes |
| `docs/architecture.md` | Dual-brain topology |
| `docs/development_workflow.md` | Threading + switch_controller patterns |
| `docs/environment_setup.md` | Host / ROS / Zenoh / Gazebo setup |
| `docs/troubleshooting_and_reset.md` | Process purge + Candle reset |
| `docs/operation_and_deployment.md` | Launch / operate procedures |
| `docs/migration_UB22_to_UB24.md` | Humble→Jazzy migration changelog |
| `docs/external_resources.md` | External doc index (still Humble-centric) |

### `ur5_description/`
| Path | Role |
|---|---|
| `package.xml` | Package manifest |
| `CMakeLists.txt` | Install urdf/meshes/launch/config/worlds |
| `urdf/ur5.urdf` | Robot model + ros2_control + gz plugin |
| `config/controllers.yaml` | **Stale** alternate controller YAML (unused by launch) |
| `launch/sim.launch.py` | Gazebo + spawn + JSB→JTC→FEC(inactive) spawners |
| `scripts/ensure_controllers.py` | Recovery-only Candle/JTC activation (`ur5 candle`) |
| `worlds/empty.sdf` | Empty ODE world |
| `meshes/{visual,collision}/*.stl` | 14 STL files each (base + 6 links + collision variants) |

### `ur5_moveit_config/`
| Path | Role |
|---|---|
| `package.xml` | MoveIt package (still lists Humble-era deps) |
| `CMakeLists.txt` | Install launch + config |
| `.setup_assistant` | Points to `ur5.urdf` + `ur57_arm.srdf` |
| `config/ur57_arm.srdf` | Semantic model + `candle_pose` / `ready` group states |
| `config/ros2_controllers.yaml` | Active controller definitions |
| `config/moveit_controllers.yaml` | MoveIt → FollowJointTrajectory mapping |
| `config/joint_limits.yaml` | MoveIt velocity/accel scaling |
| `config/kinematics.yaml` | KDL solver |
| `config/pilz_cartesian_limits.yaml` | Pilz limits |
| `config/moveit.rviz` | RViz config |
| `launch/gazebo_sim.launch.py` | Master: sim + move_group + rviz + clock |
| `launch/move_group.launch.py` | MoveItConfigsBuilder → move_group |
| `launch/moveit_rviz.launch.py` | MoveItConfigsBuilder → RViz |
| `launch/rsp.launch.py` | MoveItConfigsBuilder → RSP (not used by master) |
| `launch/spawn_controllers.launch.py` | MoveItConfigsBuilder spawners (not used by master) |
| `launch/demo.launch.py` | Setup-assistant demo |
| `launch/setup_assistant.launch.py` | MoveIt Setup Assistant |
| `launch/static_virtual_joint_tfs.launch.py` | Virtual joint TFs |
| `launch/warehouse_db.launch.py` | Warehouse DB |

### `ur5_controller/`
| Path | Role |
|---|---|
| `package.xml` | C++ planner package (+ unused Python deps declared) |
| `CMakeLists.txt` | Builds/installs **only** `multi_waypoint_planner` |
| `launch/planner.launch.py` | Launches planner with MoveIt params |
| `src/multi_waypoint_planner.cpp` | **ACTIVE** Brain A |
| `src/interactive_control.py` | **ACTIVE** HIL trajectory replay + HDF5 (Brain A path) |
| `src/planner_cli.py` | **ACTIVE** `/planner_command` publisher CLI |
| `src/waypoints/default_joint.csv` | 7-DOF joint waypoints |
| `src/waypoints/default_cartesian.csv` | Cartesian XYZRPY waypoints |
| `src/data/*.csv`, `*.h5` | Prior telemetry artifacts |
| `src/archive/v1/*` | Archived Brain B / switch prototypes |
| `src/archive/v2/*` | Archived joint/cartesian HIL scripts |

**Absent relative to §2.4 expected layout (names only):** `ur5.srdf` (file is `ur57_arm.srdf`), `conventional_research.py`, `ai_research.py`, `unified_control.py` (active), `setup.py` / `setup.cfg`.  
**Present now:** `candle_pose` in `ur57_arm.srdf`, URDF Candle IC, FEC inactive spawner, Zenoh launch default, GUI+RViz defaults.

---

## 1. Intended functionality (design intent)

A physics-enabled 7-DOF UR5 digital twin on ROS 2 Jazzy + Gazebo Harmonic that (A) plans Cartesian multi-waypoint paths with MoveIt and tracks them through an effort-commanded joint trajectory controller, and (B) transfers exclusive control authority to a raw effort controller so research scripts can inject 7-wide torque arrays at 100 Hz while logging `/joint_states` to HDF5 — with candle-pose reset via `switch_controller`, never by restarting Gazebo.

---

## 2. Authoritative 7-joint name list and ordering

### Canonical ordering (controller command order)

Declared identically in `ur5_moveit_config/config/ros2_controllers.yaml` for both `ur_manipulator_controller` and `forward_effort_controller`, and mirrored in `moveit_controllers.yaml`:

| Index | Joint name |
|------:|---|
| 0 | `shoulder_pan_joint1` |
| 1 | `shoulder_pan_joint` |
| 2 | `shoulder_lift_joint` |
| 3 | `elbow_joint` |
| 4 | `wrist_1_joint` |
| 5 | `wrist_2_joint` |
| 6 | `wrist_3_joint` |

URDF kinematic chain order (`ur5_description/urdf/ur5.urdf`) matches this sequence (base → tip). Fixed joints (`world_to_table`, `table_to_robot`, `ee_fixed_joint`, `wrist_3_link_tool0_fixed_joint`) are not actuated.

### Ordering agreement matrix

| Source | Ordering | Verdict |
|---|---|---|
| URDF revolute joints (base→tip) | 0…6 as above | **Agree** |
| `ros2_controllers.yaml` → `ur_manipulator_controller.joints` | same | **Agree** (authoritative for Brain A cmds) |
| `ros2_controllers.yaml` → `forward_effort_controller.joints` | same | **Agree** (authoritative for Brain B cmds) |
| `moveit_controllers.yaml` | same | **Agree** |
| `ur57_arm.srdf` `ready` / `candle_pose` | same 7 names; candle lift −1.5708 | **Agree** Candle; `ready` still all zeros |
| `interactive_control.py` `self.joint_names` | same | **Agree** (hardcoded, not runtime-resolved) |
| `multi_waypoint_planner.cpp` export | uses `joint_traj.joint_names` from MoveIt | **Runtime-dependent**; capped to 7 |
| `default_joint.csv` header | same | **Agree** |
| `ur5_description/config/controllers.yaml` (`arm_controller`) | same names, **position** cmd iface | **Stale / unused**; disagrees on interface |
| `archive/v1/research_control.py` | **6 joints**, omits `shoulder_pan_joint1` | **Disagree** (archived) |
| Docs referencing `ur5.srdf` | File is `ur57_arm.srdf` | **Name mismatch only** |

**Runtime unresolved:** `/joint_states` name order from `joint_state_broadcaster` is not guaranteed to match controller `joints:` order. Active scripts that index by name (`interactive_control.py`) are safe; any positional `msg.position[:7]` use (docs examples, archived loggers) is unsafe until measured at S4.

---

## 3. URDF / hardware interface summary

### Actuated joints (limits from URDF)

| Joint | Type | Effort | Velocity | Accel | Jerk | Position range |
|---|---|---:|---:|---:|---:|---|
| `shoulder_pan_joint1` | revolute | 150 | π | 5 | 10 | ±±2π |
| `shoulder_pan_joint` | revolute | 150 | π | 5 | 10 | ±±2π |
| `shoulder_lift_joint` | revolute | 150 | π | 5 | 10 | ±±2π |
| `elbow_joint` | revolute | 150 | π | 5 | 10 | ±±π |
| `wrist_1_joint` | revolute | 28 | π | 5 | 10 | ±±2π |
| `wrist_2_joint` | revolute | 28 | π | 5 | 10 | ±±2π |
| `wrist_3_joint` | revolute | 28 | π | 5 | 10 | ±±2π |

Per-joint Gazebo PID params exist under each `<ros2_control>` joint (siblings of interfaces). Damping/friction set via `<dynamics>`.

### Workbench collision
- Link `table_link`: box `1.525 × 0.76 × 0.99` m, visual/collision origin `xyz="0 0 -0.495"`.
- `table_to_robot`: `xyz="0 0.28 0"`, `rpy="0 0 -1.5708"`.

### `ros2_control` block
- Hardware plugin: `gz_ros2_control/GazeboSimSystem`
- Each joint exposes **both** `command_interface`: `position` **and** `effort`
- State interfaces: `position`, `velocity`, `effort`
- Gazebo plugin: `gz_ros2_control-system` / `gz_ros2_control::GazeboSimROS2ControlPlugin`
- `<parameters>__ROS2_CONTROLLERS_YAML__</parameters>` substituted at launch to absolute path of `ros2_controllers.yaml`

---

## 4. Controllers table (Brain ownership)

| Controller | Type | Joints order | Command iface | State iface | PID / notes | Owner | Spawned? |
|---|---|---|---|---|---|---|---|
| `joint_state_broadcaster` | `JointStateBroadcaster` | (all HW joints) | — | pos/vel/eff | — | Shared telemetry | **Yes** (sim.launch) |
| `ur_manipulator_controller` | `JointTrajectoryController` | canonical 7 | **effort** | position, velocity | Per-joint `gains:` in YAML | **Brain A** | **Yes** (sim.launch, active) |
| `forward_effort_controller` | `effort_controllers/JointGroupEffortController` | canonical 7 | YAML lists `command_interfaces: [effort]` | `position` | No `interface_name:` key (docs claim it) | **Brain B** | **No** — never spawned |

`controller_manager.update_rate`: **100 Hz**.

MoveIt maps Brain A via `FollowJointTrajectory` action:
`/ur_manipulator_controller/follow_joint_trajectory`.

---

## 5. Node / topic / service graph

### Mode A — Kinematic (as launched today)

```text
gazebo_sim.launch.py
├── SetParameter(use_sim_time=true)          [global for this launch context]
├── [optional] rmw_zenohd                    [start_zenoh_router:=false default]
├── Timer(1.0s) → sim.launch.py
│   ├── robot_state_publisher               [/robot_description]
│   ├── gz_sim (ros_gz_sim) -r empty.sdf
│   ├── ros_gz_sim create (spawn ur5_arm)
│   ├── spawner joint_state_broadcaster     [--controller-manager-timeout 30]
│   └── spawner ur_manipulator_controller   [--controller-manager-timeout 30]
├── Timer(1.1s) → ros_gz_bridge /clock
├── Timer(6.0s) → move_group.launch.py
└── Timer(8.0s) → moveit_rviz.launch.py

planner.launch.py (separate process)
└── multi_waypoint_planner
    ├── sub /clicked_point (PointStamped)
    ├── sub /planner_command (String)
    ├── pub /display_planned_path (DisplayTrajectory)
    └── MoveGroupInterface("ur_manipulator") → MoveIt →
         action /ur_manipulator_controller/follow_joint_trajectory
```

### Mode B — Torque injection (documented intent; **not wired in active code**)

```text
[MISSING] spawner forward_effort_controller --inactive
[MISSING] conventional_research.py / ai_research.py
Intended:
  Python thread → Float64MultiArray → /forward_effort_controller/commands
  switch_controller: deactivate ur_manipulator_controller, activate forward_effort_controller
  sub /joint_states → HDF5
```

Active `interactive_control.py` stays on Brain A (`FollowJointTrajectory` + optional `/compute_ik`). It does **not** call `switch_controller` and does **not** publish torques.

### Primary interfaces

| Kind | Name | Type / notes | Expected rate |
|---|---|---|---|
| Topic | `/clock` | `rosgraph_msgs/Clock` via bridge | sim step |
| Topic | `/joint_states` | `sensor_msgs/JointState` | ~100 Hz (CM rate / RTF) |
| Topic | `/robot_description` | `std_msgs/String` (latched param/topic) | once |
| Topic | `/clicked_point` | `geometry_msgs/PointStamped` | event |
| Topic | `/planner_command` | `std_msgs/String` | event |
| Topic | `/display_planned_path` | `moveit_msgs/DisplayTrajectory` | event |
| Topic | `/forward_effort_controller/commands` | `std_msgs/Float64MultiArray` | 100 Hz (Mode B) |
| Action | `/ur_manipulator_controller/follow_joint_trajectory` | `control_msgs/FollowJointTrajectory` | Mode A |
| Service | `/controller_manager/switch_controller` | `controller_manager_msgs/SwitchController` | handoff |
| Service | `/controller_manager/list_controllers` | list | inspect |
| Service | `/compute_ik` | `moveit_msgs/GetPositionIK` | Mode A IK |
| TF | `world`→`table_link`→`base_link`→…→`tool0` | via RSP + joint states | continuous |

---

## 6. Brain handoff state machine (contract)

### Pre-conditions (A→B)
1. `controller_manager` available; hardware interfaces claimed only by active controllers.
2. `ur_manipulator_controller` **active**; `forward_effort_controller` **loaded and inactive**.
3. Arm held near a known pose (documented: Candle Pose). No open FollowJointTrajectory goals.
4. Research node uses sim time; control logic not on the spin thread.

### Transition A→B (single atomic request)
```
activate:   [forward_effort_controller]
deactivate: [ur_manipulator_controller]
strictness: STRICT (preferred) | BEST_EFFORT (archived prototypes)
```
**Post-conditions:** response `ok == true`; list_controllers shows A inactive, B active; joint-state continuous (no free-fall for > ε under gravity compensation / immediate torque publish).

### Transition B→A
```
activate:   [ur_manipulator_controller]
deactivate: [forward_effort_controller]
```
**Post-conditions:** A active, B inactive; then command Candle Pose and verify `/joint_states` within tolerance — **without** restarting Gazebo.

### As-coded gap (historical — updated 2026-09-24)
`forward_effort_controller` is now spawned **inactive** at bring-up. Intentional B↔A handoff still has no active production script (`mode_switcher.py` / `ai_torque_controller.py` remain archive-only). Default Stage-3 path is JTC only.

---

## 7. Launch graph details

### `sim.launch.py` (**Updated**)
| Item | Value |
|---|---|
| URDF load | share `ur5_description/urdf/ur5.urdf` with YAML path injection |
| Gazebo args | Composed: optional `-s` (`gz_headless`), optional `--render-engine`, then `-r {empty.sdf}` |
| Spawn | `-string` URDF, name `ur5_arm`; joint IC via URDF `initial_value` (Candle) |
| Spawner timeout | 60 s; **2 s** TimerAction after `create` before JSB |
| Sequencing | `OnProcessExit(spawn→delayed JSB→JTC→FEC inactive)` |
| Effort controller | Spawned **inactive** |

### `gazebo_sim.launch.py` (**Updated**)
| Item | Value |
|---|---|
| Global `use_sim_time` | `SetParameter(True)` |
| Default RMW | **`rmw_zenoh_cpp`** |
| `gz_headless` / `start_rviz` | defaults **`false` / `true`** (operator GUI path) |
| Zenoh router | off by default; optional `start_zenoh_router:=true` (prefer `ur5zenoh_restart`) |
| Clock bridge | `/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock` @ 1.1 s |
| move_group delay | 6.0 s wall |
| RViz delay | 8.0 s wall |

### `planner.launch.py`
- `MoveItConfigsBuilder("ur57_arm", package_name="ur5_moveit_config")`
- Passes robot_description, semantic, kinematics, planning_pipelines, joint_limits, trajectory_execution, planning_scene_monitor, `use_sim_time: True`
- Executable: `multi_waypoint_planner` only

---

## 8. Active control script traces

### `multi_waypoint_planner.cpp` (Brain A)
- Node ctor forces `use_sim_time=true`.
- **Threading:** `MultiThreadedExecutor::spin()` on a **background** thread; `run_terminal_loop()` (blocking stdin) on **main**. Inverts the Python convention in §1.3; acceptable for stdin ownership but callbacks share the background executor.
- Command path: RViz click → waypoint vector → `computeCartesianPath` → optional `execute` via MoveGroup.
- Export writes CWD CSVs (`trajectory_joints_export.csv`, `trajectory_cartesian_export.csv`).
- Mutexes protect waypoint/trajectory state; commands serialized by `command_mutex_`.

### `interactive_control.py` (active HIL; Brain A only)
- Main: `rclpy.spin`; worker daemon thread: CLI + action goal + export.
- IK via async `call_async` + `threading.Event` (structurally OK).
- Hardcoded joint name list (canonical). Telemetry reorders by name.
- Wall-clock `trajectory_event.wait(timeout=max_time)` and IK `wait(timeout=2.0)` — **not sim-time**.
- HDF5 written on worker thread after recording stops; then `rclpy.shutdown()`.
- **No** `switch_controller`, **no** torque publisher.

### `planner_cli.py`
- Publishes `String` to `/planner_command`; no `use_sim_time`; `spin_once` after each publish.

### Build / install
- CMake installs **only** the C++ binary + launch dir.
- Python scripts are **not** installed as executables; must be run from source tree paths.
- Archive trees are **not** referenced by CMakeLists (good for A14 build exclusion).

---

## 9. SRDF / Candle Pose

| Claim | Reality (as of 2026-09-24 restore) |
|---|---|
| File `ur5.srdf` | **Does not exist**; file is `config/ur57_arm.srdf` |
| Group state `candle_pose` with `shoulder_lift_joint=-1.5708` | **Present** in `ur57_arm.srdf` |
| URDF spawn IC | `ur5.urdf` `state_interface` `initial_value` — especially `shoulder_lift_joint=-1.5708` |
| Group | `ur_manipulator` chain `base_link`→`tool0` |
| Runtime reset | `ur5 reset` / `ur5 candle` — no Gazebo restart |

Candle clears the workbench at first physics step. Verify with `ur5 states` (`shoulder_lift ≈ -1.57`).

---

## 10. Documentation vs code discrepancies

| ID | Claim | Reality | Mark | Reasoning |
|---|---|---|---|---|
| D1 | `candle_pose` in `ur5.srdf` | `candle_pose` in `ur57_arm.srdf` + URDF IC | **FIXED in code** (2026-09) | Filename still `ur57_arm.srdf` not `ur5.srdf` |
| D2 | Brain B: `conventional_research.py` / `ai_research.py` | Files absent; archive only | **DOC-WRONG** | Active tree has `interactive_control.py` (Brain A) |
| D3 | `gazebo_sim` loads both brains' controllers | JSB+JTC active; FEC spawned **inactive** | **FIXED in code** | Dual-brain ready; Stage-3 path remains JTC |
| D4 | Zenoh is default middleware | Launch default `rmw_zenoh_cpp` | **FIXED in code** | Matches `ur5env` |
| D5 | `interactive_control` performs handoff | No switch / no torque pub in active path | **DOC-WRONG** | Torque handoff remains research/archive |
| D6 | `forward_effort` YAML has `interface_name: effort` | JointGroupEffortController; no JTC-only keys | **Agree** | See `ros2_controllers.yaml` |
| D7 | Package deps cleaned for Jazzy | `gazebo_ros_control`, `warehouse_ros_mongo` still in moveit `package.xml` | **CODE-WRONG** vs migration doc | Migration claims removal; files still list Humble packages |
| D8 | `ur5_description` is 7-DOF | `package.xml` description says "6-DOF" | **CODE-WRONG** (cosmetic) | Manifest text stale |
| D9 | Master launch is single MoveItConfigsBuilder | `gazebo_sim` includes separate move_group/rviz launches each calling builder again | **DOC-WRONG** | Migration §6 describes a unified builder; current code re-instantiates |
| D10 | Reset never restarts Gazebo | `ur5 reset` / `ur5 candle` primary; kill/relaunch only for orphans | **FIXED in ops docs** | See `troubleshooting_and_reset.md` |
| D11 | README quick start complete | Commands correct names (`planner.launch.py`) | **Agree** on truncated-name claim | Truncation mentioned in mission brief not present in current README |
| D12 | `unified_control.py` active | Archived / missing | **DOC-WRONG** | README structure section |
| D13 | external_resources recommends CycloneDDS | Project mandates Zenoh | **DOC-WRONG** | Stale Humble-era advice |
| D14 | `trajectory_exporter.cpp` exists | Does not; export is inside planner | **DOC-WRONG** | architecture.md |
| D15 | Default launch is headless | Defaults GUI+RViz (`gz_headless=false`, `start_rviz=true`) | **FIXED 2026-09-24** | Headless via `ur5 sim` only |

---

## 11. Assumptions not verifiable statically

| # | Assumption | Discriminating runtime experiment |
|---|---|---|
| U1 | `/joint_states` name order equals controller `joints:` order | S4: print both; assert name-index map |
| U2 | Dual `position`+`effort` command interfaces do not double-claim under JTC effort mode | S2: `list_hardware_interfaces` claim flags with only JTC active |
| U3 | `JointGroupEffortController` accepts `command_interfaces` key (vs `interface_name`) | Spawn inactive controller; capture spawner stderr |
| U4 | Wall-clock TimerAction delays (1/6/8 s) suffice under ARM64 low RTF | S5 soak: measure time-to-`/controller_manager` and move_group ready |
| U5 | Empty world + default ogre2 renders on this VM's llvmpipe | §4.1: `gz sim -v4 --render-engine ogre shapes.sdf` |
| U6 | Zenoh router required before any node when `RMW=rmw_zenoh_cpp` | Start nodes without `rmw_zenohd`; observe empty graph |
| U7 | Launch default FastDDS vs shell Zenoh causes split-brain discovery | Launch with defaults while shell exports Zenoh; `ros2 node list` from second shell |
| U8 | All-zero spawn collides with table before JTC activates | S1 visual / contact logs at spawn |
| U9 | MoveIt `ready` named state is reachable and collision-free | S6 plan to `ready` |
| U10 | Effort-mode JTC gains hold arm against gravity at RTF≪1 | S3–S6: sag measurement after activate |
| U11 | Python scripts' CWD-relative `waypoints/` / `data/` paths resolve when launched from repo `src/` | Run `interactive_control.py` from documented cwd |
| U12 | `computeCartesianPath` succeeds around workbench with current SRDF disables | S6 multi-waypoint around table |

---

## 12. Top suspected defects (static ranking)

Historical Phase-1 ranking. Status after repair/restore:

| Rank | Defect | Status (2026-09-24) |
|-----:|---|---|
| 1 | `forward_effort_controller` never spawned | **Fixed** — spawned inactive |
| 2 | No Candle Pose / all-zero spawn | **Fixed** — URDF IC + SRDF `candle_pose` |
| 3 | Launch forces FastDDS vs Zenoh | **Fixed** — default `rmw_zenoh_cpp` |
| 4 | Mode B production path missing | **Open** — archive-only torque handoff |
| 5 | Wall-clock timeouts under low RTF | **Mitigated** — longer spawner timeout + 2 s post-spawn delay |

Remaining ops risk: orphaned `gz sim` → duplicate CM (purge with `ur5kill` / `hard_clean.sh`).

---

## 13. Archived scripts (non-production)

| Path | Notes |
|---|---|
| `archive/v1/ai_torque_controller.py` | switch + torque; **deadlock risk**: `spin_until_future_complete` on main before `spin` |
| `archive/v1/mode_switcher.py` | STRICT switch CLI |
| `archive/v1/research_control.py` | **6-DOF** waypoints — defect if ever reactivated |
| `archive/v1/move_arm.cpp`, `arm_controller.cpp` | Not in CMake |
| `archive/v2/conventional_*.py`, `conv_control_template.py` | HIL / candle helper prototypes |

None are installed by CMake. Keep excluded from build and launches (A14).

---

## 14. Frozen CSV schemas (A4 — do not “improve”)

### Joint-space waypoints (read by `interactive_control.py` / written by planner `export joints`)

| Property | Value |
|---|---|
| Example file | `ur5_controller/src/waypoints/default_joint.csv` |
| Planner export name | `trajectory_joints_export.csv` (CWD — **not** yet `waypoints/`) |
| Header | **required**, first row skipped by reader |
| Columns | `time_sec`, then 7 joint angles in **controller YAML order**: `shoulder_pan_joint1`, `shoulder_pan_joint`, `shoulder_lift_joint`, `elbow_joint`, `wrist_1_joint`, `wrist_2_joint`, `wrist_3_joint` |
| Units | `time_sec` = seconds (float); joints = **radians** |
| Row rule | ≥ 8 fields; `positions = row[1:8]`; velocities forced to 0 in HIL |

### Cartesian waypoints (read by `interactive_control.py` mode 2 / planner `export cartesian`)

| Property | Value |
|---|---|
| Example file | `ur5_controller/src/waypoints/default_cartesian.csv` |
| Planner export name | `trajectory_cartesian_export.csv` (CWD) |
| Header | **required** |
| Columns | `time_sec,x,y,z,roll,pitch,yaw` |
| Units | seconds; metres; **radians** (RPY) |
| Frame (IK request) | `world` in `interactive_control.py` |

### Telemetry export (`interactive_control.py`)

| Property | Value |
|---|---|
| Paths | `data/<name>_filtered.csv`, `data/<name>_filtered.h5` (CWD-relative) |
| CSV columns | `timestamp_sec`, then per joint in YAML order: `{name}_pos`, `{name}_vel`, `{name}_torque` |
| HDF5 | `timestamp_sec`; `position/{name}`, `velocity/{name}`, `effort/{name}` |

---

## 15. Runtime ordering (ARM64 VM, 2026-09-22)

| Index | `/joint_states.name` (alpha) | Controller `joints:` (YAML) |
|------:|---|---|
| 0 | elbow_joint | shoulder_pan_joint1 |
| 1 | shoulder_lift_joint | shoulder_pan_joint |
| 2 | shoulder_pan_joint | shoulder_lift_joint |
| 3 | shoulder_pan_joint1 | elbow_joint |
| 4 | wrist_1_joint | wrist_1_joint |
| 5 | wrist_2_joint | wrist_2_joint |
| 6 | wrist_3_joint | wrist_3_joint |

**Disagree.** Map by name for reads; command arrays follow YAML order.

---

## 16. A4 stage mapping

See `docs/REPAIR_LOG.md` (authoritative running table). Summary: stages 1–3 **partial**; stage 4 export **partial**, reset **absent**.

---

## 17. Phase 2 progress

| Item | Status |
|---|---|
| Q3 headless/render | **Done** — D-001 PASS (`gz_headless`, `gz_render_engine`, `start_rviz`) |
| Q4 spawn pose | **Proven broken** — all-zero; candle absent; patch deferred (stop after item 3) |
| Q5 ordering | **Mapped** — alpha vs YAML disagreement confirmed |
| Q2 / Q1 step 3 | Not started (needs `forward_effort` spawn) |
)
