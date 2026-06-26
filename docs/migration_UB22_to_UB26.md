# Migration: Ubuntu 22.04 / ROS 2 Humble → Ubuntu 26.04 / ROS 2 Lyrical Luthien

**Branch:** `feature/ubuntu26-ll-migration`
**Date:** 2026-06-25

This document is the technical changelog for the migration. Each section lists the exact file, the category of change, the before/after state, and the reason.

---

## Table of Contents

1. [Phase 0 — Git Branch](#phase-0--git-branch)
2. [Phase 1 — Workspace & Middleware Modernization](#phase-1--workspace--middleware-modernization)
3. [Phase 2 — URDF & Gazebo Plugin Upgrades](#phase-2--urdf--gazebo-plugin-upgrades)
4. [Phase 3 — C++ API & CMakeLists](#phase-3--c-api--cmakelists)
5. [Phase 4 — MoveIt 2 & Launch File Restructuring](#phase-4--moveit-2--launch-file-restructuring)
6. [Pre-existing Bugs Fixed](#pre-existing-bugs-fixed)
7. [Verification Checklist](#verification-checklist)

---

## Phase 0 — Git Branch

```bash
git checkout -b feature/ubuntu26-ll-migration
```

---

## Phase 1 — Workspace & Middleware Modernization

### `ur5_description/package.xml`

| | Before | After |
|---|---|---|
| Dependency | `ign_ros2_control` | `gz_ros2_control` |

**Reason:** `ign_ros2_control` is the Ignition-era package name, fully retired in ROS 2 LL. `gz_ros2_control` is the canonical Gazebo Harmonic+ package.

### `ur5_moveit_config/package.xml`

| | Before | After |
|---|---|---|
| Dependency | `gazebo_ros_control` | `gz_ros2_control` |
| Dependency | `warehouse_ros_mongo` | `warehouse_ros_sqlite` |

**Reason:** `gazebo_ros_control` was the classic Gazebo (pre-Ignition) integration package. `warehouse_ros_mongo` is no longer the default backend for the MoveIt warehouse in LL; `warehouse_ros_sqlite` ships as the default.

### `.bashrc` — Eclipse Zenoh middleware (manual step)

```bash
# ROS 2 Lyrical Luthien
source /opt/ros/ll/setup.bash
source ~/ros2_ws/install/setup.bash

# Eclipse Zenoh — Tier 1 RMW for HIL high-frequency torque channels
export RMW_IMPLEMENTATION=rmw_zenoh_cpp

# Gazebo resource path (Ignition path retired)
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/ros2_ws/install/ur5_description/share
```

**Reason:** FastDDS adds XML-configured participant discovery round-trips on every new publisher/subscriber pair. At 100 Hz with raw torque arrays on the `forward_effort_controller` channel, this creates measurable latency spikes. Zenoh uses broker-less pub/sub with zero-copy transport, which is optimal for the HIL architecture.

**Installation:**
```bash
sudo apt install ros-ll-rmw-zenoh-cpp
```

---

## Phase 2 — URDF & Gazebo Plugin Upgrades

**File:** `ur5_description/urdf/ur5.urdf`

### 2.1 — Robot name standardized

| | Before | After |
|---|---|---|
| `<robot name=...>` | `ur57_arm` | `ur5_arm` |

**Reason:** The `7` was a typo. The SRDF, MoveItConfigsBuilder calls, and Gazebo spawn arguments all referenced `ur5_arm`. This mismatch caused silent SRDF loading failures where MoveIt could not match the semantic description to the robot model.

### 2.2 — URDF 1.2 acceleration and jerk limits

Added `acceleration="5.0" jerk="10.0"` to the `<limit>` tag of all 7 revolute joints.

**Example (shoulder_pan_joint1):**
```xml
<!-- Before -->
<limit effort="150.0" velocity="3.14159" lower="-6.28318" upper="6.28318"/>

<!-- After -->
<limit effort="150.0" velocity="3.14159" acceleration="5.0" jerk="10.0" lower="-6.28318" upper="6.28318"/>
```

**Applied to:** `shoulder_pan_joint1`, `shoulder_pan_joint`, `shoulder_lift_joint`, `elbow_joint`, `wrist_1_joint`, `wrist_2_joint`, `wrist_3_joint`.

**Reason:** Gazebo Harmonic+ (Gz-Harmonic and later) honours the URDF 1.2 `acceleration` and `jerk` attributes in the physics integrator (DART/Bullet). Without them, the solver has no ceiling on joint acceleration, so large P-gains (30000 on `shoulder_lift_joint`) cause the integrator to accumulate unbounded velocity in the first few ticks — manifesting as the arm "exploding" through the table.

### 2.3 — `<ros2_control>` hardware plugin renamed

```xml
<!-- Before -->
<ros2_control name="IgnitionSystem" type="system">
    <hardware>
        <plugin>ign_ros2_control/IgnitionSystem</plugin>
    </hardware>

<!-- After -->
<ros2_control name="GazeboSystem" type="system">
    <hardware>
        <plugin>gz_ros2_control/GazeboSystem</plugin>
    </hardware>
```

**Reason:** The `ign_ros2_control` namespace is retired. `gz_ros2_control/GazeboSystem` is the LL-canonical plugin identifier for the Gazebo Harmonic+ hardware system.

**P-gain structure confirmed correct — no change required:** The existing layout placing `<param name="p">` tags as direct siblings to `<command_interface name="position">` inside each `<joint>` block is the exact structure `gz_ros2_control/GazeboSystem` requires. The internal Gazebo position PID defaults to `p=0.1` if these params are absent or misplaced, which causes the arm to collapse under gravity.

### 2.4 — `<gazebo>` plugin block updated; hardcoded path removed

```xml
<!-- Before -->
<gazebo>
    <plugin filename="ign_ros2_control-system"
            name="ign_ros2_control::IgnitionROS2ControlPlugin">
        <parameters>/home/kallestewart/ros2_ws/src/.../ros2_controllers.yaml</parameters>
    </plugin>
</gazebo>

<!-- After -->
<gazebo>
    <plugin filename="gz_ros2_control-system"
            name="gz_ros2_control::GazeboROS2ControlPlugin">
        <!-- Path injected at launch time via MoveItConfigsBuilder in gazebo_sim.launch.py -->
    </plugin>
</gazebo>
```

**Reason (plugin rename):** `ign_ros2_control-system` / `IgnitionROS2ControlPlugin` are retired symbols. `gz_ros2_control-system` / `GazeboROS2ControlPlugin` are the LL equivalents.

**Reason (hardcoded path removed):** The absolute path `/home/kallestewart/...` was machine-specific and would silently fail on any other machine. The `ros2_controllers.yaml` is now passed in-memory via `MoveItConfigsBuilder.trajectory_execution()` in `gazebo_sim.launch.py`, which resolves the installed path from the ament index at runtime.

### 2.5 — `<gazebo reference="table_link">` material tag removed

`<material>Gazebo/Wood</material>` used the Classic Gazebo material system, which is not available in Gazebo Harmonic+. Removed to prevent a non-fatal but noisy parse warning on every launch.

---

## Phase 3 — C++ API & CMakeLists

**File:** `ur5_controller/src/multi_waypoint_planner.cpp`

### 3.1 — MoveIt include paths updated

| | Before | After |
|---|---|---|
| Header | `moveit/move_group_interface/move_group_interface.h` | `moveit/planning_interface/move_group_interface.hpp` |
| Header | `moveit/robot_state/conversions.h` | `moveit/robot_state/robot_state.hpp` |

**Reason:** MoveIt 2 for ROS 2 LL reorganized internal headers under `moveit/planning_interface/` and merged `conversions.h` into the main `robot_state.hpp`. The old `.h` paths are removed from the install.

### 3.2 — `computeCartesianPath` API updated

```cpp
// Before — jump_threshold as positional argument (removed in LL)
const double jump_threshold = 0.0;
const double eef_step = 0.01;
double fraction = move_group_->computeCartesianPath(
    waypoints_, eef_step, jump_threshold, calculated_trajectory_);

// After — CartesianInterpolator struct
moveit::planning_interface::MoveGroupInterface::CartesianInterpolator interp;
interp.max_step = 0.01;
double fraction = move_group_->computeCartesianPath(
    waypoints_, interp, calculated_trajectory_);
```

**Reason:** The `jump_threshold` parameter was removed from the `computeCartesianPath` signature in MoveIt 2 for ROS 2 LL. Path interpolation parameters are now encapsulated in the `CartesianInterpolator` struct.

**Note:** `append_parameter_override("use_sim_time", true)` in the constructor was already the correct LL API — no change required.

**File:** `ur5_controller/CMakeLists.txt`

### 3.3 — C++17 standard enforced

```cmake
# Added after project() declaration
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
```

**Reason:** MoveIt 2 on ROS 2 LL requires C++17. Without this, `ament_cmake` may fall back to C++14 ABI, causing link failures with MoveIt libraries.

### 3.4 — `ament_target_dependencies` replaced with `target_link_libraries`

```cmake
# Before
ament_target_dependencies(multi_waypoint_planner
    rclcpp moveit_msgs geometry_msgs moveit_ros_planning_interface)

# After
target_link_libraries(multi_waypoint_planner
    rclcpp::rclcpp
    ${moveit_ros_planning_interface_TARGETS}
    ${geometry_msgs_TARGETS}
    ${moveit_msgs_TARGETS}
)
```

**Reason:** `ament_target_dependencies` is a macro wrapper that sets include dirs and link flags non-transitively via CMake's legacy include-directory mechanism. Starting with ROS 2 LL, all ament packages export proper CMake targets (e.g., `rclcpp::rclcpp`). The `ament_target_dependencies` macro is heavily discouraged and will be removed in a future colcon release. Using `target_link_libraries` with explicit targets is transitive, correct, and faster to parse.

---

## Phase 4 — MoveIt 2 & Launch File Restructuring

### `ur5_moveit_config/config/ur5.srdf` — Candle Pose added

```xml
<group_state name="candle_pose" group="ur_manipulator">
    <joint name="shoulder_pan_joint1"  value="0.0"/>
    <joint name="shoulder_pan_joint"   value="0.0"/>
    <joint name="shoulder_lift_joint"  value="-1.5708"/>
    <joint name="elbow_joint"          value="0.0"/>
    <joint name="wrist_1_joint"        value="0.0"/>
    <joint name="wrist_2_joint"        value="0.0"/>
    <joint name="wrist_3_joint"        value="0.0"/>
</group_state>
```

**Reason:** Without a named safe start pose, the arm spawns at all-zeros, which points the upper arm horizontally into the table surface. The first PID tick drives joints into the collision mesh. `shoulder_lift_joint = -1.5708 rad` (−90°) points the upper arm straight up (candle pose), keeping all links clear of the table.

### `ur5_moveit_config/config/ros2_controllers.yaml` — `forward_effort_controller` config block added

```yaml
forward_effort_controller:
  ros__parameters:
    joints:
      ["shoulder_pan_joint1", "shoulder_pan_joint", "shoulder_lift_joint",
       "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
    interface_name: effort
```

**Reason:** The controller type was declared in the `controller_manager` section but had no `ros__parameters` block. `ForwardCommandController` requires `joints` and `interface_name` to load. Without this block, activating `forward_effort_controller` via the switch service returned a `LOAD_ERROR` and the controller stayed in the unconfigured state.

**Format note:** Inline bracket array format `["joint1", ...]` used instead of YAML block sequences to avoid the ros2_controllers YAML parser tokenization bug where multi-line block sequences are parsed as a single space-separated string literal.

### `ur5_moveit_config/launch/gazebo_sim.launch.py` — Full rewrite

**Before:** Multi-file `IncludeLaunchDescription` chain — 5 separate launch files included sequentially (`rsp.launch.py`, `move_group.launch.py`, `moveit_rviz.launch.py`). Robot name spawn arg was `ur57_arm`. `IGN_GAZEBO_RESOURCE_PATH` set alongside `GZ_SIM_RESOURCE_PATH`.

**After:** Single self-contained launch file. `MoveItConfigsBuilder("ur5_arm", ...)` serves as the single source of truth for robot description, SRDF, kinematics, joint limits, and controller config. All nodes receive their parameters in-memory via the builder's dict. `IGN_GAZEBO_RESOURCE_PATH` removed. Spawn arg corrected to `ur5_arm`.

**Reason:** The multi-file include chain had a race condition on slow VMs (each included file independently re-parsed the URDF from disk). The builder approach parses once and passes the same object to all nodes, eliminating the race and the disk-dependency. It is also the canonical LL/MoveIt 2 pattern.

### `ur5_controller/launch/planner.launch.py` — Typo fix

```python
# Before
MoveItConfigsBuilder("ur57_arm", package_name="ur5_moveit_config")

# After
MoveItConfigsBuilder("ur5_arm", package_name="ur5_moveit_config")
```

**Reason:** The `ur57_arm` typo caused the builder to look for a package named `ur5_moveit_config` with a robot named `ur57_arm`, which did not match the SRDF robot name `ur5_arm`, resulting in a `KeyError` at runtime.

### `ur5_description/launch/sim.launch.py` — Legacy `ign` command retired

```python
# Before
gazebo = ExecuteProcess(cmd=['ign', 'gazebo', '-r', 'empty.sdf'], output='screen')
load_joint_state_broadcaster = ExecuteProcess(
    cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', ...])

# After
gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
    launch_arguments={'gz_args': '-r empty.sdf'}.items())
load_joint_state_broadcaster = Node(
    package='controller_manager', executable='spawner', ...)
```

**Reason:** The `ign` binary is not installed in Ubuntu 26.04. The `ros2 control load_controller` CLI subcommand was removed in LL; `spawner` is the correct mechanism. Using `IncludeLaunchDescription` from `ros_gz_sim` is the canonical approach and handles environment variable propagation correctly.

---

## Pre-existing Bugs Fixed

These bugs were present in the Humble branch and are corrected as part of this migration.

| # | File | Bug | Fix |
|---|---|---|---|
| 1 | `ur5.urdf` | `<robot name="ur57_arm">` typo | Standardized to `ur5_arm` across all files |
| 2 | `ur5.urdf` | Hardcoded absolute path `/home/kallestewart/...` in `<gazebo>` plugin | Removed; path injected at launch time via `MoveItConfigsBuilder` |
| 3 | `ros2_controllers.yaml` | `forward_effort_controller` had no `joints`/`interface_name` config | Added complete `ros__parameters` block |
| 4 | `planner.launch.py` | `MoveItConfigsBuilder("ur57_arm")` typo | Corrected to `"ur5_arm"` |
| 5 | `sim.launch.py` | `ExecuteProcess(cmd=['ign', 'gazebo', ...])` — `ign` binary retired | Replaced with `ros_gz_sim` `IncludeLaunchDescription` |

---

## Verification Checklist

After merging this branch and rebuilding:

```bash
# Clean build
cd ~/ros2_ws && rm -rf build/ install/ log/
source /opt/ros/ll/setup.bash
colcon build --symlink-install
source install/setup.bash

# Middleware check
ros2 doctor --report | grep rmw
# Expected: rmw_zenoh_cpp

# Launch
ros2 launch ur5_moveit_config gazebo_sim.launch.py

# Controller check (in a second terminal)
ros2 control list_controllers
# Expected:
#   joint_state_broadcaster   [active]
#   ur_manipulator_controller [active]
#   forward_effort_controller [inactive]

# Verify robot name in TF
ros2 run tf2_tools view_frames
# Expected: base_link present under world frame

# Waypoint planner
ros2 launch ur5_controller planner.launch.py
# Issue 'plan' then 'execute' — expect 100% Cartesian fraction
```
