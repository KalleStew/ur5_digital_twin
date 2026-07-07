# Migration Changelog: Ubuntu 22.04 / ROS 2 Humble → Ubuntu 24.04 / ROS 2 Jazzy

**Branch:** `feature/ubuntu24-jazzy-migration`  
**Migration Date:** 2026-07-07  
**Maintainer:** Kalle Stewart — ICE Lab

This document is a precise technical record of every structural and API change applied to the codebase during the migration from the Humble stack to the Jazzy stack. It serves as a reference for debugging regressions and for any future migration.

---

## 1. Middleware: FastDDS → Eclipse Zenoh

### Change

Removed all reliance on the default `rmw_fastrtps_cpp` middleware. Configured `rmw_zenoh_cpp` as the mandatory default via `~/.bashrc`.

```bash
# Added to ~/.bashrc
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
```

### Rationale

FastDDS uses UDP multicast for participant discovery. In WSL2 (Hyper-V virtual switch) and ARM64 VMs (VirtIO bridge), multicast packets are filtered by the hypervisor, causing intermittent node discovery failures and dropped messages. This is catastrophic for the 100 Hz torque array pipeline of the HIL system.

Eclipse Zenoh replaces the entire DDS transport layer with a brokerless peer-to-peer protocol that:

- Uses TCP as the primary transport (passes through NAT and hypervisor networks reliably)
- Supports zero-copy shared-memory paths for intra-host communication
- Reduces latency for `Float64MultiArray` at 100 Hz by eliminating the DDS serialization heap pressure
- Is Tier 1 supported in ROS 2 Jazzy (not an experimental plugin)

### Files Modified

- `docs/environment_setup.md` — Installation instructions and `~/.bashrc` configuration
- `README.md` — Middleware table updated

---

## 2. Simulation Engine: Ignition Fortress/Garden → Gazebo Harmonic

### Change

All references to the legacy `ignition-*` / `ign_ros2_control` ecosystem were replaced with the `gz-*` / `gz_ros2_control` equivalents. Gazebo Harmonic (v8) is the Tier 1 simulator for ROS 2 Jazzy.

| Old (Humble) | New (Jazzy) |
|---|---|
| `ign_ros2_control` (package dep) | `gz_ros2_control` |
| `ign_ros2_control/IgnitionSystem` (URDF plugin) | `gz_ros2_control/GazeboSimSystem` |
| `IGN_GAZEBO_RESOURCE_PATH` (env var) | `GZ_SIM_RESOURCE_PATH` |
| `ign gazebo -r empty.sdf` (CLI) | `gz sim -r empty.sdf` (via `ros_gz_sim`) |
| `ros_gz_sim` IncludeLaunch (already correct) | unchanged |

### Files Modified

- `ur5_description/package.xml` — `ign_ros2_control` → `gz_ros2_control`
- `ur5_moveit_config/package.xml` — removed `gazebo_ros_control`, added `gz_ros2_control` and `ros_gz_sim`
- `ur5_description/urdf/ur5.urdf` — hardware plugin renamed; `<gazebo>` world plugin tag added; `<ros2_control name>` attribute renamed from `IgnitionSystem` to `GazeboSystem`
- `ur5_description/launch/sim.launch.py` — replaced `ExecuteProcess(cmd=['ign', 'gazebo', ...])` with `IncludeLaunchDescription` of `ros_gz_sim/gz_sim.launch.py`
- `ur5_moveit_config/launch/gazebo_sim.launch.py` — removed stale `IGN_GAZEBO_RESOURCE_PATH` env action; updated to Gazebo Harmonic launch

---

## 3. URDF: URDF 1.2 Physics Safety Constraints

### Change: Acceleration and Jerk Limits

Added `acceleration` and `jerk` attributes to all seven revolute joint `<limit>` tags. These are URDF 1.2 extensions parsed by `gz_ros2_control` and MoveIt 2 Jazzy to constrain the physics solver.

```xml
<!-- Before (Humble) -->
<limit effort="150.0" velocity="3.14159" lower="-6.28318" upper="6.28318"/>

<!-- After (Jazzy) -->
<limit effort="150.0" velocity="3.14159" acceleration="5.0" jerk="10.0" lower="-6.28318" upper="6.28318"/>
```

Without explicit acceleration limits, the Gazebo position solver can request instantaneous velocity steps, which at high P-gains (`p=30000`) produces force spikes that violate physics constraints and cause the arm to explode out of the scene on startup.

### Change: Gazebo World Plugin Tag Added

A `<gazebo>` world plugin element was added to the URDF. This instructs Gazebo Harmonic to load the `gz_ros2_control::GazeboSimROS2ControlPlugin`, which bootstraps the `/controller_manager` node inside the simulation context.

```xml
<gazebo>
    <plugin filename="gz_ros2_control-system"
            name="gz_ros2_control::GazeboSimROS2ControlPlugin">
        <ros>
            <remapping>~/robot_description:=/robot_description</remapping>
        </ros>
    </plugin>
</gazebo>
```

**Critical Physics Gain Placement (unchanged — documented here for reference):**

The Gazebo physics gains (`<param name="p">`) **must** remain as direct children of each `<joint>` element, as siblings of `<command_interface>`. If placed inside a `<command_interface>` block, `gz_ros2_control` ignores them and the internal PID defaults to `p=0.1`, causing the arm to sag and collide with the table immediately.

```xml
<!-- CORRECT -->
<joint name="shoulder_lift_joint">
    <param name="p">30000.0</param>   ← sibling of command_interface
    <param name="i">200.0</param>
    <param name="d">1000.0</param>
    <command_interface name="position"/>
    ...
</joint>
```

### Files Modified

- `ur5_description/urdf/ur5.urdf`

---

## 4. C++ API: MoveIt 2 Jazzy Header Changes

### Change: `.h` → `.hpp` Headers

MoveIt 2 for ROS 2 Jazzy renamed all public headers from `.h` to `.hpp` to comply with the modern C++ convention adopted across the ROS 2 ecosystem.

```cpp
// Before (Humble)
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/conversions.h>

// After (Jazzy)
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_state/conversions.hpp>
```

Including the old `.h` paths in Jazzy produces `fatal error: file not found` at compile time.

### Files Modified

- `ur5_controller/src/multi_waypoint_planner.cpp`

---

## 5. Build System: Modern CMake in `ur5_controller`

### Change: Replaced `ament_target_dependencies` with `target_link_libraries`

The `ament_target_dependencies()` macro is a convenience wrapper that works but violates modern CMake target-based design. It pollutes the global include-path list rather than attaching includes transitively to the target. In complex workspaces this causes spurious header resolution and build-ordering issues that are hard to debug.

The new `CMakeLists.txt` uses proper imported targets and package variables:

```cmake
# Before (Humble — discouraged macro)
ament_target_dependencies(multi_waypoint_planner
  rclcpp moveit_msgs geometry_msgs moveit_ros_planning_interface)

# After (Jazzy — modern CMake)
target_link_libraries(multi_waypoint_planner
  rclcpp::rclcpp
  ${moveit_ros_planning_interface_LIBRARIES}
  ${geometry_msgs_TARGETS}
  ${moveit_msgs_TARGETS}
)
target_include_directories(multi_waypoint_planner PUBLIC
  ${moveit_ros_planning_interface_INCLUDE_DIRS}
)
# ament_target_dependencies retained only for ament_index environment hooks
ament_target_dependencies(multi_waypoint_planner
  moveit_ros_planning_interface geometry_msgs moveit_msgs)
```

CMake minimum version bumped from `3.8` to `3.22` to match Ubuntu 24.04's bundled CMake.

### Files Modified

- `ur5_controller/CMakeLists.txt`

---

## 6. MoveIt 2: `MoveItConfigsBuilder` Master Launch File

### Change: Multi-file `IncludeLaunchDescription` chain → single `MoveItConfigsBuilder` launch

The old `gazebo_sim.launch.py` orchestrated three separate `IncludeLaunchDescription` calls:
- `rsp.launch.py` — robot state publisher
- `move_group.launch.py` — MoveIt planning server
- `moveit_rviz.launch.py` — RViz

Each of those files independently instantiated its own `MoveItConfigsBuilder`, meaning the URDF was parsed three times and the parameter dicts were built three times. This created opportunities for inconsistencies if config files changed between calls.

The new `gazebo_sim.launch.py` builds the config **once**:

```python
moveit_config = (
    MoveItConfigsBuilder("ur5_arm", package_name="ur5_moveit_config")
    .robot_description(file_path="urdf/ur5.urdf")
    .robot_description_semantic(file_path="config/ur5.srdf")
    .robot_description_kinematics(file_path="config/kinematics.yaml")
    .joint_limits(file_path="config/joint_limits.yaml")
    .trajectory_execution(file_path="config/ros2_controllers.yaml")
    .to_moveit_configs()
)
```

The resulting `moveit_config` object is passed directly to each node's `parameters=[]` list, ensuring all nodes share the identical configuration dict from a single parse.

### Change: `initial_positions.yaml` Removed — Candle Pose Baked into SRDF

The `initial_positions.yaml` file (used in Humble to seed the arm starting pose) is incompatible with the Jazzy `MoveItConfigsBuilder` loading chain. Instead, the safe startup pose is now declared directly in `ur5.srdf` as a `<group_state>`:

```xml
<group_state name="candle_pose" group="ur_manipulator">
    <joint name="shoulder_pan_joint1"  value="0"/>
    <joint name="shoulder_pan_joint"   value="0"/>
    <joint name="shoulder_lift_joint"  value="-1.5708"/>  <!-- vertical, clears table -->
    <joint name="elbow_joint"          value="0"/>
    <joint name="wrist_1_joint"        value="0"/>
    <joint name="wrist_2_joint"        value="0"/>
    <joint name="wrist_3_joint"        value="0"/>
</group_state>
```

`shoulder_lift_joint = -1.5708 rad (-90°)` points the arm straight up, clearing the 0.99 m table entirely and preventing startup collisions.

### Files Modified

- `ur5_moveit_config/launch/gazebo_sim.launch.py` — complete rewrite
- `ur5_moveit_config/config/ur5.srdf` — added `candle_pose` group state

---

## 7. Package Dependency Changes

### `ur5_description/package.xml`

| Before | After | Reason |
|---|---|---|
| `ign_ros2_control` | `gz_ros2_control` | Gazebo Harmonic rename |

### `ur5_moveit_config/package.xml`

| Before | After | Reason |
|---|---|---|
| `gazebo_ros_control` | `ros_gz_sim`, `gz_ros2_control` | Replaced by Gazebo Harmonic packages |
| `warehouse_ros_mongo` | `warehouse_ros_sqlite` | MongoDB not packaged for Ubuntu 24.04; SQLite backend is default in Jazzy |

### `ur5_controller/package.xml`

| Added | Reason |
|---|---|
| `rclpy` | Explicit dep for Python HIL scripts |
| `std_msgs` | Used by torque injection scripts |
| `controller_manager_msgs` | Required for `SwitchController` service calls |

---

## 8. Summary of Files Changed

| File | Change Type | Summary |
|---|---|---|
| `ur5_description/package.xml` | Edit | `ign_ros2_control` → `gz_ros2_control` |
| `ur5_description/CMakeLists.txt` | No change | Already minimal and correct |
| `ur5_description/urdf/ur5.urdf` | Edit | gz plugin, URDF 1.2 limits, world plugin tag |
| `ur5_description/launch/sim.launch.py` | Edit | `ign gazebo` → Gazebo Harmonic `IncludeLaunchDescription` |
| `ur5_moveit_config/package.xml` | Edit | Remove Gazebo Classic deps, add Harmonic deps |
| `ur5_moveit_config/config/ur5.srdf` | Edit | Added `candle_pose` group state |
| `ur5_moveit_config/launch/gazebo_sim.launch.py` | Full rewrite | Single `MoveItConfigsBuilder` master launch |
| `ur5_controller/package.xml` | Edit | Added `rclpy`, `std_msgs`, `controller_manager_msgs` |
| `ur5_controller/CMakeLists.txt` | Full rewrite | Modern CMake, CMake 3.22, `target_link_libraries` |
| `ur5_controller/src/multi_waypoint_planner.cpp` | Edit | `.h` → `.hpp` MoveIt headers |
| `README.md` | Full rewrite | Jazzy stack, dual-brain architecture, Zenoh, quick start |
| `docs/environment_setup.md` | Full rewrite | Ubuntu 24.04, WSL2, ARM64, Zenoh, Gazebo Harmonic |
| `docs/development_workflow.md` | Full rewrite | Threaded Python pattern, controller switching, HDF5 |
| `docs/troubleshooting_and_reset.md` | Full rewrite | Zenoh reset, Gazebo Harmonic issues, physics gain fix |
| `docs/migration_UB22_to_UB24.md` | New file | This document |
