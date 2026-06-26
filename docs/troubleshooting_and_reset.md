# Environment Reset & Troubleshooting

When working with complex ROS 2 architectures, Gazebo physics, and hardware controllers, background processes can occasionally hang or crash, leaving zombie nodes. This document provides the exact sequence to completely reset and cleanly relaunch the simulation environment under ROS 2 Lyrical Luthien.

---

## 1. Safe Shutdown

If the simulation is running, press `Ctrl+C` in the terminal where the launch command was run.

**Never** click the X button on the Gazebo or RViz windows. This kills the GUI but leaves the `controller_manager`, `move_group`, and physics engine running invisibly — causing port conflicts on the next launch.

---

## 2. Total Purge of Zombie Processes

If the system is frozen or zombie nodes exist, execute the full kill sequence:

```bash
# Kill all known zombie ROS/Gazebo processes (errors about "no process found" are harmless)
killall -9 ruby rviz2 gz ign spawner move_group \
  ros2_control_node robot_state_publisher static_transform_publisher 2>/dev/null

# Stop and restart the ROS 2 daemon (clears node graph and DDS memory)
ros2 daemon stop && ros2 daemon start
```

**For Eclipse Zenoh:** If using `rmw_zenohd` as a router, kill it too:
```bash
killall -9 rmw_zenohd 2>/dev/null
```

---

## 3. Nuke Corrupted Build Cache

If a spawner threw a segfault or a `colcon build` failed mid-way, the build cache may be stuck in a partially-written state. Wipe it:

```bash
cd ~/ros2_ws
rm -rf build/ install/ log/
```

---

## 4. Rebuild and Relaunch

```bash
cd ~/ros2_ws
source /opt/ros/ll/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash

# Launch the master simulation stack
ros2 launch ur5_moveit_config gazebo_sim.launch.py
```

---

## 5. Common Issues

### Controllers fail to load (timeout)

All spawners use `--controller-manager-timeout 30`. On slow VMs or WSL2 with low RAM allocation, the `controller_manager` node may not be ready within 30 seconds.

**Fix:** Increase timeout in `gazebo_sim.launch.py`:
```python
arguments=["joint_state_broadcaster", "--controller-manager-timeout", "60"]
```
Or increase WSL2 memory allocation in `.wslconfig`.

### `forward_effort_controller` stays in `unconfigured` state

This means the `ros__parameters` block in `ros2_controllers.yaml` is missing or malformed.

**Verify:**
```bash
ros2 control list_controllers
ros2 param get /controller_manager forward_effort_controller.joints
```

The `joints` parameter must list all 7 joint names. See `ur5_moveit_config/config/ros2_controllers.yaml`.

### `move_group` exits immediately after launch

Usually caused by a robot name mismatch between the URDF (`<robot name>`) and the SRDF (`<robot name>`).

**Verify:**
```bash
ros2 topic echo /robot_description --once | grep 'name='
```
Both URDF and SRDF must show `ur5_arm`.

### Arm collapses through the table on spawn

This is caused by the Gazebo internal position PID defaulting to `p=0.1` because the `<param name="p">` tags inside `<ros2_control>` are missing or misplaced.

**Correct structure in `ur5.urdf`:**
```xml
<joint name="shoulder_lift_joint">
    <param name="p">30000.0</param>   <!-- MUST be a direct child of <joint> -->
    <param name="d">1000.0</param>
    <command_interface name="position"/>
    <command_interface name="effort"/>
    ...
</joint>
```

### Zenoh nodes not discovering each other

On WSL2, the virtual network adapter uses NAT. Zenoh peer-to-peer discovery may fail if nodes are on different WSL2 instances.

**Fix:** Start the Zenoh router before launching ROS nodes:
```bash
ros2 run rmw_zenoh_cpp rmw_zenohd &
sleep 2
ros2 launch ur5_moveit_config gazebo_sim.launch.py
```

### `gz` command not found

The `ign` binary is fully retired in Ubuntu 26.04. If scripts or launch files reference `ign`, update them to use the `ros_gz_sim` `IncludeLaunchDescription` pattern. See [docs/migration_UB22_to_UB26.md](migration_UB22_to_UB26.md).

---

## 6. One-Line Nuke Alias

Add to `~/.bashrc` for convenience:

```bash
alias nuke='cd ~/ros2_ws && \
  killall -9 ruby rviz2 gz spawner move_group ros2_control_node robot_state_publisher static_transform_publisher 2>/dev/null ; \
  ros2 daemon stop && ros2 daemon start && \
  rm -rf build/ install/ log/ && \
  source /opt/ros/ll/setup.bash && \
  colcon build --symlink-install && \
  source install/setup.bash && \
  echo "Environment reset complete."'
```
