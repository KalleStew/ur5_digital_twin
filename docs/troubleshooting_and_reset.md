# Environment Reset & Troubleshooting

When working with Gazebo Harmonic, ROS 2 Jazzy, and Eclipse Zenoh, background processes can occasionally hang or orphan themselves. This document provides the exact reset procedure for a clean environment recovery.

---

## 1. Safe Shutdown (Try This First)

Click into the terminal where you ran the launch command and press `Ctrl + C`.

**CRITICAL:** Never click the `×` on the Gazebo or RViz windows to close them. This kills only the renderer but leaves the `ros2_control_node`, `move_group`, and Zenoh router alive in the background, causing port conflicts on the next launch.

Wait 3–5 seconds after `Ctrl + C` for all child processes to terminate before proceeding.

---

## 2. The Total Purge Command

If the system is frozen or you have zombie nodes that did not shut down cleanly, execute a total purge of all simulation and ROS 2 processes:

```bash
killall -9 ruby rviz2 gz spawner move_group ros2_control_node \
  robot_state_publisher static_transform_publisher
ros2 daemon stop
ros2 daemon start
```

> **Note:** `ruby` is the Zenoh router process. `gz` is the Gazebo Harmonic server. If any of these are not running, `killall` will print `no process found` — this is expected and harmless.

**Then verify the ROS 2 node list is clean:**
```bash
ros2 node list
# Expected: empty output (no nodes running)
```

---

## 3. Nuke the Corrupted Colcon Cache

If a controller spawner died mid-build, or a `colcon build` was interrupted, the `build/`, `install/`, and `log/` directories may contain stale artifacts that cause `package not found` or library linking errors.

```bash
cd ~/ros2_ws
rm -rf build/ install/ log/
```

This is always safe — colcon will regenerate everything from source on the next build.

---

## 4. Rebuild and Relaunch (Clean Boot)

After purging processes and clearing the cache:

```bash
cd ~/ros2_ws

# Rebuild all three packages from scratch
colcon build --symlink-install

# Source the fresh install overlay
source install/setup.bash

# Launch the master simulation
ros2 launch ur5_moveit_config gazebo_sim.launch.py
```

---

## 5. Zenoh-Specific Issues

### Symptom: `ros2 topic list` returns nothing / nodes can't see each other

Zenoh's router may have crashed. Reset the daemon:

```bash
ros2 daemon stop && ros2 daemon start
# Wait 2 seconds, then verify
ros2 topic list
```

### Symptom: `rmw_zenoh_cpp: failed to open session`

The Zenoh router (`zenohd`) may have a port conflict. Kill any stale router:

```bash
killall -9 ruby
ros2 daemon stop && ros2 daemon start
```

If your router is managed externally, launch the stack with the embedded router disabled:
```bash
ros2 launch ur5_moveit_config gazebo_sim.launch.py start_zenoh_router:=false
```

### Symptom: Reverting to FastDDS temporarily (debugging only)

```bash
# Override for a single terminal session — does NOT affect other terminals
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch ur5_moveit_config gazebo_sim.launch.py
```

---

## 6. Gazebo Harmonic — Common Issues

### Symptom: Arm collapses through the table on spawn

**Cause:** The `gz_ros2_control/GazeboSimSystem` physics gains defaulted to `p=0.1` because the `<param name="p">` blocks in the URDF were not parsed correctly.

**Fix:** Verify the URDF `<ros2_control>` block. Each `<joint>` must have `<param name="p">` tags as **direct children of the `<joint>` element**, not inside a `<command_interface>` block:

```xml
<!-- CORRECT — gains are siblings of command_interface -->
<joint name="shoulder_lift_joint">
    <param name="p">30000.0</param>
    <param name="i">200.0</param>
    <param name="d">1000.0</param>
    <command_interface name="position"/>
    <command_interface name="effort"/>
    ...
</joint>

<!-- WRONG — gains buried inside command_interface are ignored -->
<joint name="shoulder_lift_joint">
    <command_interface name="position">
        <param name="p">30000.0</param>  <!-- This does nothing -->
    </command_interface>
</joint>
```

### Symptom: `[gz_ros2_control]: Could not load hardware plugin gz_ros2_control/GazeboSimSystem`

```bash
# Verify the package is installed
ros2 pkg list | grep gz_ros2_control

# If missing:
sudo apt install -y ros-jazzy-gz-ros2-control
```

### Symptom: RViz opens but shows no robot / all-grey scene

The `robot_state_publisher` may have started before the URDF was available. Stop everything, run the total purge, then relaunch.

---

## 7. Controller Manager Issues

### Symptom: Spawner times out (`Waiting for service /controller_manager/...`)

The `gz_ros2_control` world plugin (declared in the URDF's `<gazebo>` block) has not yet registered the controller manager. This is common on slow VMs. The launch file already uses `--controller-manager-timeout 30`. If 30 seconds is not enough:

```bash
# Manually activate a controller after the fact
ros2 control load_controller --set-state active joint_state_broadcaster
ros2 control load_controller --set-state active ur_manipulator_controller
```

### Symptom: `forward_effort_controller` not found

Verify its configuration block exists in `ros2_controllers.yaml`:

```yaml
forward_effort_controller:
  ros__parameters:
    joints: ["shoulder_pan_joint1", "shoulder_pan_joint", "shoulder_lift_joint",
             "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
    interface_name: effort
```

Then verify it was spawned `--inactive`:
```bash
ros2 control list_controllers
# Expected: forward_effort_controller  [inactive]
```

---

## 8. WSL2 / ARM64 VM Specific Issues

### Symptom: Gazebo crashes immediately on WSL2 with OpenGL error

```bash
# Force Mesa software renderer for this session
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch ur5_moveit_config gazebo_sim.launch.py
```

For a permanent fix, update your Windows GPU driver to a WSL2-certified version.

### Symptom: `gz sim` command not found after install

```bash
source /opt/ros/jazzy/setup.bash
which gz
# If still missing:
sudo apt install -y ros-jazzy-ros-gz-sim
```