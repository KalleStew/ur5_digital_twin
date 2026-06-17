# Environment Reset & Troubleshooting

When working with complex ROS 2 architectures, Gazebo physics, and hardware controllers, background processes can occasionally hang or crash (leaving "zombie" nodes). This document outlines the exact sequence to completely reset and cleanly relaunch your simulation environment.

---

## 1. Safely Close or Force Kill Background Processes

If the simulation is currently running, try to safely close it first by clicking into the terminal where you ran the launch command and pressing `Ctrl + C`. 

**CRITICAL:** Never click the "X" on the Gazebo or RViz visualizer windows to close them. This immediately kills the visual interface but leaves invisible controller managers and physics engines running in the background, causing port conflicts on your next launch.

### The "Total Purge" Command
If the system is completely frozen or you are dealing with "zombie" nodes that didn't shut down properly, you must execute a total purge of the ROS 2 daemon and network memory. Run these commands in any terminal:

```bash
killall -9 ruby rviz2 gz ign spawner move_group ros2_control_node robot_state_publisher static_transform_publisher
ros2 daemon stop
ros2 daemon start
```
*(Note: If the terminal says "no process found" for some of these, that is perfectly fine and just means they were successfully shut down previously.)*

---

## 2. Nuke the Corrupted Cache

If a core component like a spawner died or threw a segmentation fault, your ROS 2 build cache may be stuck in a corrupted state. You need to wipe it clean so the system can read your fresh configuration files.

```bash
cd ~/ros2_ws
rm -rf build/ install/ log/
```

---

## 3. Rebuild and Relaunch

Now that your memory is purged and the cache is cleared, rebuild the workspace and do a clean boot of the digital twin:

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

# Launch the master simulation environment
ros2 launch ur5_moveit_config gazebo_sim.launch.py
```