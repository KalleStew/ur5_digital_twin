# Operation Guide

Standard procedures for launching the UR5 Digital Twin, planning, and Candle reset.

**Platform:** ROS 2 Jazzy + Gazebo Harmonic + Zenoh (`rmw_zenoh_cpp`)  
**Preferred helpers:** `scripts/ur5` / `scripts/ur5_env.sh` (see `OPERATOR_WALKTHROUGH.md`)

**Prerequisite — every new terminal:**

```bash
source ~/ros2_ws/src/ur5_digital_twin/scripts/ur5_env.sh
# or: source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
#     export RMW_IMPLEMENTATION=rmw_zenoh_cpp
```

Ensure **one** healthy Zenoh router (`ur5 zenoh` / `ur5zenoh_restart`) before launching nodes.

---

## 1. Launch simulation (Terminal 1)

### Default — Gazebo GUI + RViz (Candle upright at spawn)

```bash
ur5 simgui
# equivalents: ur5simgui | scripts/run_sim | ur5 run_sim
```

Bare launch (same defaults as of 2026-09-24 restore):

```bash
ros2 launch ur5_moveit_config gazebo_sim.launch.py
# defaults: gz_headless:=false  start_rviz:=true  rmw_implementation:=rmw_zenoh_cpp
```

On this ARM64 VM, force `ogre` (ogre2 often fails under software GL):

```bash
GZ_SIM_RENDER_ENGINE=ogre ur5 simgui
```

### Headless (no windows)

```bash
ur5 sim
```

**What this does:**
* Starts Gazebo Harmonic with the UR5 URDF (Candle IC: `shoulder_lift_joint = -1.5708`)
* Spawns controllers via `sim.launch.py`: JSB → JTC active, FEC inactive
* Launches MoveIt `move_group` + RViz (unless headless)
* Bridges `/clock` (`use_sim_time:=true`)

**Proof of life (Terminal 2):**

```bash
ur5 ctl      # JSB+JTC active, FEC inactive
ur5 states   # shoulder_lift ≈ -1.57
```

---

## 2. Reposition to Candle (upright) without restarting Gazebo

After a demo, disturbance, or sag:

```bash
ur5 reset       # FollowJointTrajectory → Candle Pose
# if JTC is inactive:
ur5 candle      # ensure_controllers.py recovery + hold Candle
```

Do **not** restart Gazebo just to stand the arm up.

---

## 3. C++ waypoint planner (Terminal 2)

```bash
ros2 launch ur5_controller planner.launch.py
```

If stdin is not forwarded, use:

```bash
python3 ~/ros2_ws/src/ur5_digital_twin/ur5_controller/src/planner_cli.py
```

`planner_cli.py` publishes to `/planner_command`. Waypoint **definition** still uses RViz Publish Point (`/clicked_point`) today.

---

## 4. Interactive / batch trajectory (Terminal 3)

```bash
ur5 run demo_15s.csv          # or any waypoints/*.csv
ur5 demo                      # presentation sweep + Candle return
```

Optional Python deps for filtered HDF5 export:

```bash
sudo apt install -y python3-numpy python3-scipy python3-h5py
```

All terminals in one run must share `RMW_IMPLEMENTATION=rmw_zenoh_cpp`.

---

## 5. Dual-brain note

- **Brain A (default):** `ur_manipulator_controller` (JTC, effort-commanded trajectory)
- **Brain B (loaded inactive):** `forward_effort_controller` — activate only via intentional `switch_controllers` for torque research

See `architecture.md` and `OPERATOR_WALKTHROUGH.md` for the A4 research loop.
