# UR5 Digital Twin — Operator Walkthrough

**Platform:** ROS 2 Jazzy + Gazebo Harmonic + Zenoh (`rmw_zenoh_cpp`)  
**Repo:** `~/ros2_ws/src/ur5_digital_twin`  
**Workspace:** `~/ros2_ws`

---

## 0. Why “nothing launched” is often a false alarm

After the repair, **default simulation is GUI + RViz** (`ur5 simgui` / `run_sim`):

- Gazebo GUI window opens
- MoveIt RViz opens

Use `ur5 sim` only when you want headless (no windows). Controllers list successfully either way.

| What you want | Command |
|---|---|
| Sim + Gazebo GUI + RViz (default / `run_sim`) | `ur5 simgui` or `run_sim` |
| Headless only (no windows) | `ur5 sim` |

Also required: **one healthy** Zenoh router (`rmw_zenohd`) before any ROS node. A **stale** router (process alive but unreachable) looks exactly like “nothing launched”: launch prints start messages, but `ur5ctl` / `ros2 topic list` stay empty.

If discovery looks dead:

```bash
ur5zenoh_restart
# then Ctrl+C the old launch and run ur5 simgui again
```

**Fixed 2026-09-22:** `gazebo_sim.launch.py` defaulted to FastDDS while `ur5env` used Zenoh — the CLI could not see the launched graph. Default is now `rmw_zenoh_cpp`.

**Restored 2026-09-24:** bare `gazebo_sim` / `run_sim` again defaults to GUI + RViz; headless is explicit via `ur5 sim`.


---

## 1. One-time setup (each new terminal)

**Do this first** — until the functions are loaded, `ur5simgui` will be `command not found`.

Type these lines yourself (do not paste a whole markdown block):

```bash
source ~/ros2_ws/src/ur5_digital_twin/scripts/ur5_env.sh
type ur5simgui
```

You should see `ur5simgui is a function`. Then:

```bash
ur5kill
ur5zenoh_restart
ur5simgui          # default: Gazebo GUI + RViz + Candle upright
```

### Alternative — no `source` required

```bash
~/ros2_ws/src/ur5_digital_twin/scripts/ur5 kill
~/ros2_ws/src/ur5_digital_twin/scripts/ur5 zenoh
~/ros2_ws/src/ur5_digital_twin/scripts/ur5 simgui
# second terminal:
~/ros2_ws/src/ur5_digital_twin/scripts/ur5 ctl
~/ros2_ws/src/ur5_digital_twin/scripts/ur5 states   # shoulder_lift ≈ -1.57
```

Optional PATH shortcut (add to `~/.bashrc`):

```bash
export PATH="$HOME/ros2_ws/src/ur5_digital_twin/scripts:$PATH"
# Prefer Zenoh + GUI + ogre (ARM64):
alias run_sim='source ~/ros2_ws/src/ur5_digital_twin/scripts/ur5_env.sh >/dev/null; ur5simgui'
# then: ur5 simgui   or   run_sim
```

---

## 2. Build (when you change code)

```bash
ur5build                  # whole workspace
ur5build ur5_controller   # one package
ur5clean                  # wipe build/install/log — then ur5build
```

---

## 3. Start the simulation

### A — Default operator path (Gazebo GUI + RViz)

```bash
ur5simgui
# or: run_sim   /   ur5 run_sim   /   scripts/run_sim
```

Underlying command:

```bash
ros2 launch ur5_moveit_config gazebo_sim.launch.py \
  gz_headless:=false start_rviz:=true gz_render_engine:=ogre \
  rmw_implementation:=rmw_zenoh_cpp
```

Arm should spawn in **Candle Pose** (upright): `shoulder_lift_joint ≈ -1.5708`.
Bring-up runs `hold_candle.py` after JTC activates so the effort controller does not leave the arm flat on the table.

Leave this terminal running. Open a **second** terminal for control commands (repeat §1 in that shell).

### B — Headless (no windows)

```bash
ur5sim
```

Underlying command:

```bash
ros2 launch ur5_moveit_config gazebo_sim.launch.py \
  gz_headless:=true start_rviz:=false rmw_implementation:=rmw_zenoh_cpp
```

### C — Render-engine override (ARM64)

On this ARM64 VM use `ogre` (not ogre2):

```bash
GZ_SIM_RENDER_ENGINE=ogre ur5simgui
```

### Prove it is up (second terminal)

```bash
ur5ctl
# expect:
#   joint_state_broadcaster   active
#   ur_manipulator_controller active
#   forward_effort_controller inactive

ur5states     # 7 joint names + positions; shoulder_lift ≈ -1.57 (Candle)
ur5hz         # /joint_states rate
ur5iface      # effort interfaces claimed by JTC
```

If the arm is not upright after motion or a disturbance:

```bash
ur5reset      # FollowJointTrajectory → Candle (preferred)
# or, if JTC failed to activate:
ur5candle     # ensure_controllers.py recovery + hold Candle
```

If `ur5ctl` hangs or is empty: Zenoh router missing, or RMW mismatch. Re-run `ur5env` and ensure only **one** `rmw_zenohd`.

---

## 4. A4 research workflow (one Gazebo session)

```
┌─────────────┐     ┌──────────────────┐     ┌─────────────────────┐     ┌──────────────────┐
│ 1. ur5simgui│ ──► │ 2. plan → CSV    │ ──► │ 3. ur5run <csv>     │ ──► │ 4. export+Candle │
│    (Candle) │     │    waypoints/    │     │    (JTC trajectory) │     │    (no gz restart)│
└─────────────┘     └──────────────────┘     └─────────────────────┘     └──────────────────┘
```

### Stage 1 — Sim already up (§3)

### Stage 2 — Plan and export CSV into `waypoints/`

**Today (R5):** trajectory *definition* still needs RViz **Publish Point**. There is no CLI goal-entry path yet.

```bash
# Terminal A: sim running (ur5sim or ur5simgui with RViz)
# Terminal B:
ros2 launch ur5_controller planner.launch.py   # if you use the planner package launch
# or run multi_waypoint_planner + planner_cli per your usual planner setup

ur5plan my_traj
# NOTICE: click ≥2 points in RViz, then in planner CLI:
#   plan
#   export joints
# Copies share/.../waypoints/trajectory_joints_export.csv → waypoints/my_traj.csv
```

Or use the shipped defaults (no planning):

```bash
ur5wp
# default_joint.csv / default_cartesian.csv already in waypoints/
```

### Stage 3 — Interactive / batch control

```bash
# Interactive prompts:
ur5run default_joint.csv

# Batch (no prompts) — used by automation:
python3 ~/ros2_ws/src/ur5_digital_twin/ur5_controller/src/interactive_control.py \
  --mode 1 --csv default_joint.csv --out run1
```

Results land in `./data/` relative to the **current working directory** (`*_filtered.csv` + `*_filtered.h5`).

### Stage 4 — Candle reset without restarting Gazebo

Automatic at the end of `interactive_control` / `ur5run`, or manually:

```bash
ur5reset
ur5states    # shoulder_lift should return near -1.5708
```

Gazebo PID must not change across cycles:

```bash
pgrep -a 'gz sim'
```

---

## 5. Dual-brain controllers (optional lab)

| Brain | Controller | Use |
|---|---|---|
| A | `ur_manipulator_controller` (JTC, effort) | Trajectories / A4 Stage 3 |
| B | `forward_effort_controller` (Float64MultiArray) | Torque experiments — **no active 100 Hz driver in tree** |

```bash
ur5brain b    # JTC → FEC (atomic)
ur5brain a    # FEC → JTC (atomic)
```

Illegal double-activation is refused by `controller_manager` (both claiming effort).

---

## 6. Diagnostics

```bash
ur5doctor     # B0+B3 dump → docs/repair_artifacts/doctor_<timestamp>.log
ur5ctl
ur5iface
ur5states
ur5hz /clock
ur5wp
```

Raw equivalents (always work without the shortcut file):

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
source ~/ros2_ws/install/setup.bash
ros2 run rmw_zenoh_cpp rmw_zenohd &    # once

ros2 launch ur5_moveit_config gazebo_sim.launch.py \
  gz_headless:=true start_rviz:=false rmw_implementation:=rmw_zenoh_cpp

ros2 control list_controllers -v
ros2 topic echo /joint_states --once
```

---

## 7. Shortcut cheat sheet

| Function | What it wraps |
|---|---|
| `ur5env` | ROS + workspace + RMW + zenohd |
| `ur5zenoh_restart` | Kill and restart `rmw_zenohd` (fix discovery) |
| `ur5kill` | Kill all Gazebo / bridges / MoveIt (keeps zenohd) |
| `ur5build [pkg]` | `colcon build --symlink-install` |
| `ur5clean` | wipe `build/ install/ log/` |
| `ur5sim [args]` | headless Gazebo + MoveIt stack |
| `ur5simgui [args]` | GUI Gazebo (`ogre` default) + RViz |
| `ur5candle` | Recovery: activate JTC + hold Candle |
| `ur5demo` | Presentation `demo_15s.csv` + Candle return |
| `ur5plan <name> [-f]` | plan/export helper (RViz still required) |
| `ur5run <csv>` | HIL from waypoint CSV |
| `ur5reset` | Candle Pose via JTC |
| `ur5ctl` / `ur5iface` | controllers / hardware interfaces |
| `ur5brain a\|b` | exclusive controller switch |
| `ur5states` / `ur5hz` | joint state / topic rate |
| `ur5wp` | list `waypoints/` |
| `ur5doctor` | health dump |

---

## 8. Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| No window after `ur5sim` | Headless by design | Use `ur5simgui`, or treat `ur5ctl` as proof of life |
| `ur5ctl` empty / hangs | No zenohd, or RMW mismatch | `ur5env`; `pgrep rmw_zenohd`; never mix FastDDS CLI with Zenoh launch |
| Spawners die: cannot contact `controller_manager` | Gazebo plugin / RMW / stale processes | Kill old `gz`/`spawner`; one zenohd; relaunch |
| ogre2 crash on GUI | ARM64 software GL | `GZ_SIM_RENDER_ENGINE=ogre ur5simgui` |
| Arm not vertical at start | Candle IC missing / sag | `ur5reset` or `ur5candle`; check `ur5states` (`shoulder_lift≈-1.57`) |
| Duplicate `/controller_manager` | Orphaned `gz sim` | `ur5kill` / purge all `gz sim`; one relaunch |

---

## Presentation demo (10–15 s arm motion)

### Option A — one command (recommended for talks)

```bash
source ~/.bashrc
~/ros2_ws/src/ur5_digital_twin/scripts/run_presentation_demo.sh
```

Opens Gazebo + RViz, waits for Candle Pose, runs `demo_15s.csv` (~12 s sweep), returns to Candle. Leave the launch running; Ctrl+C or `ur5 kill` when done.

### Option B — two terminals (more control)

```bash
# Terminal 1 — GUI
source ~/.bashrc
ur5 kill
ur5 zenoh
ur5 simgui          # Gazebo + RViz windows

# Terminal 2 — after ur5 ctl shows JTC active
source ~/.bashrc
ur5 demo            # ~12 s motion + Candle
```

Trajectory file: `ur5_controller/src/waypoints/demo_15s.csv`

---

## 9. Recommended daily session

```bash
# Terminal 1 — bring-up (leave running)
source ~/.bashrc
ur5 kill
ur5 zenoh
ur5 simgui              # GUI — or: ur5 sim for headless

# Terminal 2 — operate
source ~/.bashrc
ur5 ctl                 # must show JTC + JSB active
ur5 states              # Candle: shoulder_lift ≈ -1.57
ur5 demo                # presentation motion
# or: ur5 run default_joint.csv
```

Stop: `Ctrl+C` in Terminal 1, then `ur5 kill`.
