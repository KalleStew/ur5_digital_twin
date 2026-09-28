# Environment Reset & Troubleshooting

When working with Gazebo Harmonic, ROS 2 Jazzy, and Eclipse Zenoh, background processes can hang or orphan themselves. Prefer the `ur5` helpers in `scripts/ur5_env.sh` over ad-hoc `killall`.

---

## 1. Safe shutdown (try this first)

Click into the terminal where you ran the launch command and press `Ctrl + C`.

**CRITICAL:** Do not close Gazebo or RViz with the window `×`. That kills only the renderer and leaves `controller_manager`, `move_group`, and sometimes a stuck `gz sim` alive — the next launch then sees **duplicate** `/controller_manager` nodes and a collapsed arm.

Wait 3–5 seconds after `Ctrl + C` before relaunching.

---

## 2. Operator purge (`ur5 kill`)

```bash
source ~/ros2_ws/src/ur5_digital_twin/scripts/ur5_env.sh
ur5kill          # or: ur5 kill
# Leaves rmw_zenohd running. Restart router if discovery looks dead:
ur5zenoh_restart
```

`ur5kill` stops Gazebo (`gz sim` / ruby wrappers), bridges, RSP, move_group, and RViz. It also matches launch parents by process args.

If something still holds the world:

```bash
pkill -9 -f 'gz sim'
pkill -9 -f 'gazebo_sim.launch.py'
# verify:
pgrep -af 'gz sim' || echo clean
```

**Do not** `killall ruby` — that is not the Zenoh router on Jazzy (`rmw_zenohd` is). Killing all `ruby` can take down unrelated tools.

---

## 3. Candle upright without restarting Gazebo

If the arm left Candle Pose (demo, sag, disturbance):

```bash
ur5reset       # FollowJointTrajectory → Candle (shoulder_lift ≈ -1.5708)
ur5states      # confirm
```

If JTC is inactive / spawners raced:

```bash
ur5candle      # ensure_controllers.py: activate JSB+JTC, hold Candle
```

Spawn IC is already Candle via URDF `initial_value` + SRDF `candle_pose` — reset is for **recovery**, not first boot.

---

## 4. Nuke the colcon cache (last resort)

```bash
cd ~/ros2_ws
rm -rf build/ install/ log/
# or: ur5clean && ur5build
```

---

## 5. Rebuild and relaunch

```bash
ur5build
ur5zenoh_restart
ur5simgui          # GUI + RViz default
# or: ur5sim       # headless
```

---

## 6. Zenoh-specific issues

### Symptom: `ros2 topic list` empty / “nothing launched”

Usually a **stale** or missing `rmw_zenohd`, or RMW mismatch (shell Zenoh + launch FastDDS).

```bash
ur5zenoh_restart
# Ctrl+C old launch, then:
ur5simgui
```

Confirm launch default is `rmw_zenoh_cpp` (`gazebo_sim.launch.py`).

### Symptom: duplicate `/controller_manager`

Orphaned `gz sim` from a prior session. Run §2 purge until `pgrep -af 'gz sim'` is empty, then relaunch **one** sim.

---

## 7. ARM64 / render

| Symptom | Fix |
|---|---|
| ogre2 crash on GUI | `GZ_SIM_RENDER_ENGINE=ogre ur5simgui` (default in `ur5simgui`) |
| No window after `ur5sim` | Headless by design — use `ur5simgui` |
| `create` stuck “Requesting list of world names” | World never finished loading — `ur5kill`, ensure single gz, retry; headless is a reliable fallback |

---

## 8. Controllers

Expect after healthy bring-up:

```text
joint_state_broadcaster     active
ur_manipulator_controller   active
forward_effort_controller   inactive
```

Spawners live in `ur5_description/launch/sim.launch.py` (JSB → JTC → FEC inactive). `ensure_controllers.py` is **recovery-only** (`ur5 candle`), not on the launch critical path.
