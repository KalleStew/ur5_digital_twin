# Operation Guide

Run each stage in its own terminal, or use the `ur5` shortcut, which sources Jazzy and starts `rmw_zenohd` if it is not already running. One Gazebo session at a time. A state reset does not start a second one.

## 1. Launching the Master Simulation Environment (Terminal 1)

Plain command, GUI:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 launch ur5_moveit_config gazebo_sim.launch.py
```

Headless:

```bash
ros2 launch ur5_moveit_config gazebo_sim.launch.py gz_headless:=true start_rviz:=false
```

Shortcuts: `ur5 simgui` and `ur5 sim`. `ur5 run_sim` is `ur5 simgui`.

Expected: Gazebo starts, then `joint_state_broadcaster`, `ur_manipulator_controller`, and `forward_effort_controller` (inactive). `move_group` follows. The log ends with a line like:

```text
OK: Candle reached: max|dq|=0.0008 rad, RMS qd=3.6e-06 rad/s
```

`ur5 ctl` lists `joint_state_broadcaster` and `ur_manipulator_controller` as active, and `forward_effort_controller` as inactive. The arm is upright. `candle_pose` is all zeros unless you edited the SRDF.

To skip the pre-flight: `run_preflight:=false`. If `ur5_controller` is not in the workspace, bring-up still starts and prints `ERROR: ur5_controller not found; Candle pre-flight was not started`.

## 2. Running the C++ Waypoint Planner (Terminal 2)

```bash
ros2 launch ur5_controller planner.launch.py
ros2 run ur5_controller planner_cli.py
```

Shortcuts: `ur5 planner`, and `ur5 plan <name> [--points file.points.yaml]`.

RViz Publish Point needs `ur5 simgui`. The export lands in `$UR5_DATA_DIR/waypoints/<name>.csv`. Shipped examples are `ur5_controller/src/waypoints/example.points.yaml` and the CSVs beside it. Each shipped joint file starts and ends at Candle.

## 3. Run and experiment (Terminal 3)

Stage 3a, Brain A:

```bash
ros2 run ur5_controller interactive_control.py --scheme position --csv demo_15s.csv
# shortcut:
ur5 run demo_15s.csv
ur5 demo
```

Expected: a pre-flight line, motion, an HDF5 path under `$UR5_DATA_DIR/telemetry/`, then `reset: Candle reached: ...`.

Stage 3b, Brain B, is the handoff. The trial switches to `forward_effort_controller`, runs one HDF5 per trial, and returns to Candle under Brain A.

```bash
ros2 run ur5_controller interactive_control.py --scheme torque --experiment experiment_nominal_check.yaml
# shortcut:
ur5 torque experiment_nominal_check.yaml
```

`experiment_slice.yaml` is the short unattended slice (eight trials). `experiment_benchmark.yaml` is the full matrix and needs `learned_id.npz` if that law is left in the file.

With no flags, `interactive_control.py` prints:

```text
UR5 digital twin
  [1] Run waypoints (Brain A)
  [2] Run experiment (Brain B)
  [3] Return to Candle
  [4] Switch brain
  [5] Validate data
  [6] Quit
```

`ur5 brain b` holds the current pose with PD + gravity. `ur5 brain a` gives authority back to the trajectory controller. Brain B is never left active without a torque command.

## 4. Reset and export (Terminal 4)

```bash
ros2 run ur5_controller interactive_control.py --candle
ur5 candle
ur5 reset
ros2 run ur5_controller telemetry.py validate "$UR5_DATA_DIR/telemetry"
ur5 validate "$UR5_DATA_DIR/telemetry"
```

`ur5 validate` prints `PASS:` or `FAIL:` per file. A torque file whose Nyquist ratio exceeds 8.5 fails. Exit 0 means every file passed.

## Expected Outputs & Validation

After stage 1, `ur5 ctl` shows the three controllers above. Candle output quotes `max|dq|` under 0.005 rad and RMS velocity under 0.001 rad/s (`platform.yaml` `candle:`).

A Brain A run prints the HDF5 path and a successful reset. A nominal check prints one `rate=100.0Hz rms_err=...` line per trial and `OK: 6 trials`. Computed torque and PD + inverse dynamics both track more tightly than PD + gravity on `bench_reach` and `bench_sweep`.

`ur5 validate` on that directory prints `PASS:` for each kept file. Rejected retries are under `$UR5_DATA_DIR/rejected/` and are not required to pass.

## Changing the starting pose

`candle_pose` in `ur5_moveit_config/config/ur57_arm.srdf` is the only pose to edit.

1. Edit `candle_pose`.
2. `ur5 build`.
3. `ur5 test --unit`. The clearance test fails with the link name if a collision mesh meets the workbench.
4. `ur5 kill`, then `ur5 sim`. This is a new robot configuration, not a reset.

Spawn and `ur5 candle` both use the new numbers. No other file stores them.

## Data and HDF5 layout

`$UR5_DATA_DIR` defaults to `ur5_controller/src/data`. Set the variable to put data somewhere else. `ur5 clean` does not delete it.

Each file is raw N×7: `t`, `q`, `qd`, `tau_measured`, `tau_commanded`, `q_ref`, `qd_ref`, plus metadata (`scheme`, `controller`, `gains`, `perturbation`, `trajectory_sha256`, `joint_names`, `result`, and the artefact fields on torque trials).

Exit codes from `interactive_control.py` and `ur5`: 0 ok, 1 run failed, 2 input rejected, 3 platform not ready, 4 safety abort, 130 interrupted.
