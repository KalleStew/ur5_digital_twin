# Environment Reset & Troubleshooting

## 1. Safely Close or Force Kill Background Processes

Stop the session from a terminal that can see it:

```bash
ur5 kill
```

That sends `kill -9` to operator clients, Gazebo, the bridges, `move_group`, RViz, and the planner. It leaves `rmw_zenohd` running.

### The "Total Purge" Command

If `ur5 kill` reports that `gz sim` is still running, repeat it, then check:

```bash
pgrep -a -f 'gz sim' || echo "gz sim not running"
```

Do not start a second `ur5 sim` while one is alive. The dispatcher refuses with exit code 3.

## 2. Nuke the Corrupted Cache

```bash
ur5 clean
```

This removes `<workspace>/build`, `install`, and `log`. It does not remove `$UR5_DATA_DIR`. The same thing by hand:

```bash
rm -rf ~/ros2_ws/build ~/ros2_ws/install ~/ros2_ws/log
```

## 3. Rebuild and Relaunch

```bash
ur5 build
ur5 sim
```

`ur5 build` is `colcon build --symlink-install` from the workspace. A plain `colcon build` also works; Python imports resolve from `install/ur5_controller/lib/ur5_controller` either way.

## 4. Candle reset

The state reset does not restart Gazebo.

```bash
ur5 candle
# or
ros2 run ur5_controller interactive_control.py --candle
```

It switches to Brain A, moves to `candle_pose`, and checks the hold. A new SRDF value is not a reset: `ur5 kill` and `ur5 sim` after `ur5 build`.

## 5. Zenoh router

Every `ur5` command sets `RMW_IMPLEMENTATION=rmw_zenoh_cpp` and starts `rmw_zenohd` if it is down. If `ros2 ctl` sees nothing while Gazebo is clearly running, the shell and the launch are on different RMWs.

```bash
ur5 zenoh
```

That kills `rmw_zenohd` and starts it again. The log is `/tmp/ur5_zenohd.log`.

## 6. Sample-drop reruns

On a 4-vCPU host the physics step can stall. A torque trial that drops a 10 ms sample fails `ur5 validate`. The runner reruns that trial up to three times and moves the bad files to `$UR5_DATA_DIR/rejected/`. Close other programs before a long slice. Do not loosen the validator.

## 7. Symptom, cause, fix

| Symptom | Cause | Fix |
|---|---|---|
| `a simulation is already running` | A `gz sim` process is still up | `ur5 kill`, then `ur5 sim` |
| `ros2` sees an empty graph | Zenoh router down, or another RMW | `ur5 zenoh` |
| Candle goal times out | Controllers still spawning, or the arm is far from the pose | Wait for bring-up, or `ur5 candle` again. Do not start a second Gazebo |
| `ur5 validate` reports gaps | Sim-time stall | Rerun. The torque runner already does this three times |
| `controller plug-in ... not found` | YAML `plugin` is not one of the four laws | Use `pd_gravity`, `pd_id_ff`, `computed_torque`, or `learned_id` |
| Clearance test names a link | `candle_pose` puts that link on the workbench | Edit the SRDF and `ur5 test --unit` again |
| `learned_id: model file ... not found` | Weights are not shipped | Remove `learned_id` from the experiment or add the `.npz` under `$UR5_DATA_DIR/models/` |
