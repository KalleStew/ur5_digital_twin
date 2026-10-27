# UR5 7-DOF digital twin

ROS 2 Jazzy, Gazebo Harmonic, and `rmw_zenoh_cpp` on Ubuntu 24.04. One arm, two controllers, one of them active at a time.

Brain A is the joint-trajectory controller. It runs shipped or user waypoint files and the MoveIt planner. Brain B is the torque benchmark: PD + gravity, PD + inverse-dynamics feedforward, computed torque, and a learned inverse-dynamics plug-in. Switching controllers is `ur5 brain` or the start of a torque trial. Resetting the arm is `ur5 candle`. That returns it to `candle_pose` under Brain A. It does not restart Gazebo.

`scripts/ur5` finds the workspace, sources Jazzy, sets `RMW_IMPLEMENTATION=rmw_zenoh_cpp`, and starts `rmw_zenohd` if it is not already running. User files go to `$UR5_DATA_DIR`, which defaults to `<workspace>/ur5_data`.

Setup is [docs/setup.md](docs/setup.md). The torque benchmark is [docs/experiments.md](docs/experiments.md).

Exit codes: 0 ok, 1 run failed, 2 input rejected, 3 platform not ready, 4 safety abort, 130 interrupted.

## Commands

| Command | What it does |
|---|---|
| `ur5 setup` | Add `scripts/` to `PATH` in `~/.bashrc` |
| `ur5 build` | `colcon build --symlink-install` from the workspace |
| `ur5 clean` | Remove `build/`, `install/`, and `log/`. Keeps `$UR5_DATA_DIR` |
| `ur5 doctor` | Environment report, saved under `$UR5_DATA_DIR/doctor` |
| `ur5 zenoh` | Restart the `rmw_zenohd` router |
| `ur5 sim` | Headless Gazebo + MoveIt. One session at a time |
| `ur5 simgui` | Gazebo GUI + RViz |
| `ur5 run_sim` | Same as `ur5 simgui` |
| `ur5 kill` | Stop Gazebo, bridges, `move_group`, and RViz |
| `ur5 candle` | Pre-flight: Brain A, move to `candle_pose`, verify the settle |
| `ur5 reset` | Same as `ur5 candle` |
| `ur5 run` | Execute a waypoint CSV, record HDF5, return to Candle |
| `ur5 demo` | Run `demo_15s.csv` |
| `ur5 plan` | Plan through task-space points and export a CSV |
| `ur5 planner` | Interactive planner menu. RViz Publish Point needs `ur5 simgui` |
| `ur5 brain` | Switch control authority. `b` holds the pose with PD + gravity |
| `ur5 torque` | Run an experiment YAML. One HDF5 file per trial |
| `ur5 data` | Show `$UR5_DATA_DIR` and file counts |
| `ur5 wp` | List shipped and user waypoint files |
| `ur5 validate` | Check telemetry shape, rate, timestamps, and metadata |
| `ur5 ctl` | `ros2 control list_controllers` |
| `ur5 iface` | `ros2 control list_hardware_interfaces` |
| `ur5 states` | One `/joint_states` message |
| `ur5 hz` | Topic rate. Default topic is `/joint_states` |
| `ur5 test` | Unit tests with `--unit`. Without it, the full sim acceptance run |

## Walkthrough

From a workspace whose `src/` contains this repository, and with ROS 2 Jazzy installed (see [docs/setup.md](docs/setup.md)):

```bash
cd ~/ros2_ws
src/ur5_digital_twin/scripts/ur5 build
src/ur5_digital_twin/scripts/ur5 sim
src/ur5_digital_twin/scripts/ur5 candle
src/ur5_digital_twin/scripts/ur5 demo
src/ur5_digital_twin/scripts/ur5 kill
```

`ur5 run` with no file runs `default_joint.csv`. Add `--cartesian` for `default_cartesian.csv`. A torque slice that does not need learned weights:

```bash
src/ur5_digital_twin/scripts/ur5 sim
src/ur5_digital_twin/scripts/ur5 torque slice_s10.yaml
src/ur5_digital_twin/scripts/ur5 validate "$UR5_DATA_DIR"
src/ur5_digital_twin/scripts/ur5 kill
```
