# UR5 7-DOF Digital Twin & HIL Research Platform

A digital twin and hardware-in-the-loop platform for a custom 7-DOF UR5 arm. It runs on Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, and `rmw_zenoh_cpp`.

## System Overview

The arm has one kinematic brain and one torque brain. Only one of them commands the joints.

1. **Kinematic trajectory planning.** `position_control.py` (Brain A) executes waypoint CSVs through `ur_manipulator_controller`. `multi_waypoint_planner` plans Cartesian paths in MoveIt and exports the CSV.
2. **Torque control.** `torque_control.py` (Brain B) takes `forward_effort_controller` and runs PD + gravity, PD + inverse-dynamics feedforward, computed torque, or a learned inverse-dynamics plug-in.

```text
candle_pose (SRDF)
        |
        v
gazebo_sim.launch.py --> joint_state_broadcaster
                      --> ur_manipulator_controller     position_control.py
                      --> forward_effort_controller     torque_control.py
```

A reset is `ur5 candle`. It switches to Brain A, moves to `candle_pose`, and checks the settle. It does not restart Gazebo.

### Key Features

* **Exclusive handoff.** `ur_manipulator_controller` and `forward_effort_controller` are never active together. The switch is one controller-manager call and the post-condition is checked.
* **Two joint-state sources.** MoveIt keeps `/joint_states` from `joint_state_broadcaster`. Brain B and the HDF5 log read `/ur5/gz/joint_states` (the name is `topics.joint_states` in `platform.yaml`), decimated by stamp to 100 Hz.
* **Telemetry.** Each run writes a raw N×7 HDF5 file with metadata. `ur5 validate` checks shape, time, rate, and the Nyquist artefact criterion.
* **One starting pose.** `candle_pose` in the SRDF is the only pose number. Launch writes it into the URDF `initial_value` placeholders.

## Repository Structure

```text
ur5_digital_twin/
├── README.md
├── docs/
├── scripts/  ur5  ur5_env.sh  run_sim
├── ur5_description/          URDF, meshes, empty world, sim.launch.py
├── ur5_moveit_config/        SRDF, MoveIt and controller YAML, gazebo_sim.launch.py
└── ur5_controller/
    ├── config/               platform.yaml and experiment_*.yaml
    ├── launch/planner.launch.py
    ├── src/                  control code, waypoints/, data/
    └── test/
```

`ur5_controller/src/` holds `interactive_control.py` (operator entry), `position_control.py` (Brain A), `torque_control.py` (Brain B), `control_laws.py`, `ur5_common.py`, `dynamics.py`, `telemetry.py`, and `planner_cli.py`, plus `multi_waypoint_planner.cpp`.

## Quick Start

From a workspace whose `src/` contains this repository, after [environment setup](docs/environment_setup.md):

```bash
cd ~/ros2_ws
src/ur5_digital_twin/scripts/ur5 build
# Stage 1. Headless Gazebo, MoveIt, arm upright at candle_pose.
ros2 launch ur5_moveit_config gazebo_sim.launch.py gz_headless:=true start_rviz:=false
# same thing:
src/ur5_digital_twin/scripts/ur5 sim
```

Stage 2, in a second terminal after `source install/setup.bash` (or another `ur5` command, which sources it):

```bash
ros2 launch ur5_controller planner.launch.py
ros2 run ur5_controller planner_cli.py
# same thing:
ur5 planner
```

## Workflow

| Stage | Plain command | Shortcut |
|---|---|---|
| 1 Bring-up | `ros2 launch ur5_moveit_config gazebo_sim.launch.py` | `ur5 simgui` / `ur5 sim` |
| 2 Plan | `ros2 launch ur5_controller planner.launch.py` and `ros2 run ur5_controller planner_cli.py` | `ur5 planner` / `ur5 plan <name>` |
| 3a Run | `ros2 run ur5_controller interactive_control.py --scheme position --csv <file>` | `ur5 run <file>` |
| 3b Experiment | `ros2 run ur5_controller interactive_control.py --scheme torque --experiment <yaml>` | `ur5 torque <yaml>` |
| 4 Reset / export | `ros2 run ur5_controller interactive_control.py --candle` and `ros2 run ur5_controller telemetry.py validate <path>` | `ur5 candle` / `ur5 validate` |

`interactive_control.py` with no flags opens a menu: run waypoints, run experiment, return to Candle, switch brain, validate data, quit.

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

## Documentation Overview

1. [System Architecture](docs/architecture.md) — brains, topics, and which file does what.
2. [Environment Setup](docs/environment_setup.md) — Ubuntu, Jazzy, Harmonic, Zenoh, and the workspace.
3. [Operation & Deployment](docs/operation_and_deployment.md) — the four stages, terminal by terminal.
4. [Development Workflow](docs/development_workflow.md) — branches, the torque laws, and the hardware roadmap.
5. [Troubleshooting & Reset](docs/troubleshooting_and_reset.md) — kill, clean, Candle, and Zenoh.
6. [External Resources](docs/external_resources.md) — upstream manuals.
