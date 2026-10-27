# Setup

Ubuntu 24.04, ROS 2 Jazzy at `/opt/ros/jazzy`, Gazebo Harmonic, and MoveIt 2 for Jazzy. The dispatcher sets `RMW_IMPLEMENTATION=rmw_zenoh_cpp` on every command. Do not point the middleware at a different RMW to work around a fault. `ur5 zenoh` restarts the router.

The repository belongs at `<workspace>/src/ur5_digital_twin`. The usual workspace is `~/ros2_ws`. `$UR5_DATA_DIR` defaults to `<workspace>/ur5_data` and holds telemetry, user waypoints, learned weights, rejected trials, and `ur5 doctor` reports. `ur5 clean` does not delete it.

## Install and build

With Jazzy and rosdep already on the machine:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y
src/ur5_digital_twin/scripts/ur5 build
src/ur5_digital_twin/scripts/ur5 doctor
```

`rosdep` pulls the packages named in `ur5_description/package.xml`, `ur5_moveit_config/package.xml`, and `ur5_controller/package.xml`, including `gz_ros2_control`, `ros_gz_sim`, `ros_gz_bridge`, `rmw_zenoh_cpp`, `python3-numpy`, `python3-h5py`, `python3-yaml`, and the KDL vendor. Tests also need `python3-pytest`.

`ur5 setup` appends `scripts/` to `PATH` in `~/.bashrc`. After that, the same commands are `ur5 build`, `ur5 doctor`, and so on. `ur5 help` lists them.

## Run

```bash
ur5 sim          # headless Gazebo and MoveIt
ur5 candle       # Brain A to candle_pose, then a settle check
ur5 kill         # stop the session
```

`ur5 simgui` (and `ur5 run_sim`) adds the Gazebo GUI and RViz. The render engine is `$GZ_SIM_RENDER_ENGINE`, default `ogre`. Only one `ur5 sim` or `ur5 simgui` session runs at a time.

`ur5 candle` and `ur5 reset` are the state reset. They switch to Brain A, move to `candle_pose`, and check the hold in `ur5_controller/config/platform.yaml`: max joint error 0.005 rad and RMS velocity below 0.001 rad/s. They do not restart Gazebo. `ur5 kill` is the teardown.

## Joint state used by the controllers

Brain B and the telemetry read `/ur5/gz/joint_states` from the Gazebo joint-state publisher and keep one sample per 10 ms of sim time. Brain A execution still uses the `ros2_control` joint-state broadcaster on `/joint_states`. Both are sim time (`use_sim_time`).

On a 4-vCPU host the physics steps sometimes stall and then catch up in a burst. A trial that drops a 10 ms sample fails `ur5 validate`. The torque runner reruns that trial; see [experiments.md](experiments.md). Closing other GUI programs, or giving the VM more cores, is what reduces the stalls. The validator does not ignore them.
