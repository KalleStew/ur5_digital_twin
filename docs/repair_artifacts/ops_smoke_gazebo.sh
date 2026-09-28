#!/usr/bin/env bash
# Test full ur5sim path (gazebo_sim.launch.py) against a live zenohd.
set -eo pipefail
ART="$(cd "$(dirname "$0")" && pwd)"
killall -9 gz gz-sim gz-sim-server parameter_bridge robot_state_publisher \
  move_group rviz2 2>/dev/null || true
sleep 2

source /home/kallestewart/ros2_ws/src/ur5_digital_twin/scripts/ur5_env.sh
# Force unset so we can re-source after edits
unset UR5_ENV_LOADED
source /home/kallestewart/ros2_ws/src/ur5_digital_twin/scripts/ur5_env.sh
ur5zenoh_restart

nohup ros2 launch ur5_moveit_config gazebo_sim.launch.py \
  gz_headless:=true start_rviz:=false \
  rmw_implementation:=rmw_zenoh_cpp start_zenoh_router:=false \
  >"$ART/ops_gazebo_sim.log" 2>&1 &
echo "LAUNCH=$!"

for i in $(seq 1 90); do
  if timeout 3 ros2 control list_controllers 2>/dev/null | grep -q ur_manipulator; then
    echo "READY_at_${i}s"
    {
      ros2 control list_controllers
      timeout 8 ros2 topic echo /joint_states --once --field name
      timeout 8 ros2 topic echo /joint_states --once --field position
      pgrep -a gz || true
    } | tee "$ART/ops_gazebo_ok.log"
    echo GAZEBO_SIM_PASS
    exit 0
  fi
  sleep 1
done
echo GAZEBO_SIM_FAIL
grep -E 'Loading controller|Unable to connect|Entity|FATAL|error' "$ART/ops_gazebo_sim.log" | head -50
exit 1
