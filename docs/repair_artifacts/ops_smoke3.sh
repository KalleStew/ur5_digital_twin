#!/usr/bin/env bash
set -eo pipefail
ART="$(cd "$(dirname "$0")" && pwd)"
killall -9 gz gz-sim gz-sim-server parameter_bridge robot_state_publisher \
  move_group rviz2 rmw_zenohd 2>/dev/null || true
sleep 3

source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_ROUTER_CHECK_ATTEMPTS=30
source /home/kallestewart/ros2_ws/install/setup.bash

ros2 run rmw_zenoh_cpp rmw_zenohd >"$ART/ops_zenoh.log" 2>&1 &
sleep 3
pgrep -a rmw_zenohd | tee "$ART/ops_pids.txt"

nohup ros2 launch ur5_description sim.launch.py >"$ART/ops_sim3.log" 2>&1 &
echo "SIM=$!"
sleep 2
nohup ros2 run ros_gz_bridge parameter_bridge \
  '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' >"$ART/ops_clock3.log" 2>&1 &

for i in $(seq 1 60); do
  if timeout 3 ros2 control list_controllers 2>/dev/null | grep -q ur_manipulator; then
    echo "READY_at_${i}s"
    timeout 5 ros2 control list_controllers | tee "$ART/ops_ctl3.log"
    pgrep -a gz | tee -a "$ART/ops_pids.txt" || true
    timeout 8 ros2 topic echo /joint_states --once --field position | tee "$ART/ops_pose3.log"
    echo SMOKE_PASS
    exit 0
  fi
  sleep 1
done
echo FAIL
grep -E 'Loading controller|Unable to connect|Entity|FATAL' "$ART/ops_sim3.log" | head -40 || true
exit 1
