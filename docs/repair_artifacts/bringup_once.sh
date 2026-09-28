#!/usr/bin/env bash
set -eo pipefail
ART="$(cd "$(dirname "$0")" && pwd)"
# Kill by binary name only — never pkill -f patterns that match this script.
killall -9 gz gz-sim gz-sim-server rmw_zenohd parameter_bridge robot_state_publisher 2>/dev/null || true
sleep 2

# ROS setup scripts reference optional unset vars — do not enable nounset around them.
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_ROUTER_CHECK_ATTEMPTS=20
source /home/kallestewart/ros2_ws/install/setup.bash

ros2 run rmw_zenoh_cpp rmw_zenohd >"$ART/zenoh_clean.log" 2>&1 &
sleep 2
pgrep -a rmw_zenohd | tee "$ART/zenoh_pid.txt"

nohup ros2 launch ur5_description sim.launch.py >"$ART/bringup_retry.log" 2>&1 &
echo LP=$!
sleep 2
nohup ros2 run ros_gz_bridge parameter_bridge \
  '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' >"$ART/bringup_clock.log" 2>&1 &

for i in $(seq 1 90); do
  if grep -q 'Loading controller_manager' "$ART/bringup_retry.log" 2>/dev/null; then
    echo "CM_LOADED_at_${i}"
  fi
  if timeout 2 ros2 control list_controllers 2>/dev/null | grep -q ur_manipulator; then
    echo "READY_at_${i}"
    timeout 5 ros2 control list_controllers | tee "$ART/bringup_ctl.log"
    pgrep -a gz | tee "$ART/bringup_gz.pid" || true
    exit 0
  fi
  sleep 1
done

echo FAIL
grep -E 'gz_ros|controller_manager|error|Error|FATAL|plugin|Entity' "$ART/bringup_retry.log" | head -80 || true
tail -50 "$ART/bringup_retry.log"
exit 1
