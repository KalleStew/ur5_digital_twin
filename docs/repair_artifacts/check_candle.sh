#!/usr/bin/env bash
set -eo pipefail
ART="$(cd "$(dirname "$0")" && pwd)"
killall -9 gz gz-sim gz-sim-server parameter_bridge robot_state_publisher \
  move_group rviz2 rmw_zenohd 2>/dev/null || true
ps -eo pid,cmd | awk '/gazebo_sim.launch|sim.launch.py/{print $1}' | xargs -r kill -9 2>/dev/null || true
sleep 3

source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_ROUTER_CHECK_ATTEMPTS=30
source /home/kallestewart/ros2_ws/install/setup.bash

ros2 run rmw_zenoh_cpp rmw_zenohd >"$ART/candle_zenoh.log" 2>&1 &
sleep 3

# Use same path as successful B4: sim.launch headless + clock
nohup ros2 launch ur5_description sim.launch.py >"$ART/candle_sim.log" 2>&1 &
sleep 2
nohup ros2 run ros_gz_bridge parameter_bridge \
  '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' >"$ART/candle_clock.log" 2>&1 &

for i in $(seq 1 60); do
  if timeout 3 ros2 control list_controllers 2>/dev/null | grep -q 'ur_manipulator_controller.*active'; then
    echo "READY_at_${i}s"
    break
  fi
  sleep 1
done

{
  echo '=== controllers ==='
  ros2 control list_controllers
  echo '=== names ==='
  timeout 10 ros2 topic echo /joint_states --once --field name
  echo '=== positions (alpha order) ==='
  timeout 10 ros2 topic echo /joint_states --once --field position
  echo '=== efforts ==='
  timeout 10 ros2 topic echo /joint_states --once --field effort
} | tee "$ART/candle_pose_check.log"

python3 - <<'PY'
import re
from pathlib import Path
t=Path('/home/kallestewart/ros2_ws/src/ur5_digital_twin/docs/repair_artifacts/candle_pose_check.log').read_text()
names=re.search(r'names ===\n(.*?)===', t, re.S)
# parse arrays
import ast
# ros2 prints array('d', [...]) or yaml list
pos_m=re.search(r"positions.*?===\n(.*?)\n---", t, re.S)
# simpler: last array
arrays=re.findall(r"array\('d', \[([^\]]+)\]\)", t)
names_m=re.search(r"name:\n((?:- .+\n)+)", t)
if not names_m:
    # field name prints as python list sometimes
    pass
print('arrays_found', len(arrays))
if arrays:
    pos=[float(x.strip()) for x in arrays[0].split(',')]
    # alpha order from earlier sessions
    alpha=['elbow_joint','shoulder_lift_joint','shoulder_pan_joint','shoulder_pan_joint1','wrist_1_joint','wrist_2_joint','wrist_3_joint']
    print('alpha zip:')
    for n,p in zip(alpha,pos):
        print(f'  {n:24s} {p: .6f}')
    lift=pos[1]
    ok=abs(lift - (-1.5708)) < 0.05
    print('CANDLE_OK' if ok else 'CANDLE_FAIL', 'lift=', lift)
PY
