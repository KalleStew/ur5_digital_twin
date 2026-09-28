#!/usr/bin/env bash
set -eo pipefail
ART="$(cd "$(dirname "$0")" && pwd)"
killall -9 gz gz-sim gz-sim-server parameter_bridge robot_state_publisher \
  move_group rviz2 rmw_zenohd 2>/dev/null || true
ps -eo pid,cmd | awk '/gazebo_sim.launch|sim.launch.py|ensure_controllers/{print $1}' | xargs -r kill -9 2>/dev/null || true
sleep 3

source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_ROUTER_CHECK_ATTEMPTS=30
source /home/kallestewart/ros2_ws/install/setup.bash

ros2 run rmw_zenoh_cpp rmw_zenohd >"$ART/fix_zenoh.log" 2>&1 &
sleep 3

nohup ros2 launch ur5_description sim.launch.py >"$ART/fix_sim.log" 2>&1 &
sleep 2
nohup ros2 run ros_gz_bridge parameter_bridge \
  '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' >"$ART/fix_clock.log" 2>&1 &

# Wait for ensure_controllers to finish candle + JTC active
ok=0
for i in $(seq 1 120); do
  ctl=$(timeout 3 ros2 control list_controllers 2>/dev/null || true)
  if echo "$ctl" | grep -q 'ur_manipulator_controller.*active'; then
    echo "JTC_ACTIVE_at_${i}s"
    # give candle command a few seconds
    sleep 6
    ok=1
    break
  fi
  sleep 1
done

{
  echo "$ctl"
  timeout 10 ros2 topic echo /joint_states --once --field name
  timeout 10 ros2 topic echo /joint_states --once --field position
} | tee "$ART/fix_pose.log"

python3 - <<'PY'
import re
from pathlib import Path
t=Path('/home/kallestewart/ros2_ws/src/ur5_digital_twin/docs/repair_artifacts/fix_pose.log').read_text()
arrays=re.findall(r"array\('d', \[([^\]]+)\]\)", t)
alpha=['elbow_joint','shoulder_lift_joint','shoulder_pan_joint','shoulder_pan_joint1','wrist_1_joint','wrist_2_joint','wrist_3_joint']
if not arrays:
    print('NO_POS'); raise SystemExit(1)
pos=[float(x.strip()) for x in arrays[-1].split(',')]
for n,p in zip(alpha,pos):
    print(f'{n:24s} {p: .6f}')
lift=pos[1]
ok=abs(lift-(-1.5708))<0.08
print('CANDLE_OK' if ok else 'CANDLE_FAIL', 'lift=', lift)
raise SystemExit(0 if ok else 2)
PY
