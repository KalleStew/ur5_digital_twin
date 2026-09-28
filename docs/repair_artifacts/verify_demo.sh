#!/usr/bin/env bash
# Headless verify of presentation trajectory (no GUI).
set -eo pipefail
ART="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$ART/../.." && pwd)/scripts"
# Actually ART is docs/repair_artifacts
SCRIPTS="$(cd "$ART/../../scripts" 2>/dev/null && pwd || true)"
if [[ -z "$SCRIPTS" || ! -f "$SCRIPTS/ur5" ]]; then
  SCRIPTS=/home/kallestewart/ros2_ws/src/ur5_digital_twin/scripts
fi

killall -9 gz gz-sim parameter_bridge robot_state_publisher move_group rviz2 rmw_zenohd 2>/dev/null || true
sleep 2

source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_ROUTER_CHECK_ATTEMPTS=30
source /home/kallestewart/ros2_ws/install/setup.bash

ros2 run rmw_zenoh_cpp rmw_zenohd >"$ART/demo_zenoh.log" 2>&1 &
sleep 2
nohup ros2 launch ur5_description sim.launch.py >"$ART/demo_sim.log" 2>&1 &
sleep 2
nohup ros2 run ros_gz_bridge parameter_bridge \
  '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' >"$ART/demo_clock.log" 2>&1 &

for i in $(seq 1 60); do
  if timeout 3 ros2 control list_controllers 2>/dev/null | grep -q 'ur_manipulator_controller.*active'; then
    echo "READY_at_${i}s"
    break
  fi
  sleep 1
done

timeout 5 ros2 control list_controllers
mkdir -p /tmp/ur5_demo_verify
cd /tmp/ur5_demo_verify
timeout 90 python3 /home/kallestewart/ros2_ws/src/ur5_digital_twin/ur5_controller/src/interactive_control.py \
  --mode 1 --csv demo_15s.csv --out demo_verify 2>&1 | tee "$ART/demo_run.log" | tail -30

timeout 8 ros2 topic echo /joint_states --once --field position | tee "$ART/demo_final_pose.log"
echo DEMO_VERIFY_DONE
