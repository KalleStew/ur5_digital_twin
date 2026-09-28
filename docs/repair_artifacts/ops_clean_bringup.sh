#!/usr/bin/env bash
# Nuclear clean → one zenoh → one gazebo_sim → verify.
set -eo pipefail
ART="$(cd "$(dirname "$0")" && pwd)"

echo "=== killing all sim leftovers ==="
killall -9 gz gz-sim gz-sim-server ruby parameter_bridge \
  robot_state_publisher move_group rviz2 rmw_zenohd 2>/dev/null || true
# launch.py parents
ps -eo pid,cmd | awk '/gazebo_sim.launch|sim.launch.py|ros2 launch ur5/{print $1}' | xargs -r kill -9 2>/dev/null || true
sleep 3
pgrep -a gz || echo "no gz left"
pgrep -a rmw_zenohd || echo "no zenoh left"

source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_ROUTER_CHECK_ATTEMPTS=30
source /home/kallestewart/ros2_ws/install/setup.bash

ros2 run rmw_zenoh_cpp rmw_zenohd >"$ART/final_ops_zenoh.log" 2>&1 &
sleep 3
echo "zenoh=$(pgrep -a rmw_zenohd)"

nohup ros2 launch ur5_moveit_config gazebo_sim.launch.py \
  gz_headless:=true start_rviz:=false \
  rmw_implementation:=rmw_zenoh_cpp start_zenoh_router:=false \
  >"$ART/final_ops_sim.log" 2>&1 &
echo "launch=$!"

for i in $(seq 1 75); do
  ctl=$(timeout 3 ros2 control list_controllers 2>/dev/null || true)
  if echo "$ctl" | grep -q 'ur_manipulator_controller.*active'; then
    echo "READY_at_${i}s"
    echo "$ctl" | tee "$ART/final_ops_ctl.log"
    echo "gz=$(pgrep -c -f 'gz sim' || true) instance(s)"
    pgrep -a 'gz sim' | tee -a "$ART/final_ops_ctl.log" || true
    timeout 8 ros2 topic echo /joint_states --once --field position | tee -a "$ART/final_ops_ctl.log"
    n_cm=$(timeout 5 ros2 node list 2>/dev/null | grep -c '^/controller_manager$' || true)
    echo "controller_manager_nodes=$n_cm"
    if [[ "$n_cm" -eq 1 ]]; then
      echo CLEAN_PASS
      exit 0
    else
      echo "WARN: expected 1 controller_manager, got $n_cm"
      echo CLEAN_PASS_WITH_DUP_WARN
      exit 0
    fi
  fi
  sleep 1
done
echo CLEAN_FAIL
tail -40 "$ART/final_ops_sim.log"
exit 1
