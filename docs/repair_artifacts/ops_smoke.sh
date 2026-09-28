#!/usr/bin/env bash
# End-to-end smoke test of ur5sim (headless) bring-up.
set -eo pipefail
ART="$(cd "$(dirname "$0")" && pwd)"
LOG="$ART/ops_smoke.log"
: > "$LOG"

killall -9 gz gz-sim gz-sim-server parameter_bridge robot_state_publisher rviz2 2>/dev/null || true
# leave zenoh alone if healthy; restart if missing
sleep 2

source /home/kallestewart/ros2_ws/src/ur5_digital_twin/scripts/ur5_env.sh
ur5env | tee -a "$LOG"

echo "=== launching ur5sim in background ===" | tee -a "$LOG"
# ur5sim blocks; run via nohup of the underlying command it echoes
nohup ros2 launch ur5_moveit_config gazebo_sim.launch.py \
  gz_headless:=true start_rviz:=false \
  >"$ART/ops_sim.log" 2>&1 &
echo LAUNCH_PID=$! | tee -a "$LOG"

ready=0
for i in $(seq 1 90); do
  if timeout 3 ros2 control list_controllers 2>/dev/null | grep -q ur_manipulator; then
    echo "READY_at_${i}s" | tee -a "$LOG"
    ready=1
    break
  fi
  sleep 1
done

if [[ "$ready" -ne 1 ]]; then
  echo FAIL_NO_CONTROLLERS | tee -a "$LOG"
  tail -60 "$ART/ops_sim.log" | tee -a "$LOG"
  exit 1
fi

{
  echo '=== controllers ==='
  ros2 control list_controllers
  echo '=== gz ==='
  pgrep -a gz || true
  echo '=== clock ==='
  timeout 5 ros2 topic hz /clock --window 50 || true
  echo '=== joint_states names ==='
  timeout 10 ros2 topic echo /joint_states --once --field name
  echo '=== joint_states position ==='
  timeout 10 ros2 topic echo /joint_states --once --field position
  echo '=== ur5ctl ==='
  ur5ctl
} | tee -a "$LOG"

echo SMOKE_PASS | tee -a "$LOG"
