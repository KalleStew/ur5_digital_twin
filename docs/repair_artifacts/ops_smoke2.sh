#!/usr/bin/env bash
# Ops smoke: clean → ur5env → gazebo_sim (zenoh) → verify controllers + candle.
set -eo pipefail
ART="$(cd "$(dirname "$0")" && pwd)"
LOG="$ART/ops_smoke2.log"
: > "$LOG"

killall -9 gz gz-sim gz-sim-server parameter_bridge robot_state_publisher \
  rviz2 move_group 2>/dev/null || true
pkill -9 -f 'ros2 launch ur5' 2>/dev/null || true
sleep 2

# Fresh single zenoh
killall -9 rmw_zenohd 2>/dev/null || true
sleep 1

source /home/kallestewart/ros2_ws/src/ur5_digital_twin/scripts/ur5_env.sh
ur5env | tee -a "$LOG"

# Rebuild moveit_config so launch default change is installed (symlink usually enough)
# Symlink-install: source tree is already what launch reads via share symlink.
ls -la "$(ros2 pkg prefix ur5_moveit_config)/share/ur5_moveit_config/launch/gazebo_sim.launch.py" | tee -a "$LOG"
grep -n "default_value='rmw_zenoh_cpp'" \
  "$(ros2 pkg prefix ur5_moveit_config)/share/ur5_moveit_config/launch/gazebo_sim.launch.py" | tee -a "$LOG"

nohup ros2 launch ur5_moveit_config gazebo_sim.launch.py \
  gz_headless:=true start_rviz:=false rmw_implementation:=rmw_zenoh_cpp \
  >"$ART/ops_sim2.log" 2>&1 &
echo LAUNCH_PID=$! | tee -a "$LOG"

ready=0
for i in $(seq 1 90); do
  if timeout 3 ros2 control list_controllers 2>/dev/null | grep -q 'ur_manipulator_controller'; then
    echo "READY_at_${i}s" | tee -a "$LOG"
    ready=1
    break
  fi
  # Also watch for CM plugin load
  if grep -q 'Loading controller_manager' "$ART/ops_sim2.log" 2>/dev/null; then
    echo "CM_LOG_at_${i}s" | tee -a "$LOG"
  fi
  sleep 1
done

if [[ "$ready" -ne 1 ]]; then
  echo FAIL | tee -a "$LOG"
  grep -E 'gz_ros|controller_manager|Entity|FATAL|error|Error' "$ART/ops_sim2.log" | head -40 | tee -a "$LOG"
  tail -30 "$ART/ops_sim2.log" | tee -a "$LOG"
  exit 1
fi

{
  echo '=== PASS diagnostics ==='
  pgrep -a 'gz sim' || pgrep -a gz
  ros2 control list_controllers
  timeout 8 ros2 topic echo /joint_states --once --field name
  timeout 8 ros2 topic echo /joint_states --once --field position
  timeout 5 ros2 topic hz /clock --window 30 || true
  echo 'NOTE: headless = no GUI window. Controllers above = system is up.'
} | tee -a "$LOG"

echo SMOKE_PASS | tee -a "$LOG"
