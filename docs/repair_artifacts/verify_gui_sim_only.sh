#!/usr/bin/env bash
set +e
ART="$(cd "$(dirname "$0")" && pwd)"
SCR="$(cd "$ART/../../scripts" && pwd)"
bash "$ART/hard_clean.sh"
# shellcheck disable=SC1091
source "$SCR/ur5_env.sh"
set +e
ur5zenoh_restart
sleep 2
nohup bash -lc "source '$SCR/ur5_env.sh'; set +e; ur5env; ros2 launch ur5_description sim.launch.py gz_headless:=false gz_render_engine:=ogre" \
  >"$ART/verify_gui_sim_only.log" 2>&1 &
echo "PID=$!"
for i in $(seq 1 90); do
  out=$(timeout 3 ros2 control list_controllers 2>/dev/null)
  if echo "$out" | grep -q 'ur_manipulator_controller.*active'; then
    echo "SIM_ONLY_ACTIVE_at_${i}s"
    echo "$out"
    pgrep -a gz | head -5
    timeout 8 ros2 topic echo /joint_states --once | tee "$ART/verify_gui_sim_only_js.log"
    exit 0
  fi
  if (( i % 10 == 0 )); then
    echo "t=$i"
    echo "$out"
    pgrep -a gz | head -3
    grep -E 'FATAL|Loaded|Configured|Entity|died|Error' "$ART/verify_gui_sim_only.log" | tail -10
  fi
  sleep 1
done
echo FAIL
grep -E 'FATAL|Loaded|Configured|Entity|died|Error' "$ART/verify_gui_sim_only.log" | head -40
exit 1
