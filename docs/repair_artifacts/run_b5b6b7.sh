#!/usr/bin/env bash
# B5 isolation + B6 exclusivity + B7 rate — requires sim up with FEC loaded.
set -eo pipefail
ART="$(cd "$(dirname "$0")" && pwd)"
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
source /home/kallestewart/ros2_ws/install/setup.bash
OUT="$ART/B5B6B7.log"
: > "$OUT"

echo "=== B6 illegal double-activation ===" | tee -a "$OUT"
timeout 15 ros2 service call /controller_manager/switch_controller \
  controller_manager_msgs/srv/SwitchController \
  "{activate_controllers: ['forward_effort_controller'], deactivate_controllers: [], strictness: 2}" | tee -a "$OUT"

echo "=== B6 handoff A->B ===" | tee -a "$OUT"
timeout 15 ros2 service call /controller_manager/switch_controller \
  controller_manager_msgs/srv/SwitchController \
  "{activate_controllers: ['forward_effort_controller'], deactivate_controllers: ['ur_manipulator_controller'], strictness: 2}" | tee -a "$OUT"
timeout 5 ros2 control list_controllers | tee -a "$OUT"
timeout 5 ros2 control list_hardware_interfaces | tee -a "$OUT"

echo "=== B5 isolation (20 Nm) ===" | tee -a "$OUT"
timeout 10 ros2 param get /forward_effort_controller joints | tee -a "$OUT"
for i in 0 1 2 3 4 5 6; do
  data=$(python3 -c "d=[0.0]*7; d[$i]=20.0; print('[%s]' % ', '.join(str(x) for x in d))")
  echo "INDEX $i data=$data" | tee -a "$OUT"
  # zero first
  timeout 5 ros2 topic pub --once /forward_effort_controller/commands std_msgs/msg/Float64MultiArray "{data: [0,0,0,0,0,0,0]}" >/dev/null
  sleep 0.3
  timeout 5 ros2 topic pub --once /forward_effort_controller/commands std_msgs/msg/Float64MultiArray "{data: $data}" >/dev/null
  sleep 0.4
  timeout 10 ros2 topic echo /joint_states --once | tee -a "$OUT"
  timeout 5 ros2 topic pub --once /forward_effort_controller/commands std_msgs/msg/Float64MultiArray "{data: [0,0,0,0,0,0,0]}" >/dev/null
  sleep 0.5
done

echo "=== B6 handoff B->A ===" | tee -a "$OUT"
timeout 15 ros2 service call /controller_manager/switch_controller \
  controller_manager_msgs/srv/SwitchController \
  "{activate_controllers: ['ur_manipulator_controller'], deactivate_controllers: ['forward_effort_controller'], strictness: 2}" | tee -a "$OUT"
timeout 5 ros2 control list_controllers | tee -a "$OUT"

echo "=== B7 (will need FEC active + publisher for rate; skip live 100Hz driver measure) ===" | tee -a "$OUT"
timeout 8 ros2 topic hz /joint_states --window 200 2>&1 | tee -a "$OUT" || true
timeout 5 ros2 topic echo /clock --once | tee -a "$OUT" || true
echo DONE | tee -a "$OUT"
