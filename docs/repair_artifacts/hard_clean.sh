#!/usr/bin/env bash
# Hard clean for verify — patterns live in this file so the caller cmdline is safe.
set +e
killall -9 gz gz-sim gz-sim-server parameter_bridge \
  robot_state_publisher move_group rviz2 2>/dev/null
# gz Harmonic often leaves ruby wrappers + "gz sim server/gui" children
pkill -9 -f '/opt/ros/.*/gz sim' 2>/dev/null
pkill -9 -f 'gz_tools_vendor/bin/gz sim' 2>/dev/null
pkill -9 -f 'ruby .*/bin/gz sim' 2>/dev/null
ps -eo pid=,args= | while read -r pid args; do
  case "$args" in
    *gazebo_sim.launch.py*|*ur5_description/launch/sim.launch.py*|*rmw_zenohd*|*controller_manager/spawner*)
      kill -9 "$pid" 2>/dev/null
      ;;
    *'gz sim '*|*'gz-sim'*|*gz_sim*)
      kill -9 "$pid" 2>/dev/null
      ;;
  esac
done
sleep 2
echo AFTER_KILL
pgrep -a 'gz|rviz|zenoh|move_group' || echo clean
# Fail closed if any gz sim still alive
if pgrep -af 'gz sim' >/dev/null 2>&1; then
  echo "WARN: gz sim still present:"
  pgrep -af 'gz sim'
  pkill -9 -f 'gz sim' 2>/dev/null
  sleep 1
fi
