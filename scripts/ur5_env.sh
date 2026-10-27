# shellcheck shell=bash
# UR5 Digital Twin — developer convenience wrappers (D-007).
# Source this file in BASH (not sh/dash):
#   source ~/ros2_ws/src/ur5_digital_twin/scripts/ur5_env.sh
# Or run without sourcing:
#   ~/ros2_ws/src/ur5_digital_twin/scripts/ur5 sim

if [[ -z "${BASH_VERSION:-}" ]]; then
  echo "ERROR: ur5_env.sh must be sourced from bash (you are in ${0:-unknown})." >&2
  echo "  bash --login" >&2
  echo "  source ~/ros2_ws/src/ur5_digital_twin/scripts/ur5_env.sh" >&2
  echo "Or run: ~/ros2_ws/src/ur5_digital_twin/scripts/ur5 help" >&2
  return 1 2>/dev/null || exit 1
fi

if [[ -n "${UR5_ENV_LOADED:-}" ]]; then
  return 0 2>/dev/null || true
fi
UR5_ENV_LOADED=1

_UR5_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_UR5_REPO_ROOT="$(cd "${_UR5_SCRIPT_DIR}/.." && pwd)"
_UR5_WS_ROOT="$(cd "${_UR5_REPO_ROOT}/../../.." 2>/dev/null && pwd)"
# Prefer detecting workspace by walking up for install/setup.bash
_probe="${_UR5_REPO_ROOT}"
while [[ "${_probe}" != "/" ]]; do
  if [[ -f "${_probe}/install/setup.bash" ]]; then
    _UR5_WS_ROOT="${_probe}"
    break
  fi
  _probe="$(dirname "${_probe}")"
done

ur5env() {
  echo "+ source /opt/ros/jazzy/setup.bash"
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"
  export ZENOH_ROUTER_CHECK_ATTEMPTS="${ZENOH_ROUTER_CHECK_ATTEMPTS:-20}"
  echo "+ export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
  if [[ -f "${_UR5_WS_ROOT}/install/setup.bash" ]]; then
    echo "+ source ${_UR5_WS_ROOT}/install/setup.bash"
    # shellcheck disable=SC1091
    source "${_UR5_WS_ROOT}/install/setup.bash"
  else
    echo "WARN: workspace install/setup.bash not found under ${_UR5_WS_ROOT}" >&2
  fi
  # Always ensure a live router. A stale rmw_zenohd presents as "nodes up, nothing talks".
  if pgrep -x rmw_zenohd >/dev/null 2>&1 || pgrep -f '/rmw_zenoh_cpp/rmw_zenohd' >/dev/null 2>&1; then
    echo "+ rmw_zenohd already running (pid $(pgrep -d, -f '/rmw_zenoh_cpp/rmw_zenohd' || true))"
  else
    echo "+ ros2 run rmw_zenoh_cpp rmw_zenohd &"
    ros2 run rmw_zenoh_cpp rmw_zenohd >/tmp/ur5_zenohd.log 2>&1 &
    sleep 2
  fi
  if ! pgrep -f '/rmw_zenoh_cpp/rmw_zenohd' >/dev/null 2>&1; then
    echo "ERROR: rmw_zenohd failed to start — see /tmp/ur5_zenohd.log" >&2
    return 1
  fi
}

ur5zenoh_restart() {
  ur5env
  echo "+ restarting rmw_zenohd"
  killall -9 rmw_zenohd 2>/dev/null || true
  sleep 1
  echo "+ ros2 run rmw_zenoh_cpp rmw_zenohd &"
  ros2 run rmw_zenoh_cpp rmw_zenohd >/tmp/ur5_zenohd.log 2>&1 &
  sleep 2
  pgrep -a rmw_zenohd || echo "ERROR: zenohd did not start" >&2
}

ur5build() {
  ur5env
  local cmd=(colcon build --symlink-install)
  if [[ $# -gt 0 ]]; then
    cmd+=(--packages-select "$@")
  fi
  echo "+ cd ${_UR5_WS_ROOT} && ${cmd[*]}"
  (cd "${_UR5_WS_ROOT}" && "${cmd[@]}")
  # shellcheck disable=SC1091
  source "${_UR5_WS_ROOT}/install/setup.bash"
}

ur5clean() {
  echo "+ rm -rf ${_UR5_WS_ROOT}/{build,install,log}"
  rm -rf "${_UR5_WS_ROOT}/build" "${_UR5_WS_ROOT}/install" "${_UR5_WS_ROOT}/log"
}

ur5kill() {
  echo "+ killall gz / bridges / move_group / rviz (leave zenohd)"
  killall -9 gz gz-sim gz-sim-server parameter_bridge \
    robot_state_publisher move_group rviz2 2>/dev/null || true
  pkill -9 -f 'gz_tools_vendor/bin/gz sim' 2>/dev/null || true
  pkill -9 -f 'ruby .*/bin/gz sim' 2>/dev/null || true
  # Match by process args without killing this shell (avoid embedding patterns in caller cmdline).
  ps -eo pid=,args= | while read -r pid args; do
    case "$args" in
      *gazebo_sim.launch.py*|*ur5_description/launch/sim.launch.py*)
        kill -9 "$pid" 2>/dev/null || true
        ;;
      *'gz sim '*|*gz_sim*)
        kill -9 "$pid" 2>/dev/null || true
        ;;
    esac
  done
  sleep 1
  pgrep -af 'gz sim' || echo "+ no gz left"
}

ur5sim() {
  ur5env
  # Headless: no Gazebo/RViz window by design — use ur5simgui for visuals.
  echo "NOTE: headless = no GUI. Proof of life is 'ur5ctl' in another terminal."
  echo "+ ros2 launch ur5_moveit_config gazebo_sim.launch.py gz_headless:=true start_rviz:=false rmw_implementation:=rmw_zenoh_cpp start_zenoh_router:=false $*"
  ros2 launch ur5_moveit_config gazebo_sim.launch.py \
    gz_headless:=true start_rviz:=false \
    rmw_implementation:=rmw_zenoh_cpp start_zenoh_router:=false "$@"
}

ur5simgui() {
  ur5env
  # ARM64 VM: default ogre (ogre2 often fails under software GL). Override with GZ_SIM_RENDER_ENGINE.
  local engine="${GZ_SIM_RENDER_ENGINE:-ogre}"
  echo "NOTE: Gazebo + RViz windows should open. Engine=${engine}"
  echo "+ ros2 launch ur5_moveit_config gazebo_sim.launch.py gz_headless:=false start_rviz:=true gz_render_engine:=${engine} rmw_implementation:=rmw_zenoh_cpp start_zenoh_router:=false $*"
  ros2 launch ur5_moveit_config gazebo_sim.launch.py \
    gz_headless:=false start_rviz:=true gz_render_engine:="${engine}" \
    rmw_implementation:=rmw_zenoh_cpp start_zenoh_router:=false "$@"
}

ur5setup() {
  # Install PATH so `ur5` works in every new bash terminal.
  local marker="# UR5 digital twin shortcuts"
  local line="export PATH=\"${_UR5_SCRIPT_DIR}:\$PATH\""
  local rc="${HOME}/.bashrc"
  if [[ -f "${rc}" ]] && grep -qF "${marker}" "${rc}" 2>/dev/null; then
    echo "+ already installed in ${rc}"
  else
    {
      echo ""
      echo "${marker}"
      echo "${line}"
      echo "# optional: auto-load functions — uncomment next line"
      echo "# source ${_UR5_SCRIPT_DIR}/ur5_env.sh"
    } >> "${rc}"
    echo "+ appended PATH to ${rc} — open a new terminal or: source ~/.bashrc"
  fi
  export PATH="${_UR5_SCRIPT_DIR}:${PATH}"
  echo "+ PATH updated for this shell. Try: ur5 help"
}

ur5candle() {
  ur5env
  local script
  script="$(ros2 pkg prefix ur5_description)/lib/ur5_description/ensure_controllers.py"
  if [[ ! -f "${script}" ]]; then
    script="${_UR5_REPO_ROOT}/ur5_description/scripts/ensure_controllers.py"
  fi
  echo "+ python3 ${script}   # activate JTC + hold Candle"
  python3 "${script}"
}

ur5demo() {
  # 10–15 s presentation motion (demo_15s.csv) + Candle return. Sim must already be up.
  ur5env
  local out="${1:-presentation_demo}"
  echo "+ presentation demo → waypoints/demo_15s.csv (≈12 s motion + Candle reset)"
  echo "+ ros2 run ur5_controller interactive_control.py --mode 1 --csv demo_15s.csv --out ${out}"
  ros2 run ur5_controller interactive_control.py \
    --mode 1 --csv demo_15s.csv --out "${out}"
}

ur5planner() {
  # multi_waypoint_planner: subscribes /clicked_point (RViz Publish Point) + /planner_command.
  ur5env
  echo "+ ros2 launch ur5_controller planner.launch.py $*"
  ros2 launch ur5_controller planner.launch.py "$@"
}

ur5plan() {
  ur5env
  local name="${1:-}"
  shift || true
  if [[ -z "${name}" ]]; then
    echo "usage: ur5plan <name> [-f]  # copies trajectory_joints_export.csv → waypoints/<name>.csv" >&2
    echo "NOTICE (R5): planner_cli.py has no argv flags — stdin only (plan|execute|export joints|...)." >&2
    echo "NOTICE (R5): waypoint DEFINITION still requires RViz Publish Point → /clicked_point." >&2
    return 1
  fi
  local wp
  # Same resolution as multi_waypoint_planner / interactive_control (package share).
  wp="$(ros2 pkg prefix ur5_controller)/share/ur5_controller/waypoints"
  local out="${wp}/${name}.csv"
  local force=0
  if [[ "${1:-}" == "-f" ]]; then force=1; shift; fi
  if [[ -f "${out}" && "${force}" -ne 1 ]]; then
    echo "REFUSE: ${out} exists (pass -f to overwrite)" >&2
    return 1
  fi
  # planner_cli.py only publishes to /planner_command; multi_waypoint_planner does the work.
  local planner_pid=""
  if ! ros2 node list 2>/dev/null | grep -qx "/multi_waypoint_planner"; then
    echo "+ starting multi_waypoint_planner in background (log: /tmp/ur5_planner.log)"
    setsid ros2 launch ur5_controller planner.launch.py > /tmp/ur5_planner.log 2>&1 < /dev/null &
    planner_pid=$!
    local i
    for i in $(seq 1 30); do
      if ros2 topic info /planner_command 2>/dev/null | grep -q "Subscription count: [1-9]"; then
        break
      fi
      if ! kill -0 "${planner_pid}" 2>/dev/null; then
        echo "ERROR: multi_waypoint_planner exited — see /tmp/ur5_planner.log" >&2
        return 1
      fi
      sleep 1
    done
    if ! ros2 topic info /planner_command 2>/dev/null | grep -q "Subscription count: [1-9]"; then
      echo "ERROR: multi_waypoint_planner not subscribed to /planner_command after 30 s — see /tmp/ur5_planner.log" >&2
      kill -TERM -- "-${planner_pid}" 2>/dev/null || true
      return 1
    fi
  else
    echo "+ multi_waypoint_planner already running"
  fi
  echo "NOTICE (R5): Manual step remains — in RViz, Publish Point until ≥2 waypoints exist."
  echo "NOTICE (R5): Then in the planner CLI type: plan → export joints (no CLI goal-entry path)."
  echo "NOTICE: planner feedback is in /tmp/ur5_planner.log (tail -f it in another terminal)."
  echo "+ ros2 run ur5_controller planner_cli.py"
  ros2 run ur5_controller planner_cli.py "$@" || true
  if [[ -n "${planner_pid}" ]]; then
    echo "+ stopping background multi_waypoint_planner"
    kill -TERM -- "-${planner_pid}" 2>/dev/null || true
    wait "${planner_pid}" 2>/dev/null || true
  fi
  local src="${wp}/trajectory_joints_export.csv"
  if [[ -f "${src}" ]]; then
    echo "+ cp ${src} ${out}"
    cp "${src}" "${out}"
    echo "Wrote $(readlink -f "${out}" 2>/dev/null || echo "${out}")"
    head -3 "${out}"
  else
    echo "No ${src} yet — run 'export joints' after plan (RViz waypoints required first)." >&2
    return 1
  fi
}

ur5run() {
  ur5env
  local csv="${1:-default_joint.csv}"
  echo "+ ros2 run ur5_controller interactive_control.py --mode 1 --csv ${csv}"
  ros2 run ur5_controller interactive_control.py --mode 1 --csv "${csv}"
}

_ur5run_complete() {
  local wp cur
  cur="${COMP_WORDS[COMP_CWORD]}"
  wp="$(ros2 pkg prefix ur5_controller 2>/dev/null)/share/ur5_controller/waypoints"
  COMPREPLY=()
  if [[ -d "${wp}" ]]; then
    # shellcheck disable=SC2207
    COMPREPLY=($(cd "${wp}" && compgen -W "$(ls -1 ./*.csv 2>/dev/null | xargs -n1 basename)" -- "${cur}"))
  fi
}
complete -F _ur5run_complete ur5run 2>/dev/null || true

ur5reset() {
  ur5env
  echo "+ Candle reset via FollowJointTrajectory (interactive_control return_to_candle values)"
  python3 - <<'PY'
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from trajectory_msgs.msg import JointTrajectoryPoint
import threading

class Reset(Node):
    def __init__(self):
        super().__init__('ur5_candle_reset', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.ac = ActionClient(self, FollowJointTrajectory, '/ur_manipulator_controller/follow_joint_trajectory')
        self.done = threading.Event()
        self.joints = ['shoulder_pan_joint1','shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']
        self.candle = [0.0,0.0,-1.5708,0.0,0.0,0.0,0.0]
    def go(self):
        self.ac.wait_for_server()
        g = FollowJointTrajectory.Goal()
        g.trajectory.joint_names = self.joints
        g.trajectory.header.stamp = self.get_clock().now().to_msg()
        p = JointTrajectoryPoint(); p.positions=list(self.candle); p.velocities=[0.0]*7
        p.time_from_start.sec=4
        g.trajectory.points=[p]
        for n in self.joints:
            g.goal_tolerance.append(JointTolerance(name=n, position=0.05, velocity=0.2))
        fut=self.ac.send_goal_async(g)
        def _acc(f):
            h=f.result()
            if not h.accepted:
                self.done.set(); return
            h.get_result_async().add_done_callback(lambda _: self.done.set())
        fut.add_done_callback(_acc)
        self.done.wait(timeout=30)

rclpy.init(); n=Reset()
threading.Thread(target=n.go, daemon=True).start()
import time; t0=time.time()
while rclpy.ok() and not n.done.is_set() and time.time()-t0<35:
    rclpy.spin_once(n, timeout_sec=0.1)
print('Candle reset', 'OK' if n.done.is_set() else 'TIMEOUT')
n.destroy_node(); rclpy.shutdown()
PY
}

ur5ctl() {
  ur5env
  echo "+ ros2 control list_controllers -v"
  ros2 control list_controllers -v
}

ur5iface() {
  ur5env
  echo "+ ros2 control list_hardware_interfaces"
  ros2 control list_hardware_interfaces
}

ur5brain() {
  ur5env
  local target="${1:-}"
  case "${target}" in
    a|A)
      echo "+ switch → ur_manipulator_controller (deactivate forward_effort)"
      ros2 service call /controller_manager/switch_controller \
        controller_manager_msgs/srv/SwitchController \
        "{activate_controllers: ['ur_manipulator_controller'], deactivate_controllers: ['forward_effort_controller'], strictness: 2}"
      ;;
    b|B)
      echo "+ switch → forward_effort_controller (deactivate ur_manipulator)"
      ros2 service call /controller_manager/switch_controller \
        controller_manager_msgs/srv/SwitchController \
        "{activate_controllers: ['forward_effort_controller'], deactivate_controllers: ['ur_manipulator_controller'], strictness: 2}"
      ;;
    *)
      echo "usage: ur5brain a|b" >&2; return 1 ;;
  esac
  ros2 control list_controllers
}

ur5states() {
  ur5env
  echo "+ ros2 topic echo /joint_states --once"
  ros2 topic echo /joint_states --once
}

ur5hz() {
  ur5env
  local topic="${1:-/joint_states}"
  echo "+ ros2 topic hz ${topic}"
  ros2 topic hz "${topic}"
}

ur5wp() {
  ur5env
  local wp
  wp="$(ros2 pkg prefix ur5_controller)/share/ur5_controller/waypoints"
  echo "+ ls -la ${wp}"
  ls -la "${wp}"
}

ur5doctor() {
  ur5env
  local stamp art
  stamp="$(date +%Y%m%d_%H%M%S)"
  art="${_UR5_REPO_ROOT}/docs/repair_artifacts/doctor_${stamp}.log"
  {
    echo "=== ur5doctor ${stamp} ==="
    uname -m; lsb_release -ds
    echo "ROS_DISTRO=$ROS_DISTRO RMW=$RMW_IMPLEMENTATION"
    gz sim --versions 2>&1 || true
    pgrep -a rmw_zenohd || echo "ZENOH ROUTER NOT RUNNING"
    ros2 control list_controllers 2>&1 || true
    ros2 control list_hardware_interfaces 2>&1 || true
    timeout 5 ros2 topic hz /clock --window 50 2>&1 || true
    timeout 5 ros2 topic hz /joint_states --window 50 2>&1 || true
    timeout 10 ros2 topic echo /joint_states --once --field name 2>&1 || true
  } | tee "${art}"
  echo "Wrote ${art}"
}

echo "ur5_env.sh loaded (repo=${_UR5_REPO_ROOT} ws=${_UR5_WS_ROOT})"
echo "Functions: ur5env ur5zenoh_restart ur5kill ur5setup ur5build ur5clean ur5sim ur5simgui ur5candle ur5demo ur5planner ur5plan ur5run ur5reset ur5ctl ur5iface ur5brain ur5states ur5hz ur5wp ur5doctor"
