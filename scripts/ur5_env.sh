# shellcheck shell=bash
# UR5 digital twin shell functions. Load into bash with `source <repo>/scripts/ur5_env.sh`,
# or call them through scripts/ur5.

if [[ -z "${BASH_VERSION:-}" ]]; then
  echo "ERROR: ur5_env.sh must be sourced from bash; or run scripts/ur5 help" >&2
  return 3 2>/dev/null || exit 3
fi

if [[ -n "${UR5_ENV_LOADED:-}" ]]; then
  return 0 2>/dev/null || true
fi
UR5_ENV_LOADED=1

_UR5_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_UR5_REPO_ROOT="$(cd "${_UR5_SCRIPT_DIR}/.." && pwd)"
_UR5_WS_ROOT="$(cd "${_UR5_REPO_ROOT}/../.." && pwd)"
if [[ "$(basename "$(dirname "${_UR5_REPO_ROOT}")")" != "src" ]]; then
  echo "WARN: repo is not under <workspace>/src; using ${_UR5_WS_ROOT} as the workspace" >&2
fi

_ur5_die() {
  local code="$1"; shift
  echo "ERROR: $*" >&2
  return "${code}"
}

ur5env() {
  if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
    _ur5_die 3 "/opt/ros/jazzy/setup.bash not found; install ROS 2 Jazzy (docs/setup.md)"; return
  fi
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  export RMW_IMPLEMENTATION=rmw_zenoh_cpp
  export ZENOH_ROUTER_CHECK_ATTEMPTS="${ZENOH_ROUTER_CHECK_ATTEMPTS:-20}"
  export UR5_DATA_DIR="${UR5_DATA_DIR:-${_UR5_REPO_ROOT}/ur5_controller/src/data}"
  if [[ -f "${_UR5_WS_ROOT}/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "${_UR5_WS_ROOT}/install/setup.bash"
  elif [[ "${1:-}" != "--no-install" ]]; then
    _ur5_die 3 "${_UR5_WS_ROOT}/install/setup.bash not found; run: ur5 build"; return
  fi
  if ! pgrep -x rmw_zenohd >/dev/null 2>&1; then
    echo "+ ros2 run rmw_zenoh_cpp rmw_zenohd &   # log /tmp/ur5_zenohd.log"
    setsid ros2 run rmw_zenoh_cpp rmw_zenohd >/tmp/ur5_zenohd.log 2>&1 < /dev/null &
    local i
    for i in 1 2 3 4 5 6 7 8 9 10; do
      pgrep -x rmw_zenohd >/dev/null 2>&1 && break
      sleep 0.5
    done
    pgrep -x rmw_zenohd >/dev/null 2>&1 || { _ur5_die 3 "rmw_zenohd failed to start; see /tmp/ur5_zenohd.log"; return; }
  fi
}

ur5zenoh_restart() {
  ur5env --no-install || return
  echo "+ pkill rmw_zenohd"
  pkill -x rmw_zenohd 2>/dev/null || true
  sleep 1
  echo "+ ros2 run rmw_zenoh_cpp rmw_zenohd &"
  setsid ros2 run rmw_zenoh_cpp rmw_zenohd >/tmp/ur5_zenohd.log 2>&1 < /dev/null &
  sleep 2
  pgrep -x rmw_zenohd >/dev/null 2>&1 || _ur5_die 3 "rmw_zenohd did not start; see /tmp/ur5_zenohd.log"
}

ur5build() {
  ur5env --no-install || return
  local cmd=(colcon build --symlink-install)
  [[ $# -gt 0 ]] && cmd+=(--packages-select "$@")
  echo "+ cd ${_UR5_WS_ROOT} && ${cmd[*]}"
  (cd "${_UR5_WS_ROOT}" && "${cmd[@]}") || return 1
  # shellcheck disable=SC1091
  source "${_UR5_WS_ROOT}/install/setup.bash"
}

ur5clean() {
  echo "+ rm -rf ${_UR5_WS_ROOT}/{build,install,log}   # ${UR5_DATA_DIR:-${_UR5_REPO_ROOT}/ur5_controller/src/data} is kept"
  rm -rf "${_UR5_WS_ROOT}/build" "${_UR5_WS_ROOT}/install" "${_UR5_WS_ROOT}/log"
}

# PIDs whose command line matches $1, skipping shells and tools that merely mention the pattern.
_ur5_pids() {
  local p a0
  for p in $(pgrep -f "$1" 2>/dev/null); do
    [[ "${p}" == "$$" || "${p}" == "${BASHPID}" ]] && continue
    IFS= read -r -d '' a0 < "/proc/${p}/cmdline" 2>/dev/null || continue
    case "$(basename "${a0}")" in
      bash|sh|dash|zsh|cursorsandbox|pgrep|pkill|timeout|grep|rg) continue ;;
    esac
    echo "${p}"
  done
}

ur5kill() {
  echo "+ stop operator clients, Gazebo, bridges, move_group, RViz, planner, spawners (rmw_zenohd stays)"
  local pat pids
  for pat in 'interactive_control.py' 'planner_cli.py' 'preflight.py' 'switch_brain.py' \
             'gz sim' 'bringup.launch.py' 'gazebo_sim.launch.py' 'ur5_description/launch/sim.launch.py' 'planner.launch.py' \
             'parameter_bridge' 'robot_state_publisher' 'move_group' 'rviz2' 'multi_waypoint_planner' \
             'controller_manager/spawner'; do
    pids="$(_ur5_pids "${pat}")"
    [[ -n "${pids}" ]] && kill -9 ${pids} 2>/dev/null
  done
  sleep 1
  pids="$(_ur5_pids 'gz sim')"
  if [[ -n "${pids}" ]]; then
    _ur5_die 1 "gz sim still running: ${pids//$'\n'/,}"
  fi
}

_ur5_refuse_second_sim() {
  local pids
  pids="$(_ur5_pids 'gz sim')"
  if [[ -n "${pids}" ]]; then
    _ur5_die 3 "a simulation is already running (gz pid ${pids//$'\n'/,}); use it, or run: ur5 kill"
  fi
}

ur5sim() {
  ur5env || return
  _ur5_refuse_second_sim || return
  echo "+ ros2 launch ur5_controller bringup.launch.py gz_headless:=true start_rviz:=false $*"
  ros2 launch ur5_controller bringup.launch.py gz_headless:=true start_rviz:=false "$@"
}

ur5simgui() {
  ur5env || return
  _ur5_refuse_second_sim || return
  local engine="${GZ_SIM_RENDER_ENGINE:-ogre}"
  echo "+ ros2 launch ur5_controller bringup.launch.py gz_headless:=false start_rviz:=true gz_render_engine:=${engine} $*"
  ros2 launch ur5_controller bringup.launch.py gz_headless:=false start_rviz:=true \
    gz_render_engine:="${engine}" "$@"
}

ur5setup() {
  local marker="# UR5 digital twin"
  local rc="${HOME}/.bashrc"
  if [[ -f "${rc}" ]] && grep -qF "${marker}" "${rc}"; then
    echo "+ ${rc} already adds ${_UR5_SCRIPT_DIR} to PATH"
  else
    printf '\n%s\nexport PATH="%s:$PATH"\n' "${marker}" "${_UR5_SCRIPT_DIR}" >> "${rc}"
    echo "+ appended PATH line to ${rc}; open a new terminal"
  fi
  export PATH="${_UR5_SCRIPT_DIR}:${PATH}"
}

ur5candle() {
  ur5env || return
  echo "+ ros2 run ur5_controller preflight.py"
  ros2 run ur5_controller preflight.py
}

ur5reset() {
  ur5candle "$@"
}

ur5run() {
  ur5env || return
  local mode=joint csv="" args=()
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --cartesian) mode=cartesian ;;
      --joint) mode=joint ;;
      --out|--csv) args+=("$1" "${2:-}"); shift ;;
      --force) args+=("$1") ;;
      -*) _ur5_die 2 "unknown option $1 (ur5 run [<csv>] [--cartesian] [--out <name>] [--force])"; return ;;
      *) csv="$1" ;;
    esac
    shift
  done
  [[ -z "${csv}" ]] && csv="default_${mode}.csv"
  echo "+ ros2 run ur5_controller interactive_control.py --scheme position --mode ${mode} --csv ${csv} ${args[*]}"
  ros2 run ur5_controller interactive_control.py --scheme position --mode "${mode}" --csv "${csv}" "${args[@]}"
}

_ur5_complete_csv() {
  local cur="${COMP_WORDS[COMP_CWORD]}" d files=""
  for d in "${UR5_DATA_DIR:-${_UR5_REPO_ROOT}/ur5_controller/src/data}/waypoints" "${_UR5_REPO_ROOT}/ur5_controller/src/waypoints"; do
    [[ -d "${d}" ]] && files+=" $(cd "${d}" && ls -1 -- *.csv 2>/dev/null)"
  done
  # shellcheck disable=SC2207
  COMPREPLY=($(compgen -W "${files}" -- "${cur}"))
}
complete -F _ur5_complete_csv ur5run 2>/dev/null || true

ur5demo() {
  ur5run demo_15s.csv "$@"
}

ur5torque() {
  ur5env || return
  local exp="${1:-}"
  if [[ -z "${exp}" || "${exp}" == -* ]]; then
    _ur5_die 2 "usage: ur5 torque <experiment.yaml> [--trial <id>] [--seed <n>]"; return
  fi
  shift
  if [[ ! -f "${exp}" ]]; then
    local shipped
    shipped="$(ros2 pkg prefix ur5_controller)/share/ur5_controller/config/experiments/${exp}"
    [[ -f "${shipped}" ]] && exp="${shipped}"
  fi
  echo "+ ros2 run ur5_controller interactive_control.py --scheme torque --experiment ${exp} $*"
  ros2 run ur5_controller interactive_control.py --scheme torque --experiment "${exp}" "$@"
}

ur5brain() {
  ur5env || return
  case "${1:-}" in
    a|A|b|B) ;;
    *) _ur5_die 2 "usage: ur5 brain a | ur5 brain b [--sec N]"; return ;;
  esac
  echo "+ ros2 run ur5_controller switch_brain.py $*"
  ros2 run ur5_controller switch_brain.py "$@"
}

# Run planner_cli.py with "$@", starting multi_waypoint_planner in the background unless one is
# already running, and stopping it afterwards if this call started it.
_ur5_with_planner() {
  local planner_pid="" rc=0
  if [[ -z "$(_ur5_pids 'multi_waypoint_planner')" ]]; then
    echo "+ ros2 launch ur5_controller planner.launch.py &   # log /tmp/ur5_planner.log"
    setsid ros2 launch ur5_controller planner.launch.py > /tmp/ur5_planner.log 2>&1 < /dev/null &
    planner_pid=$!
  fi
  echo "+ ros2 run ur5_controller planner_cli.py $*"
  ros2 run ur5_controller planner_cli.py "$@" || rc=$?
  if [[ -n "${planner_pid}" ]]; then
    kill -INT -- "-${planner_pid}" 2>/dev/null || true
    wait "${planner_pid}" 2>/dev/null || true
  fi
  return "${rc}"
}

ur5planner() {
  ur5env || return
  if [[ ! -t 0 ]]; then
    _ur5_die 2 "ur5 planner needs a terminal; headless: ur5 plan <name> --points <file.points.yaml>"; return
  fi
  if [[ -z "$(_ur5_pids 'gz sim')" ]]; then
    _ur5_die 3 "no simulation running; start one with: ur5 sim (or ur5 simgui for RViz Publish Point)"; return
  fi
  _ur5_with_planner "$@"
}

ur5plan() {
  ur5env || return
  local name="" points="" force=()
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --points) points="${2:-}"; shift ;;
      -f|--force) force=(--force) ;;
      -*) _ur5_die 2 "unknown option $1"; return ;;
      *) name="$1" ;;
    esac
    shift
  done
  if [[ -z "${name}" ]]; then
    _ur5_die 2 "usage: ur5 plan <name> [--points <file.points.yaml>] [--force]"; return
  fi
  if [[ -z "$(_ur5_pids 'gz sim')" ]]; then
    _ur5_die 3 "no simulation running; start one with: ur5 sim"; return
  fi
  local args=(--export "${name}" "${force[@]}")
  [[ -n "${points}" ]] && args+=(--points "${points}")
  _ur5_with_planner "${args[@]}"
}

ur5ctl() {
  ur5env || return
  echo "+ ros2 control list_controllers"
  ros2 control list_controllers
}

ur5iface() {
  ur5env || return
  echo "+ ros2 control list_hardware_interfaces"
  ros2 control list_hardware_interfaces
}

ur5states() {
  ur5env || return
  echo "+ ros2 topic echo /joint_states --once"
  ros2 topic echo /joint_states --once
}

ur5hz() {
  ur5env || return
  local topic="${1:-/joint_states}"
  echo "+ ros2 topic hz ${topic}"
  ros2 topic hz "${topic}"
}

ur5wp() {
  ur5env || return
  local d
  for d in "$(ros2 pkg prefix ur5_controller)/share/ur5_controller/waypoints" "${UR5_DATA_DIR}/waypoints"; do
    echo "+ ls ${d}"
    ls -1 "${d}" 2>/dev/null || echo "  (none)"
  done
}

ur5data() {
  ur5env --no-install || return
  echo "UR5_DATA_DIR=${UR5_DATA_DIR}"
  local d
  for d in waypoints telemetry models doctor; do
    printf '  %-10s %s files\n' "${d}" "$(find "${UR5_DATA_DIR}/${d}" -type f 2>/dev/null | wc -l)"
  done
}

ur5validate() {
  ur5env || return
  if [[ -z "${1:-}" ]]; then
    _ur5_die 2 "usage: ur5 validate <file.h5 | directory>"; return
  fi
  echo "+ ros2 run ur5_controller validate_h5.py $*"
  ros2 run ur5_controller validate_h5.py "$@"
}

ur5doctor() {
  ur5env || return
  local dir="${UR5_DATA_DIR}/doctor" art
  mkdir -p "${dir}" || { _ur5_die 2 "${dir} is not writable; set UR5_DATA_DIR"; return; }
  art="${dir}/doctor_$(date +%Y%m%d_%H%M%S).log"
  {
    echo "=== ur5 doctor $(date -Is) ==="
    uname -m; lsb_release -ds 2>/dev/null
    echo "ROS_DISTRO=${ROS_DISTRO} RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION} UR5_DATA_DIR=${UR5_DATA_DIR}"
    gz sim --versions 2>&1 || true
    pgrep -a rmw_zenohd || echo "rmw_zenohd not running"
    echo "gz sim pids: $(_ur5_pids 'gz sim' | paste -sd, -)"
    timeout 10 ros2 control list_controllers 2>&1 || true
    timeout 5 ros2 topic hz /clock --window 50 2>&1 || true
    timeout 5 ros2 topic hz /joint_states --window 50 2>&1 || true
  } > "${art}" 2>&1
  cat "${art}"
  echo "+ wrote ${art}"
}

ur5test() {
  ur5env || return
  if [[ "${1:-}" == "--unit" ]]; then
    echo "+ python3 -m pytest -q ${_UR5_REPO_ROOT}/ur5_controller/test"
    python3 -m pytest -q "${_UR5_REPO_ROOT}/ur5_controller/test"
    return
  fi
  echo "+ python3 ${_UR5_REPO_ROOT}/test/harness.py $*"
  python3 "${_UR5_REPO_ROOT}/test/harness.py" "$@"
}

if [[ -z "${UR5_QUIET:-}" ]]; then
  echo "ur5_env.sh loaded (repo=${_UR5_REPO_ROOT} ws=${_UR5_WS_ROOT}); try: ur5 help"
fi
