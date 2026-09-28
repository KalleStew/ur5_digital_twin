#!/usr/bin/env bash
# One-shot presentation demo: GUI sim + ~12 s arm motion + Candle return.
# Usage (from any cwd):
#   ~/ros2_ws/src/ur5_digital_twin/scripts/run_presentation_demo.sh
#
# What you will see (~25–40 s wall clock including bring-up):
#   1) Gazebo + RViz windows
#   2) Arm starts in Candle Pose
#   3) ~12 s sweeping motion (demo_15s.csv)
#   4) Automatic return to Candle
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
unset UR5_ENV_LOADED
# shellcheck disable=SC1091
source "${ROOT}/ur5_env.sh"

export GZ_SIM_RENDER_ENGINE="${GZ_SIM_RENDER_ENGINE:-ogre}"
# Soft GL fallback helps some ARM64 VMs; harmless if unused.
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"

echo "=== UR5 presentation demo ==="
echo "Render engine: ${GZ_SIM_RENDER_ENGINE}"
echo "DISPLAY=${DISPLAY:-unset}"

ur5kill
ur5zenoh_restart

echo "=== launching GUI (Gazebo + RViz) — leave this process running ==="
# Launch in background so we can drive the arm from this script.
nohup ros2 launch ur5_moveit_config gazebo_sim.launch.py \
  gz_headless:=false start_rviz:=true \
  gz_render_engine:="${GZ_SIM_RENDER_ENGINE}" \
  rmw_implementation:=rmw_zenoh_cpp start_zenoh_router:=false \
  >"${ROOT}/../docs/repair_artifacts/presentation_sim.log" 2>&1 &
LAUNCH_PID=$!
echo "launch_pid=${LAUNCH_PID}"

echo "=== waiting for controllers (up to 90 s) ==="
ready=0
for i in $(seq 1 90); do
  if timeout 3 ros2 control list_controllers 2>/dev/null | grep -q 'ur_manipulator_controller.*active'; then
    echo "READY_at_${i}s"
    ready=1
    break
  fi
  sleep 1
done

if [[ "${ready}" -ne 1 ]]; then
  echo "FAIL: controllers never came up. See docs/repair_artifacts/presentation_sim.log" >&2
  exit 1
fi

ros2 control list_controllers
echo "=== starting 12 s motion demo — watch Gazebo ==="
mkdir -p /tmp/ur5_presentation_data
cd /tmp/ur5_presentation_data
python3 "${_UR5_REPO_ROOT}/ur5_controller/src/interactive_control.py" \
  --mode 1 --csv demo_15s.csv --out presentation_live

echo ""
echo "=== DEMO COMPLETE ==="
echo "Arm should be back in Candle Pose."
echo "Sim is still running (pid ${LAUNCH_PID}). Ctrl+C that launch or run: ur5 kill"
echo "Telemetry: /tmp/ur5_presentation_data/"
