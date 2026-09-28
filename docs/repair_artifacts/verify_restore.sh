#!/usr/bin/env bash
# Verify restored spawner bring-up: controllers + Candle + optional GUI/RViz.
set +e
set -o pipefail
ART="$(cd "$(dirname "$0")" && pwd)"
SCR="$(cd "$ART/../../scripts" && pwd)"
# shellcheck disable=SC1091
source "$SCR/ur5_env.sh"
set +e

MODE="${1:-headless}"  # headless | gui

bash "$ART/hard_clean.sh"
source "$SCR/ur5_env.sh"
set +e
ur5zenoh_restart
sleep 3

LOG="$ART/verify_restore_${MODE}.log"
if [[ "$MODE" == "gui" ]]; then
  nohup bash -lc "source '$SCR/ur5_env.sh'; set +e; ur5simgui" >"$LOG" 2>&1 &
else
  nohup bash -lc "source '$SCR/ur5_env.sh'; set +e; ur5sim" >"$LOG" 2>&1 &
fi
echo "LAUNCH_PID=$! MODE=$MODE"

ACTIVE=0
STABLE=0
# Ignore early stale discovery: spawners start ~2s after entity create,
# and gazebo_sim delays sim by 1s — don't accept ACTIVE before t=8s.
for i in $(seq 1 120); do
  out=$(timeout 4 ros2 control list_controllers 2>/dev/null)
  # Require trailing "active" (not "inactive")
  if (( i >= 8 )) \
     && echo "$out" | grep -E 'ur_manipulator_controller[[:space:]].+[[:space:]]active[[:space:]]*$' >/dev/null \
     && echo "$out" | grep -E 'joint_state_broadcaster[[:space:]].+[[:space:]]active[[:space:]]*$' >/dev/null; then
    STABLE=$((STABLE + 1))
    echo "ACTIVE_candidate_${i}s (stable=$STABLE)"
    echo "$out"
    if (( STABLE >= 3 )); then
      # Confirm clock is fresh (sim time advancing) and single CM
      cm_count=$(timeout 3 ros2 node list 2>/dev/null | grep -c '^/controller_manager$' || true)
      echo "controller_manager_nodes=${cm_count}"
      if (( cm_count <= 1 )); then
        ACTIVE=1
        break
      fi
      echo "WARN: duplicate controller_manager — keep waiting"
      STABLE=0
    fi
  else
    STABLE=0
  fi
  if (( i % 15 == 0 )); then
    echo "t=${i}s"
    echo "$out" | head -5
    pgrep -af 'gz sim' | head -5
    grep -E 'FATAL|Loaded|Configured|Entity|spawner' "$LOG" | tail -12
  fi
  sleep 1
done

if [[ "$ACTIVE" != 1 ]]; then
  echo "FAIL_${MODE}"
  grep -E 'FATAL|Loaded|Configured|Entity|spawner|Error' "$LOG" | head -80
  exit 1
fi

timeout 8 ros2 control list_controllers | tee "$ART/verify_restore_ctl.log"
timeout 12 ros2 topic echo /joint_states --once | tee "$ART/verify_restore_js.log"
pgrep -af 'rviz2|gz sim|robot_state_publisher' | tee "$ART/verify_restore_pids.log"

python3 - <<'PY'
import re, pathlib, sys, os
art = pathlib.Path('/home/kallestewart/ros2_ws/src/ur5_digital_twin/docs/repair_artifacts')
mode = os.environ.get('VERIFY_MODE', 'headless')
text = (art / 'verify_restore_js.log').read_text()
ctl = (art / 'verify_restore_ctl.log').read_text()
pids = (art / 'verify_restore_pids.log').read_text()
name_block = re.search(r'name:\s*\n((?:\s*-\s*.+\n)+)', text)
pos_block = re.search(r'position:\s*\n((?:\s*-\s*.+\n)+)', text)
nlist = re.findall(r'-\s+(\S+)', name_block.group(1)) if name_block else []
plist = [float(x) for x in re.findall(r'-\s+([-\d.eE+]+)', pos_block.group(1))] if pos_block else []
m = dict(zip(nlist, plist))
sl = m.get('shoulder_lift_joint')
print('shoulder_lift', sl, 'joints', m)
jsb = next(l for l in ctl.splitlines() if 'joint_state_broadcaster' in l)
jtc = next(l for l in ctl.splitlines() if 'ur_manipulator' in l)
fec = next((l for l in ctl.splitlines() if 'forward_effort' in l), 'MISSING_FEC')
print(jsb); print(jtc); print(fec)
ok = True
if 'active' not in jsb:
    print('FAIL JSB'); ok = False
if 'active' not in jtc:
    print('FAIL JTC'); ok = False
if fec != 'MISSING_FEC' and 'inactive' not in fec:
    print('FAIL FEC not inactive'); ok = False
if sl is None or abs(sl + 1.5708) > 0.05:
    print('FAIL candle'); ok = False
else:
    print('CANDLE_OK')
if mode == 'gui':
    if 'rviz2' not in pids:
        print('FAIL rviz'); ok = False
    else:
        print('RVIZ_OK')
    if 'gz sim' not in pids and 'gz-sim' not in pids:
        print('WARN gz process string not found in pids')
    else:
        print('GZ_OK')
print('PASS' if ok else 'FAIL')
sys.exit(0 if ok else 1)
PY
