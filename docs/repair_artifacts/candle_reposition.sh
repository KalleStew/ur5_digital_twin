#!/usr/bin/env bash
# Headless bring-up (reliable) + Candle hold, then optional note for GUI.
set +e
ART="$(cd "$(dirname "$0")" && pwd)"
SCR="$(cd "$ART/../../scripts" && pwd)"
bash "$ART/hard_clean.sh"
# shellcheck disable=SC1091
source "$SCR/ur5_env.sh"
set +e
ur5zenoh_restart
sleep 2

# Prefer GUI for operator; fall back to headless if spawn stalls.
MODE=gui
nohup bash -lc "source '$SCR/ur5_env.sh'; set +e; ur5simgui" >"$ART/candle_reposition_sim.log" 2>&1 &
echo "LAUNCH_PID=$! MODE=$MODE"

READY=0
for i in $(seq 1 75); do
  out=$(timeout 4 ros2 control list_controllers 2>/dev/null)
  if (( i >= 8 )) && echo "$out" | grep -E 'ur_manipulator_controller[[:space:]].+[[:space:]]active[[:space:]]*$' >/dev/null; then
    echo "READY_at_${i}s ($MODE)"
    READY=1
    break
  fi
  # Detect stuck create (world never ready)
  if (( i == 35 )) && grep -q 'Requesting list of world names' "$ART/candle_reposition_sim.log" \
     && ! grep -q 'Entity creation successful' "$ART/candle_reposition_sim.log"; then
    echo "WARN: GUI spawn stalled — restarting headless"
    bash "$ART/hard_clean.sh"
    source "$SCR/ur5_env.sh"; set +e
    ur5zenoh_restart; sleep 2
    MODE=headless
    nohup bash -lc "source '$SCR/ur5_env.sh'; set +e; ur5sim" >"$ART/candle_reposition_sim.log" 2>&1 &
    echo "LAUNCH_PID=$! MODE=$MODE"
    i=0
  fi
  sleep 1
done

if (( READY != 1 )); then
  echo FAIL_NO_CONTROLLERS
  grep -E 'Entity|FATAL|Loaded|Configured|Error' "$ART/candle_reposition_sim.log" | tail -40
  exit 1
fi

# Even if spawn already Candle, command hold to upright (handles sag / prior motion)
echo "+ ur5reset → Candle upright"
ur5reset
rc=$?
sleep 2
timeout 8 ros2 control list_controllers | tee "$ART/candle_reposition_ctl.log"
timeout 10 ros2 topic echo /joint_states --once | tee "$ART/candle_reposition_js.log"

python3 - <<'PY'
import re, pathlib, sys
art = pathlib.Path('/home/kallestewart/ros2_ws/src/ur5_digital_twin/docs/repair_artifacts')
text = (art / 'candle_reposition_js.log').read_text()
name_block = re.search(r'name:\s*\n((?:\s*-\s*.+\n)+)', text)
pos_block = re.search(r'position:\s*\n((?:\s*-\s*.+\n)+)', text)
nlist = re.findall(r'-\s+(\S+)', name_block.group(1)) if name_block else []
plist = [float(x) for x in re.findall(r'-\s+([-\d.eE+]+)', pos_block.group(1))] if pos_block else []
m = dict(zip(nlist, plist))
sl = m.get('shoulder_lift_joint')
print('pose', {k: round(v, 4) for k, v in m.items()})
print('shoulder_lift', sl)
ok = sl is not None and abs(sl + 1.5708) < 0.08
print('CANDLE_REPOSITION_OK' if ok else 'CANDLE_REPOSITION_FAIL')
sys.exit(0 if ok else 1)
PY
