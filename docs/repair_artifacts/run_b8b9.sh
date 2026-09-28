#!/usr/bin/env bash
# B8/B9: three consecutive interactive batch cycles on one Gazebo instance.
set -eo pipefail
ART="$(cd "$(dirname "$0")" && pwd)"
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
source /home/kallestewart/ros2_ws/install/setup.bash

IC=/home/kallestewart/ros2_ws/src/ur5_digital_twin/ur5_controller/src/interactive_control.py
mkdir -p /tmp/data
OUT="$ART/B8B9_cycles.log"
: > "$OUT"

gz_pid() { pgrep -n -f 'gz sim' || true; }

echo "=== PRE ===" | tee -a "$OUT"
GZ0=$(gz_pid)
echo "GZ_PID=$GZ0" | tee -a "$OUT"
timeout 5 ros2 control list_controllers | tee -a "$OUT"
timeout 10 ros2 topic echo /joint_states --once --field position | tee -a "$OUT"

for c in 1 2 3; do
  echo "=== CYCLE $c START $(date -Is) ===" | tee -a "$OUT"
  echo "GZ_BEFORE=$(gz_pid)" | tee -a "$OUT"
  cd /tmp
  timeout 120 python3 "$IC" --mode 1 --csv default_joint.csv --out "cycle${c}" 2>&1 | tee -a "$OUT" | tail -30
  echo "GZ_AFTER=$(gz_pid)" | tee -a "$OUT"
  timeout 5 ros2 control list_controllers | tee -a "$OUT"
  echo "--- candle check ---" | tee -a "$OUT"
  timeout 10 ros2 topic echo /joint_states --once --field position | tee -a "$OUT"
  echo "threads=$(ps -eLf 2>/dev/null | grep -c interactive_control || echo 0)" | tee -a "$OUT"
  (lsof 2>/dev/null | grep '\.h5' || echo "no leaked h5 handles") | tee -a "$OUT"
  ls -la /tmp/data/ 2>/dev/null | tee -a "$OUT"
done

echo "=== POST ===" | tee -a "$OUT"
echo "GZ_FINAL=$(gz_pid) GZ0=$GZ0" | tee -a "$OUT"
# HDF5 sanity on last cycle
python3 - <<'PY' | tee -a "$OUT"
import h5py, numpy as np, glob, os
files = sorted(glob.glob('/tmp/data/**/*.h5', recursive=True), key=os.path.getmtime)
print('h5_files', files)
if not files:
    raise SystemExit('NO_H5')
f = files[-1]
h = h5py.File(f, 'r')
print(f)
h.visit(print)
keys = [k for k in h.keys() if 'time' in k.lower()]
if not keys:
    # try nested
    def find_time(name, obj):
        if isinstance(obj, h5py.Dataset) and 'time' in name.lower():
            find_time.found = name
    find_time.found = None
    h.visititems(find_time)
    tname = find_time.found
else:
    tname = keys[0]
if tname:
    t = h[tname][:]
    print('samples', len(t), 'monotonic', bool(np.all(np.diff(t) > 0)) if len(t)>1 else False,
          'mean_dt', float(np.mean(np.diff(t))) if len(t)>1 else None)
h.close()
print('H5_OK')
PY
