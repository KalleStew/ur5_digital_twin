#!/usr/bin/env python3
"""Headless acceptance harness: one Gazebo session, every §9 row that runs in sim.

Writes a pass/fail table to $UR5_REPORT_DIR (default <repo>/.agent/reports/).
Exit 0 only when every executed check passes.
"""
from __future__ import annotations

import argparse
import contextlib
import csv
import datetime as dt
import fcntl
import glob
import math
import os
import signal
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
UR5 = os.path.join(REPO, 'scripts', 'ur5')
sys.path.insert(0, HERE)

from probe import JOINTS, ProbeRunner, hold_stats  # noqa: E402

CANDLE = [0.0, 0.0, -1.5708, 0.0, 0.0, 0.0, 0.0]
HOLD_MAX_ERR = 0.005
HOLD_RMS_QD = 1e-3
SPAWN_MAX_ERR = 0.01
GOAL_TOL = 0.05
LOCK_PATH = '/tmp/ur5_sim.lock'
JTC = 'ur_manipulator_controller'
FEC = 'forward_effort_controller'


class Report:
    def __init__(self) -> None:
        self.rows: list[tuple[str, str, str, str]] = []

    def add(self, cid: str, criterion: str, ok: bool | None, observed: str) -> bool:
        status = 'SKIP' if ok is None else ('PASS' if ok else 'FAIL')
        self.rows.append((cid, criterion, status, observed.replace('|', '/').replace('\n', ' ')))
        print(f'[{status}] {cid} {criterion}: {observed}', flush=True)
        return bool(ok)

    def failed(self) -> int:
        return sum(1 for r in self.rows if r[2] == 'FAIL')

    def write(self, path: str, header: str) -> None:
        with open(path, 'w') as f:
            f.write(f'# Harness report\n\n{header}\n\n')
            f.write('| ID | Criterion | Result | Observed |\n|---|---|---|---|\n')
            for r in self.rows:
                f.write(f'| {r[0]} | {r[1]} | {r[2]} | {r[3]} |\n')
            p = sum(1 for r in self.rows if r[2] == 'PASS')
            f.write(f'\nPASS {p} / FAIL {self.failed()} / SKIP '
                    f'{sum(1 for r in self.rows if r[2] == "SKIP")}\n')


def gz_pids() -> tuple[int, ...]:
    out = subprocess.run(['pgrep', '-f', 'gz sim'], capture_output=True, text=True).stdout
    pids = []
    for p in out.split():
        try:
            with open(f'/proc/{p}/cmdline', 'rb') as f:
                cmd = f.read().replace(b'\0', b' ').decode()
        except OSError:
            continue
        if 'gz sim' in cmd and '.sdf' in cmd and not cmd.startswith('/bin/sh'):
            pids.append(int(p))
    return tuple(sorted(pids))


def ur5(args: list[str], env: dict, cwd: str, timeout: float, log) -> tuple[int, str]:
    """Run `scripts/ur5 <args>`; returns (exit code, combined output)."""
    log.write(f'\n$ ur5 {" ".join(args)}\n')
    try:
        p = subprocess.run([UR5, *args], env=env, cwd=cwd, capture_output=True, text=True,
                           timeout=timeout, stdin=subprocess.DEVNULL)
        out = p.stdout + p.stderr
        log.write(out)
        return p.returncode, out
    except subprocess.TimeoutExpired as e:
        out = (e.stdout or b'').decode(errors='replace') if isinstance(e.stdout, bytes) else (e.stdout or '')
        log.write(out + '\n<TIMEOUT>\n')
        return 124, out + '<TIMEOUT>'


def ur5_bg(args: list[str], env: dict, cwd: str, log) -> subprocess.Popen:
    """Start `scripts/ur5 <args>` in its own process group; output goes to the command log."""
    log.write(f'\n$ ur5 {" ".join(args)} &\n')
    log.flush()
    return subprocess.Popen([UR5, *args], env=env, cwd=cwd, stdout=log, stderr=subprocess.STDOUT,
                            stdin=subprocess.DEVNULL, start_new_session=True)


def wait_controllers(p, want: dict, timeout: float) -> dict | None:
    # Wall-clock give-up while a background command performs the switch.
    deadline = time.time() + timeout
    st = None
    while time.time() < deadline:
        st = p.controllers()
        if st and all(st.get(k) == v for k, v in want.items()):
            return st
        time.sleep(0.2)
    return st


OPERATOR_SCRIPTS = {'interactive_control.py', 'preflight.py', 'switch_brain.py', 'planner_cli.py',
                    'validate_h5.py'}


def leaked_operator_processes() -> list[str]:
    """Python processes still running an operator script (argv, not a substring of some shell line)."""
    found = []
    for pid in os.listdir('/proc'):
        if not pid.isdigit() or int(pid) == os.getpid():
            continue
        try:
            with open(f'/proc/{pid}/cmdline', 'rb') as f:
                argv = [a.decode(errors='replace') for a in f.read().split(b'\0') if a]
        except OSError:
            continue
        if argv and os.path.basename(argv[0]).startswith('python') and any(
                os.path.basename(a) in OPERATOR_SCRIPTS for a in argv[1:3]):
            found.append(f'{pid} {" ".join(argv)[:100]}')
    return found


def last_line(out: str) -> str:
    lines = [ln for ln in out.strip().splitlines() if ln.strip()]
    return lines[-1][:160] if lines else ''


def read_joint_csv(path: str) -> list[tuple[float, list[float]]]:
    with open(path) as f:
        rows = list(csv.reader(f))
    hdr = [h.strip() for h in rows[0]]
    idx = [hdr.index(j) for j in JOINTS]
    return [(float(r[0]), [float(r[i]) for i in idx]) for r in rows[1:] if r]


def find_new_h5(roots: list[str], since: float) -> list[str]:
    found = []
    for root in roots:
        for p in glob.glob(os.path.join(root, '**', '*.h5'), recursive=True):
            if os.path.getmtime(p) >= since:
                found.append(p)
    return sorted(found, key=os.path.getmtime)


def check_h5(path: str) -> tuple[bool, str]:
    try:
        import h5py
        import numpy as np
    except ImportError as e:
        return False, f'import failed: {e}'
    with h5py.File(path, 'r') as f:
        if 't' in f:
            t = np.asarray(f['t'])
            q = np.asarray(f['q']) if 'q' in f else None
        elif 'timestamp_sec' in f:
            t = np.asarray(f['timestamp_sec'])
            q = None
        else:
            return False, f'no time dataset; keys={list(f.keys())}'
        attrs = dict(f.attrs)
    if t.size < 2:
        return False, f'{t.size} samples'
    problems = []
    if np.all(t == 0):
        problems.append('all timestamps 0')
    if np.any(np.diff(t) <= 0):
        problems.append(f'{int(np.sum(np.diff(t) <= 0))} non-increasing steps')
    if q is None:
        problems.append('no N x 7 q dataset')
    elif q.shape[1:] != (7,):
        problems.append(f'q shape {q.shape}')
    if not attrs:
        problems.append('no metadata attributes')
    span = float(t[-1] - t[0])
    rate = (t.size - 1) / span if span > 0 else 0.0
    return (not problems), (f'n={t.size} rate={rate:.1f}Hz attrs={len(attrs)} '
                            + ('; '.join(problems) if problems else 'ok'))


class Harness:
    def __init__(self, args) -> None:
        self.args = args
        self.r = Report()
        stamp = dt.datetime.now().strftime('%Y%m%d_%H%M%S')
        rep_root = os.environ.get('UR5_REPORT_DIR', os.path.join(REPO, '.agent', 'reports'))
        self.outdir = os.path.join(rep_root, stamp)
        os.makedirs(self.outdir, exist_ok=True)
        self.report_path = os.path.join(rep_root, f'{stamp}.md')
        self.log = open(os.path.join(self.outdir, 'commands.log'), 'w')
        self.work = tempfile.mkdtemp(prefix='ur5_harness_')
        self.data_dir = os.path.join(self.work, 'ur5_data')
        self.cwd = os.path.join(self.work, 'cwd')
        os.makedirs(self.cwd)
        self.env = dict(os.environ, UR5_DATA_DIR=self.data_dir)
        self.sim = None
        self.pids: tuple[int, ...] = ()
        self.t_start = time.time()

    # ---------- infrastructure ----------
    def pid_guard(self, stage: str) -> bool:
        now = gz_pids()
        ok = bool(now) and now == self.pids
        if not ok:
            self.r.add('PID', f'Gazebo PID unchanged after {stage}', False,
                       f'before={self.pids} after={now}')
        return ok

    def start_sim(self) -> None:
        subprocess.run([UR5, 'kill'], env=self.env, capture_output=True, timeout=60)
        self.sim_log = open(os.path.join(self.outdir, 'sim.log'), 'w')
        self.sim = subprocess.Popen([UR5, 'sim'], env=self.env, cwd=self.cwd,
                                    stdout=self.sim_log, stderr=subprocess.STDOUT,
                                    stdin=subprocess.DEVNULL, start_new_session=True)

    def stop_sim(self) -> None:
        if self.sim and self.sim.poll() is None:
            with contextlib.suppress(ProcessLookupError):
                os.killpg(self.sim.pid, signal.SIGINT)
            try:
                self.sim.wait(timeout=20)
            except subprocess.TimeoutExpired:
                with contextlib.suppress(ProcessLookupError):
                    os.killpg(self.sim.pid, signal.SIGKILL)
        subprocess.run([UR5, 'kill'], env=self.env, capture_output=True, timeout=60)

    def settle_hold(self, p, label: str, cid: str, secs: float) -> bool:
        p.clear()
        p.wait_sim(secs, secs * 5 + 30)
        st = hold_stats(p.samples(), CANDLE)
        if not st['n']:
            return self.r.add(cid, f'Candle hold {label}', False, 'no samples')
        me, rq = max(st['max_err']), max(st['rms_qd'])
        worst = JOINTS[st['max_err'].index(me)]
        return self.r.add(cid, f'Candle hold {label} ({secs:.0f}s sim)',
                          me < HOLD_MAX_ERR and rq < HOLD_RMS_QD,
                          f'max|dq|={me:.5f} ({worst}) maxRMS(qd)={rq:.2e} n={st["n"]}')

    # ---------- stages ----------
    def stage_static(self) -> None:
        tests = os.path.join(REPO, 'ur5_controller', 'test')
        if not os.path.isdir(tests):
            self.r.add('S16', 'unit and policy tests', False, 'ur5_controller/test missing')
            return
        p = subprocess.run([sys.executable, '-m', 'pytest', '-q', tests], capture_output=True,
                           text=True, cwd=REPO, timeout=600)
        self.log.write(p.stdout + p.stderr)
        self.r.add('S16', 'unit and policy tests (pytest ur5_controller/test)', p.returncode == 0,
                   last_line(p.stdout))

    def stage_bringup(self, p) -> bool:
        jtc_active_clock = None
        t0 = time.monotonic()
        # Wall-clock: give-up timeout for bring-up.
        while time.monotonic() - t0 < self.args.bringup_timeout:
            states = p.controllers(timeout=2.0)
            if states and states.get(JTC) == 'active' and jtc_active_clock is None:
                jtc_active_clock = p.last_clock
            if (states and states.get(JTC) == 'active'
                    and states.get('joint_state_broadcaster') == 'active'
                    and states.get(FEC) == 'inactive'):
                break
            time.sleep(0.25)
        else:
            self.r.add('S2', 'bring-up reaches JSB+JTC active, FEC inactive', False,
                       f'states={p.controllers(timeout=2.0)}')
            return False
        self.pids = gz_pids()
        self.r.add('S2', 'bring-up reaches JSB+JTC active, FEC inactive', bool(self.pids),
                   f'{time.monotonic() - t0:.1f}s wall, gz pids={self.pids}')
        pre = [s for s in p.samples() if jtc_active_clock is None or
               (s.clock_t is not None and s.clock_t <= jtc_active_clock)]
        if pre:
            st = hold_stats(pre, CANDLE)
            me = max(st['max_err'])
            self.r.add('S3a', 'spawn to JTC-active max|q - candle|', me < SPAWN_MAX_ERR,
                       f'{me:.4f} rad over {len(pre)} samples before JTC active '
                       f'(clock {jtc_active_clock})')
        else:
            self.r.add('S3a', 'spawn to JTC-active max|q - candle|', None,
                       'no /joint_states before JTC active (JSB starts with JTC chain)')
        # Let any launch-time Candle command finish before measuring hold.
        p.wait_sim(self.args.post_bringup, self.args.post_bringup * 5 + 30)
        return True

    def stage_clock(self, p) -> None:
        p.clear()
        p.wait_sim(5.0, 60)
        s = p.samples()
        if len(s) < 10:
            self.r.add('CLK', '/joint_states header stamps', False, f'{len(s)} samples')
            return
        hdr = [x.header_t for x in s]
        mono = all(b > a for a, b in zip(hdr, hdr[1:]))
        span = s[-1].clock_t - s[0].clock_t
        rate = (len(s) - 1) / span if span else 0
        self.r.add('CLK', '/joint_states stamps nonzero, monotonic; rate vs /clock',
                   hdr[0] > 0 and mono and rate > 95,
                   f'first={hdr[0]:.3f} mono={mono} rate={rate:.1f}Hz')

    def write_csv(self, name: str, text: str) -> str:
        path = os.path.join(self.work, name)
        with open(path, 'w') as f:
            f.write(text)
        return path

    def stage_collision(self) -> None:
        files = sorted(glob.glob(os.path.join(REPO, 'ur5_controller', 'waypoints', '*.csv')))
        tracked = subprocess.run(['git', 'ls-files'], cwd=REPO, capture_output=True,
                                 text=True).stdout.split()
        demo = [f for f in tracked if f.endswith('demo_15s.csv')]
        self.r.add('S6a', 'demo_15s.csv tracked in git', bool(demo), demo[0] if demo else 'not tracked')
        checker = os.path.join(HERE, 'collision_check.py')
        for path in files:
            with open(path) as f:
                header = f.readline()
            if 'x' in header.split(',')[1:2] or ',x,' in header:
                kind = 'cartesian'
            else:
                kind = 'joint'
            p = subprocess.run([sys.executable, checker, '--kind', kind, path],
                               capture_output=True, text=True, timeout=300)
            self.log.write(p.stdout + p.stderr)
            rel = os.path.relpath(path, REPO)
            self.r.add('S6', f'collision-free vs workbench: {rel}', p.returncode == 0,
                       last_line(p.stdout + p.stderr))

    def stage_joint_run(self, p) -> None:
        src = os.path.join(REPO, 'ur5_controller', 'waypoints', 'default_joint.csv')
        if not os.path.exists(src):
            self.r.add('S5', 'joint CSV run', False, 'default_joint.csv missing')
            return
        wps = read_joint_csv(src)
        p.clear()
        t_mark = time.time()
        rc, out = ur5(['run', src, '--out', 'harness_joint'], self.env, self.cwd, 300, self.log)
        s = p.samples()
        # Every waypoint must be visited in order (the arm stops at each one).
        k, worst = 0, 0.0
        for i, (_, q_wp) in enumerate(wps):
            best = math.inf
            while k < len(s):
                d = max(abs(a - b) for a, b in zip(s[k].q, q_wp))
                best = min(best, d)
                k += 1
                if d <= GOAL_TOL:
                    break
            worst = max(worst, best)
        self.r.add('S5', 'ur5 run default_joint.csv exits 0 and visits every waypoint <= 0.05',
                   rc == 0 and worst <= GOAL_TOL,
                   f'exit={rc} worst waypoint miss={worst:.4f} | {last_line(out)}')
        self.pid_guard('joint run')
        h5 = find_new_h5([self.data_dir, self.cwd], t_mark)
        self.r.add('S13', 'telemetry written under $UR5_DATA_DIR, not $CWD',
                   bool(h5) and all(x.startswith(self.data_dir) for x in h5),
                   f'{[os.path.relpath(x, self.work) for x in h5]}')
        if h5:
            ok, msg = check_h5(h5[-1])
            self.r.add('S9', 'HDF5 raw N x 7, nonzero monotonic t, metadata', ok, msg)
            v = subprocess.run(['ros2', 'run', 'ur5_controller', 'validate_h5.py', h5[-1]],
                               env=self.env, capture_output=True, text=True, timeout=120)
            self.r.add('S9b', 'validate_h5.py exit 0', v.returncode == 0,
                       last_line(v.stdout + v.stderr))
        else:
            self.r.add('S9', 'HDF5 written', False, 'no new .h5 found')
        self.settle_hold(p, 'after joint run + reset', 'S3c', 5.0)

    def stage_cartesian_run(self, p) -> None:
        src = os.path.join(REPO, 'ur5_controller', 'waypoints', 'default_cartesian.csv')
        if not os.path.exists(src):
            self.r.add('S5c', 'cartesian CSV run', False, 'default_cartesian.csv missing')
            return
        p.clear()
        rc, out = ur5(['run', src, '--cartesian', '--out', 'harness_cart'], self.env, self.cwd,
                      300, self.log)
        moved = max((max(abs(a - b) for a, b in zip(x.q, CANDLE)) for x in p.samples()), default=0)
        self.r.add('S5c', 'ur5 run <cartesian> --cartesian exits 0 and moves the arm',
                   rc == 0 and moved > 0.05, f'exit={rc} max excursion={moved:.3f} | {last_line(out)}')
        self.pid_guard('cartesian run')
        self.settle_hold(p, 'after cartesian run', 'S3d', 5.0)

    def stage_edge_csv(self, p) -> None:
        hdr = 'time_sec,' + ','.join(JOINTS) + '\n'
        good = '2.0,0,0,-1.5708,0,0,0,0\n'
        cases = {
            'missing': None,
            'empty': '',
            'header_only': hdr,
            'wrong_columns': hdr + '2.0,0,0,-1.5708,0,0\n',
            'non_monotonic_time': hdr + '4.0,0,0,-1.5708,0,0,0,0\n' + good,
            'zero_first_time': hdr + '0.0,0,0,-1.4,0,0,0,0\n',
            'nan_value': hdr + '2.0,0,0,nan,0,0,0,0\n',
            'beyond_joint_limit': hdr + '3.0,0,0,-1.5708,3.5,0,0,0\n',
            'collides_with_table': hdr + '4.0,0,0,0,0,0,0,0\n',
        }
        for name, text in cases.items():
            path = os.path.join(self.work, f'edge_{name}.csv')
            if text is not None:
                with open(path, 'w') as f:
                    f.write(text)
            p.clear()
            rc, out = ur5(['run', path, '--out', f'edge_{name}'], self.env, self.cwd, 120, self.log)
            moved = max((max(abs(a - b) for a, b in zip(x.q, CANDLE)) for x in p.samples()),
                        default=0)
            self.r.add('S4', f'edge CSV {name}: rejected, non-zero exit, arm not moved',
                       rc not in (0, 124) and moved < 0.02,
                       f'exit={rc} excursion={moved:.3f} | {last_line(out)}')
        target = [0.5, 0.3, -1.2, 0.6, -0.4, 0.3, 0.0]
        path = self.write_csv('edge_columns_out_of_order.csv',
                              'time_sec,' + ','.join(reversed(JOINTS)) + '\n4.0,'
                              + ','.join(str(v) for v in reversed(target)) + '\n')
        p.clear()
        rc, out = ur5(['run', path, '--out', 'edge_columns_out_of_order'], self.env, self.cwd, 120,
                      self.log)
        miss = min((max(abs(a - b) for a, b in zip(x.q, target)) for x in p.samples()),
                   default=math.inf)
        self.r.add('S4o', 'edge CSV columns out of order: resolved by header name, target reached',
                   rc == 0 and miss <= GOAL_TOL, f'exit={rc} closest approach={miss:.4f} | {last_line(out)}')
        self.pid_guard('edge CSVs')

    def stage_planner(self, p) -> None:
        out_csv = os.path.join(self.data_dir, 'waypoints', 'harness_plan.csv')
        rc, out = ur5(['plan', 'harness_plan', '--points', 'example.points.yaml'], self.env, self.cwd,
                      300, self.log)
        self.r.add('S12', 'ur5 plan --points example.points.yaml (no TTY) exports a joint CSV',
                   rc == 0 and os.path.isfile(out_csv), f'exit={rc} | {last_line(out)}')
        bad = self.write_csv('inside_table.points.yaml', 'points:\n  - [0.3, 0.0, 0.5]\n')
        rc, out = ur5(['plan', 'harness_bad', '--points', bad], self.env, self.cwd, 300, self.log)
        self.r.add('S12b', 'ur5 plan through the table: refused, non-zero exit, nothing exported',
                   rc not in (0, 124) and not os.path.exists(
                       os.path.join(self.data_dir, 'waypoints', 'harness_bad.csv')),
                   f'exit={rc} | {last_line(out)}')
        if os.path.isfile(out_csv):
            rc, out = ur5(['run', 'harness_plan.csv', '--out', 'harness_plan'], self.env, self.cwd,
                          300, self.log)
            self.r.add('S12c', 'ur5 run <planner export> exits 0', rc == 0,
                       f'exit={rc} | {last_line(out)}')
        self.pid_guard('planner')

    def stage_sigint(self, p) -> None:
        h5 = os.path.join(self.data_dir, 'telemetry', 'harness_sigint.h5')
        p.clear()
        bg = ur5_bg(['run', 'demo_15s.csv', '--out', 'harness_sigint'], self.env, self.cwd, self.log)
        moved = 0.0
        # Wall-clock give-up while waiting for the run to leave Candle.
        deadline = time.time() + 120
        while time.time() < deadline and moved < 0.1 and bg.poll() is None:
            s = p.latest()
            if s:
                moved = max(abs(a - b) for a, b in zip(s.q, CANDLE))
            time.sleep(0.1)
        with contextlib.suppress(ProcessLookupError):
            os.killpg(bg.pid, signal.SIGINT)
        try:
            rc = bg.wait(timeout=60)
        except subprocess.TimeoutExpired:
            os.killpg(bg.pid, signal.SIGKILL)
            rc = 124
        p.clear()
        p.wait_sim(2.0, 30)
        s = p.samples()
        drift = (max(max(abs(a - b) for a, b in zip(x.q, s[0].q)) for x in s) if s else math.inf)
        st = p.controllers()
        self.r.add('S11a', 'Ctrl-C during ur5 run: exit 130, arm held (drift < 0.005 over 2 s), A active',
                   rc == 130 and moved >= 0.1 and drift < HOLD_MAX_ERR and bool(st)
                   and st.get(JTC) == 'active',
                   f'exit={rc} excursion_at_sigint={moved:.3f} drift={drift:.4f} {st}')
        ok, msg = check_h5(h5) if os.path.isfile(h5) else (False, 'no HDF5 written')
        self.r.add('S11b', 'Ctrl-C during ur5 run: HDF5 closed and readable', ok, msg)
        rc, out = ur5(['reset'], self.env, self.cwd, 120, self.log)
        err = self.candle_err(p)
        self.r.add('S11c', 'ur5 reset after Ctrl-C: exit 0, at Candle', rc == 0 and err <= HOLD_MAX_ERR,
                   f'exit={rc} err={err:.4f}')
        self.pid_guard('sigint')

    def stage_handoff(self, p) -> None:
        before = p.controllers()
        res = p.switch([JTC, FEC], [])
        after = p.controllers()
        self.r.add('S7a', 'illegal double activation refused, states unchanged',
                   res is not None and not res.ok and after == before,
                   f'ok={getattr(res, "ok", None)} after={after}')
        bg = ur5_bg(['brain', 'b', '--sec', '4'], self.env, self.cwd, self.log)
        st = wait_controllers(p, {FEC: 'active', JTC: 'inactive'}, 60)
        excl = st and st.get(FEC) == 'active' and st.get(JTC) == 'inactive'
        self.r.add('S7b', 'ur5 brain b --sec 4: FEC active, JTC inactive during the hold', bool(excl),
                   f'{st}')
        p.clear()
        p.wait_sim(1.0, 30)
        s = p.samples()
        drift = max((max(abs(a - b) for a, b in zip(x.q, CANDLE)) for x in s), default=math.inf)
        self.r.add('S7c', 'Brain B holds Candle for 1 s after A->B (no collapse)', drift < HOLD_MAX_ERR,
                   f'max|q-candle|={drift:.4f} rad over 1 s sim after handoff')
        try:
            rc = bg.wait(timeout=60)
        except subprocess.TimeoutExpired:
            os.killpg(bg.pid, signal.SIGINT)
            rc = 124
        st = p.controllers()
        excl = st and st.get(JTC) == 'active' and st.get(FEC) == 'inactive'
        self.r.add('S7d', 'brain b hold ends: exit 0, authority back to A (JTC active, FEC inactive)',
                   rc == 0 and bool(excl), f'exit={rc} {st}')
        rc, out = ur5(['brain', 'a'], self.env, self.cwd, 60, self.log)
        st = p.controllers()
        excl = st and st.get(JTC) == 'active' and st.get(FEC) == 'inactive'
        self.r.add('S7f', 'ur5 brain a when A already holds: exit 0, states unchanged', rc == 0 and bool(excl),
                   f'exit={rc} {st}')
        rc, out = ur5(['reset'], self.env, self.cwd, 120, self.log)
        err = self.candle_err(p)
        self.r.add('S4r', 'ur5 reset from post-handoff pose: exit 0 and at Candle <= 0.005',
                   rc == 0 and err <= HOLD_MAX_ERR, f'exit={rc} err={err:.4f} | {last_line(out)}')
        self.settle_hold(p, 'after A->B->A + reset', 'S7e', 5.0)
        self.pid_guard('handoff')

    def candle_err(self, p) -> float:
        s = p.latest()
        return max(abs(a - b) for a, b in zip(s.q, CANDLE)) if s else math.inf

    def stage_reset_negative(self, p) -> None:
        bg = ur5_bg(['brain', 'b'], self.env, self.cwd, self.log)
        held = wait_controllers(p, {FEC: 'active', JTC: 'inactive'}, 60)
        rc, out = ur5(['reset'], self.env, self.cwd, 120, self.log)
        st = p.controllers()
        err = self.candle_err(p)
        ok_state = st and st.get(JTC) == 'active' and st.get(FEC) == 'inactive'
        self.r.add('S4f', 'ur5 reset while Brain B holds: takes authority to A and reaches Candle',
                   bool(held) and rc == 0 and bool(ok_state) and err <= HOLD_MAX_ERR,
                   f'b_active_before={bool(held)} exit={rc} states={st} err={err:.4f}')
        with contextlib.suppress(ProcessLookupError):
            os.killpg(bg.pid, signal.SIGINT)
        try:
            brc = bg.wait(timeout=30)
        except subprocess.TimeoutExpired:
            os.killpg(bg.pid, signal.SIGKILL)
            brc = 124
        st = p.controllers()
        self.r.add('S4g', 'Brain B process exits after losing authority; A keeps it',
                   brc != 124 and bool(st) and st.get(JTC) == 'active' and st.get(FEC) == 'inactive',
                   f'exit={brc} {st}')
        if not ok_state:
            ur5(['brain', 'a'], self.env, self.cwd, 60, self.log)
        rc1, _ = ur5(['reset'], self.env, self.cwd, 120, self.log)
        e1 = self.candle_err(p)
        rc2, _ = ur5(['reset'], self.env, self.cwd, 120, self.log)
        e2 = self.candle_err(p)
        self.r.add('S4i', 'ur5 reset twice is idempotent (exit 0, at Candle both times)',
                   rc1 == 0 and rc2 == 0 and max(e1, e2) <= HOLD_MAX_ERR,
                   f'exit={rc1},{rc2} err={e1:.4f},{e2:.4f}')
        self.settle_hold(p, 'after resets', 'S4h', 5.0)
        self.pid_guard('reset edge cases')

    def stage_torque(self, p) -> None:
        exp_dir = os.path.join(REPO, 'ur5_controller', 'config', 'experiments')
        slice_yaml = os.path.join(exp_dir, 'slice_s10.yaml')
        if not os.path.exists(slice_yaml):
            self.r.add('S8', 'Brain B loop >= 99 Hz vs /clock', False, 'no experiment config')
            self.r.add('S10', 'benchmark slice unattended on one Gazebo PID', False,
                       'no experiment config')
            return
        t_mark = time.time()
        rc, out = ur5(['torque', slice_yaml], self.env, self.cwd, self.args.torque_timeout, self.log)
        h5 = find_new_h5([self.data_dir], t_mark)
        self.r.add('S10', 'benchmark slice unattended on one Gazebo PID',
                   rc == 0 and len(h5) >= 8 and self.pid_guard('torque slice'),
                   f'exit={rc} trials={len(h5)} | {last_line(out)}')
        rates = []
        try:
            import h5py
            for path in h5:
                with h5py.File(path, 'r') as f:
                    rates.append(float(f.attrs.get('loop_rate_hz', 0.0)))
        except Exception as e:  # noqa: BLE001
            self.log.write(f'rate read failed: {e}\n')
        self.r.add('S8', 'Brain B loop >= 99 Hz vs /clock (every trial)',
                   bool(rates) and min(rates) >= 99.0,
                   f'min={min(rates) if rates else None} rates={[round(x, 1) for x in rates]}')
        for path in h5[:2]:
            v = subprocess.run(['ros2', 'run', 'ur5_controller', 'validate_h5.py', path],
                               env=self.env, capture_output=True, text=True, timeout=120)
            self.r.add('S9t', f'validate_h5.py {os.path.basename(path)}', v.returncode == 0,
                       last_line(v.stdout + v.stderr))
        self.settle_hold(p, 'after torque slice', 'S3t', 5.0)

    def stage_cli(self) -> None:
        rc, out = ur5(['help'], self.env, self.cwd, 30, self.log)
        self.r.add('S15a', 'ur5 help exits 0', rc == 0, f'exit={rc}')
        forced = [(['brain', 'x'], 'brain with bad arg'), (['run', '/nonexistent.csv'], 'run missing csv'),
                  (['plan'], 'plan without name'), (['nosuchcommand'], 'unknown command'),
                  (['validate', '/nonexistent.h5'], 'validate missing file')]
        for args, label in forced:
            rc, out = ur5(args, self.env, self.cwd, 60, self.log)
            one_line = len([ln for ln in out.strip().splitlines() if ln.strip() and
                            not ln.startswith('+ ')]) <= 2
            self.r.add('S15', f'forced failure exits non-zero: {label}', rc != 0 and one_line,
                       f'exit={rc} | {last_line(out)}')

    def stage_leaks(self) -> None:
        lines = leaked_operator_processes()
        self.r.add('S11', 'no leaked operator processes after commands', not lines,
                   '; '.join(lines)[:160] or 'none')
        inst = subprocess.run(['bash', '-c', 'ros2 pkg prefix ur5_controller'], env=self.env,
                              capture_output=True, text=True).stdout.strip()
        newer = []
        if inst:
            for root, _, files in os.walk(os.path.join(inst, 'share', 'ur5_controller')):
                for fn in files:
                    fp = os.path.join(root, fn)
                    if not os.path.islink(fp) and os.path.getmtime(fp) >= self.t_start:
                        newer.append(fp)
        self.r.add('S13b', 'nothing written under install/ during the session', not newer,
                   f'{newer[:3]}')
        cwd_files = os.listdir(self.cwd)
        self.r.add('S13c', 'nothing written into $CWD', not cwd_files, f'{cwd_files[:5]}')

    def run(self) -> int:
        lock = open(LOCK_PATH, 'w')
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            print(f'{LOCK_PATH} is held by another sim session', file=sys.stderr)
            return 2
        try:
            if not self.args.skip_static:
                self.stage_static()
            if self.args.static_only:
                return self.finish()
            with ProbeRunner() as p:
                self.start_sim()
                if self.stage_bringup(p):
                    self.stage_clock(p)
                    self.settle_hold(p, 'at bring-up', 'S3', self.args.hold_sec)
                    self.pid_guard('bring-up hold')
                    self.stage_collision()
                    for stage in (self.stage_joint_run, self.stage_cartesian_run,
                                  self.stage_edge_csv, self.stage_planner, self.stage_sigint,
                                  self.stage_handoff,
                                  self.stage_reset_negative, self.stage_torque):
                        try:
                            stage(p)
                        except Exception as e:  # noqa: BLE001
                            self.r.add('ERR', stage.__name__, False, repr(e)[:160])
                    self.stage_cli()
                    self.r.add('PIDf', 'one Gazebo PID for the whole session',
                               gz_pids() == self.pids, f'{self.pids} -> {gz_pids()}')
            self.stop_sim()
            self.stage_leaks()
        finally:
            self.stop_sim()
            fcntl.flock(lock, fcntl.LOCK_UN)
        return self.finish()

    def finish(self) -> int:
        sha = subprocess.run(['git', 'rev-parse', '--short', 'HEAD'], cwd=REPO,
                             capture_output=True, text=True).stdout.strip()
        dirty = bool(subprocess.run(['git', 'status', '--porcelain'], cwd=REPO,
                                    capture_output=True, text=True).stdout.strip())
        header = (f'- commit: {sha}{" (dirty)" if dirty else ""}\n- host: {os.uname().machine}\n'
                  f'- hold window: {self.args.hold_sec}s sim\n- artifacts: {self.outdir}')
        self.r.write(self.report_path, header)
        print(f'\nReport: {self.report_path}')
        return 1 if self.r.failed() else 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--hold-sec', type=float, default=120.0, help='Candle hold window (sim s)')
    ap.add_argument('--post-bringup', type=float, default=15.0,
                    help='sim seconds to wait after controllers are up before measuring')
    ap.add_argument('--bringup-timeout', type=float, default=180.0)
    ap.add_argument('--torque-timeout', type=float, default=1800.0)
    ap.add_argument('--skip-static', action='store_true')
    ap.add_argument('--static-only', action='store_true')
    args = ap.parse_args()
    return Harness(args).run()


if __name__ == '__main__':
    sys.exit(main())
