"""Joint names, Candle pose, limits and live state, all resolved by name."""
from __future__ import annotations

import math
import os
import threading
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field

import yaml


def inject_candle_urdf(urdf_xml: str, values: dict[str, float]) -> str:
    """Replace __CANDLE_<joint>__ placeholders with the SRDF candle_pose numbers."""
    out = urdf_xml
    for name, value in values.items():
        token = f'__CANDLE_{name}__'
        if token not in out:
            raise KeyError(f'URDF missing {token}')
        out = out.replace(token, f'{float(value):.10g}')
    if '__CANDLE_' in out:
        raise KeyError('URDF has a candle placeholder with no SRDF value')
    return out


def _share(package: str) -> str:
    from ament_index_python.packages import get_package_share_directory
    return get_package_share_directory(package)


def _source_root() -> str:
    """Repository root when running from the source tree (tests, symlink installs)."""
    return os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))


def resolve_file(package: str, relpath: str) -> str:
    try:
        path = os.path.join(_share(package), relpath)
        if os.path.exists(path):
            return path
    except Exception:  # noqa: BLE001
        pass
    path = os.path.join(_source_root(), package, relpath)
    if os.path.exists(path):
        return path
    raise FileNotFoundError(f'{package}/{relpath} not found (is the workspace sourced?)')


def load_platform(path: str | None = None) -> dict:
    path = path or os.environ.get('UR5_PLATFORM_CONFIG') or resolve_file(
        'ur5_controller', 'config/platform.yaml')
    with open(path) as f:
        return yaml.safe_load(f)


@dataclass
class JointLimit:
    lower: float
    upper: float
    velocity: float
    effort: float
    acceleration: float = 0.0


@dataclass
class Model:
    joints: tuple[str, ...]
    limits: dict[str, JointLimit]
    initial: dict[str, float]
    candle: tuple[float, ...]
    urdf_path: str
    urdf_xml: str = field(repr=False)

    def index(self, name: str) -> int:
        return self.joints.index(name)


def chain_joints(urdf_root: ET.Element, base: str, tip: str) -> list[ET.Element]:
    """Movable joints from base to tip, in kinematic order."""
    by_child = {j.find('child').get('link'): j for j in urdf_root.findall('joint')}
    chain = []
    link = tip
    while link != base:
        j = by_child.get(link)
        if j is None:
            raise ValueError(f'no path from {base} to {tip}')
        chain.append(j)
        link = j.find('parent').get('link')
    return [j for j in reversed(chain) if j.get('type') in ('revolute', 'continuous', 'prismatic')]


def srdf_state(srdf_path: str, state: str) -> dict[str, float]:
    root = ET.parse(srdf_path).getroot()
    for gs in root.findall('group_state'):
        if gs.get('name') == state:
            return {j.get('name'): float(j.get('value')) for j in gs.findall('joint')}
    raise KeyError(f'group_state {state!r} not in {srdf_path}')


def load_model(cfg: dict | None = None) -> Model:
    cfg = cfg or load_platform()
    m = cfg['model']
    urdf_path = resolve_file(m['urdf_package'], m['urdf_file'])
    with open(urdf_path) as f:
        xml = f.read()
    root = ET.fromstring(xml)
    chain = chain_joints(root, m['base_link'], m['tip_link'])
    joints = tuple(j.get('name') for j in chain)
    limits = {}
    for j in chain:
        lim = j.find('limit')
        limits[j.get('name')] = JointLimit(float(lim.get('lower')), float(lim.get('upper')),
                                           float(lim.get('velocity')), float(lim.get('effort')),
                                           float(lim.get('acceleration', 0.0)))
    mv = cfg['moveit']
    state = srdf_state(resolve_file(mv['srdf_package'], mv['srdf_file']), mv['candle_state'])
    missing = [j for j in joints if j not in state]
    if missing:
        raise KeyError(f'candle_pose missing joints {missing}')
    xml = inject_candle_urdf(xml, {j: state[j] for j in joints})
    root = ET.fromstring(xml)
    initial = {}
    for rj in root.iter('joint'):
        if rj.find('state_interface') is None:
            continue
        for si in rj.findall('state_interface'):
            if si.get('name') == 'position':
                p = si.find("param[@name='initial_value']")
                if p is not None and p.text:
                    initial[rj.get('name')] = float(p.text)
    return Model(joints, limits, initial, tuple(state[j] for j in joints), urdf_path, xml)


def reorder(names, values, target) -> list[float]:
    """Map values keyed by `names` onto `target` order; KeyError names the missing joint."""
    idx = {n: i for i, n in enumerate(names)}
    return [float(values[idx[t]]) for t in target]


def max_abs_diff(a, b) -> float:
    return max(abs(x - y) for x, y in zip(a, b))


@dataclass
class StateSample:
    t: float
    t_source: str
    clock_t: float
    q: list[float]
    qd: list[float]
    tau: list[float]


class StateCache:
    """Latest joint state in controller order, plus an optional recording window.

    With rate_hz set, samples are decimated on their header stamps: the first sample at or after
    each 1/rate_hz sim-time boundary is kept, so a 1 kHz physics-stamped stream yields an exact grid.
    """

    def __init__(self, node, joints, topic: str = '/joint_states', rate_hz: float | None = None) -> None:
        from sensor_msgs.msg import JointState
        self.node = node
        self.joints = tuple(joints)
        self._lock = threading.Condition()
        self._latest: StateSample | None = None
        self._seq = 0
        self._rec: list[StateSample] | None = None
        self._listeners = []
        self._period = 1.0 / rate_hz if rate_hz else 0.0
        self._next_t = 0.0
        node.create_subscription(JointState, topic, self._on_js, 50)

    @property
    def _clock_t(self) -> float | None:
        """Node clock (sim time via use_sim_time); None until /clock has been received."""
        ns = self.node.get_clock().now().nanoseconds
        return ns * 1e-9 if ns > 0 else None

    def _on_js(self, msg) -> None:
        hdr = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._period and hdr > 0.0:
            if hdr < self._next_t - 1e-6:
                return
            self._next_t = (math.floor(hdr / self._period + 1e-6) + 1) * self._period
        try:
            q = reorder(msg.name, msg.position, self.joints)
            qd = reorder(msg.name, msg.velocity, self.joints) if msg.velocity else [math.nan] * 7
            tau = reorder(msg.name, msg.effort, self.joints) if msg.effort else [math.nan] * 7
        except (KeyError, IndexError):
            return
        clk = self._clock_t if self._clock_t is not None else math.nan
        if hdr > 0.0:
            t, src = hdr, 'joint_state_header'
        else:
            t, src = clk, 'clock_at_receipt'
        s = StateSample(t, src, clk, q, qd, tau)
        with self._lock:
            self._latest = s
            self._seq += 1
            if self._rec is not None:
                self._rec.append(s)
            self._lock.notify_all()
        for cb in self._listeners:
            cb(s)

    def add_listener(self, cb) -> None:
        self._listeners = self._listeners + [cb]

    def remove_listener(self, cb) -> None:
        self._listeners = [x for x in self._listeners if x is not cb]

    def latest(self) -> StateSample | None:
        with self._lock:
            return self._latest

    def clock(self) -> float | None:
        return self._clock_t

    def seq(self) -> int:
        with self._lock:
            return self._seq

    def wait_for_state(self, timeout: float) -> StateSample | None:
        # Wall-clock: give-up timeout only.
        deadline = time.monotonic() + timeout
        with self._lock:
            while self._latest is None:
                rem = deadline - time.monotonic()
                if rem <= 0:
                    return None
                self._lock.wait(rem)
            return self._latest

    def wait_next(self, after_seq: int, timeout: float) -> tuple[int, StateSample | None]:
        # Wall-clock: give-up timeout only.
        deadline = time.monotonic() + timeout
        with self._lock:
            while self._seq <= after_seq:
                rem = deadline - time.monotonic()
                if rem <= 0:
                    return self._seq, None
                self._lock.wait(rem)
            return self._seq, self._latest

    def wait_sim(self, seconds: float, timeout: float) -> bool:
        """Block until `seconds` of sim time pass, measured on /joint_states stamps.

        Returns False on wall-clock give-up (sim paused or not publishing).
        """
        deadline = time.monotonic() + timeout
        with self._lock:
            while self._latest is None:
                rem = deadline - time.monotonic()
                if rem <= 0:
                    return False
                self._lock.wait(rem)
            start = self._latest.t
            while self._latest.t - start < seconds:
                rem = deadline - time.monotonic()
                if rem <= 0:
                    return False
                self._lock.wait(min(rem, 0.5))
            return True

    def start_recording(self) -> None:
        with self._lock:
            self._rec = []

    def stop_recording(self) -> list[StateSample]:
        with self._lock:
            rec, self._rec = self._rec or [], None
            return rec


def call_service(client, request, timeout: float):
    """Blocking call from a worker thread while the executor spins elsewhere. None on timeout."""
    if not client.wait_for_service(timeout_sec=min(timeout, 10.0)):
        return None
    done = threading.Event()
    fut = client.call_async(request)
    fut.add_done_callback(lambda _: done.set())
    # Wall-clock: give-up timeout only.
    if not done.wait(timeout):
        fut.cancel()
        return None
    return fut.result()


def state_cache(node, model, cfg: dict) -> StateCache:
    t = cfg['topics']
    return StateCache(node, model.joints, t['joint_states'], t.get('joint_states_rate_hz'))


def wait_future(fut, timeout: float):
    done = threading.Event()
    fut.add_done_callback(lambda _: done.set())
    # Wall-clock: give-up timeout only.
    if not done.wait(timeout):
        return None
    return fut.result()

EXIT_OK = 0
EXIT_RUN_FAILED = 1
EXIT_INPUT = 2
EXIT_PLATFORM = 3
EXIT_SAFETY = 4
EXIT_INTERRUPTED = 130


@dataclass
class RunResult:
    ok: bool
    code: int
    reason: str
    outputs: list[str] = field(default_factory=list)
    # (path, telemetry.Recorder, metadata) for the export stage; None when already written.
    export: tuple | None = None

    @classmethod
    def success(cls, reason: str = 'ok', outputs=None) -> 'RunResult':
        return cls(True, EXIT_OK, reason, list(outputs or []))

    @classmethod
    def fail(cls, code: int, reason: str, outputs=None) -> 'RunResult':
        return cls(False, code, reason, list(outputs or []))


class PlatformError(RuntimeError):
    """Platform not ready or a control-authority post-condition failed."""

    code = EXIT_PLATFORM


class InputError(ValueError):
    """Operator input (CSV, YAML, name) rejected before any motion."""

    code = EXIT_INPUT


class Scheme:
    def preflight(self) -> RunResult:
        raise NotImplementedError

    def run(self, spec) -> RunResult:
        raise NotImplementedError

    def abort(self) -> None:
        raise NotImplementedError

import csv
import hashlib
import math
import os
import re
from dataclasses import dataclass


CARTESIAN_HEADER = ('time_sec', 'x', 'y', 'z', 'roll', 'pitch', 'yaw')
NAME_RE = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$')


def source_dir_marker() -> str | None:
    """share/ur5_controller/source_dir, written by CMake from the package source tree."""
    try:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory('ur5_controller'), 'source_dir')
    except Exception:  # noqa: BLE001
        return None
    return path if os.path.isfile(path) else None


def package_source_dir() -> str:
    """Package source directory. Installed builds read the CMake marker; a checkout uses this file."""
    marker = source_dir_marker()
    if marker:
        text = open(marker).read().strip()
        if text:
            return text
    return os.path.dirname(os.path.dirname(os.path.realpath(__file__)))


def data_dir() -> str:
    """$UR5_DATA_DIR if set, otherwise <package source>/src/data. Never install/ or $CWD."""
    env = os.environ.get('UR5_DATA_DIR')
    if env:
        return os.path.abspath(os.path.expanduser(env))
    return os.path.join(package_source_dir(), 'src', 'data')


def ensure_writable(path: str) -> str:
    try:
        os.makedirs(path, exist_ok=True)
    except OSError as e:
        raise InputError(f'cannot create {path}: {e.strerror}; set UR5_DATA_DIR to a writable directory')
    if not os.access(path, os.W_OK):
        raise InputError(f'{path} is not writable; set UR5_DATA_DIR to a writable directory')
    return path


def check_name(name: str) -> str:
    if not NAME_RE.match(name or '') or '..' in name:
        raise InputError(f'invalid name {name!r}: use letters, digits, _ . - (no spaces, no /)')
    return name


def shipped_waypoints_dir() -> str:
    candidates = []
    try:
        candidates.append(resolve_file('ur5_controller', 'waypoints'))
    except FileNotFoundError:
        pass
    candidates.append(os.path.join(package_source_dir(), 'src', 'waypoints'))
    for directory in candidates:
        if os.path.isfile(os.path.join(directory, 'default_joint.csv')):
            return directory
    raise FileNotFoundError('shipped waypoints not found')


def resolve_waypoints(arg: str) -> str:
    """Existing path as given, else $UR5_DATA_DIR/waypoints/<arg>, else the shipped set."""
    if os.path.isfile(arg):
        return os.path.abspath(arg)
    if os.sep not in arg:
        for d in (os.path.join(data_dir(), 'waypoints'), shipped_waypoints_dir()):
            p = os.path.join(d, arg)
            if os.path.isfile(p):
                return p
    raise InputError(f'waypoint file not found: {arg} (ur5 wp lists available files)')


def sha256(path: str) -> str:
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        h.update(f.read())
    return h.hexdigest()


@dataclass
class Waypoints:
    path: str
    kind: str
    times: list[float]
    values: list[list[float]]


def _rows(path: str) -> list[list[str]]:
    try:
        with open(path, newline='') as f:
            rows = [r for r in csv.reader(f) if r and any(c.strip() for c in r)]
    except OSError as e:
        raise InputError(f'cannot read {path}: {e.strerror}')
    if not rows:
        raise InputError(f'{os.path.basename(path)} is empty')
    if len(rows) < 2:
        raise InputError(f'{os.path.basename(path)} has a header but no waypoints')
    return rows


def _floats(row: list[str], n: int, line: int, fname: str) -> list[float]:
    if len(row) != n:
        raise InputError(f'{fname} line {line}: {len(row)} columns, expected {n}')
    try:
        vals = [float(c) for c in row]
    except ValueError:
        raise InputError(f'{fname} line {line}: non-numeric value in {row}')
    if not all(math.isfinite(v) for v in vals):
        raise InputError(f'{fname} line {line}: NaN or inf in {row}')
    return vals


def _check_times(times: list[float], fname: str) -> None:
    if times[0] <= 0.0:
        raise InputError(f'{fname}: first time_sec must be > 0 (got {times[0]})')
    for i in range(1, len(times)):
        if times[i] <= times[i - 1]:
            raise InputError(f'{fname} line {i + 2}: time_sec {times[i]} is not after {times[i - 1]}')


def detect_kind(path: str) -> str:
    rows = _rows(path)
    hdr = tuple(h.strip() for h in rows[0])
    return 'cartesian' if hdr == CARTESIAN_HEADER else 'joint'


def load_joint_csv(path: str, model: Model, start=None) -> Waypoints:
    """Seven named joint columns in any order; checks limits and velocity feasibility from `start`."""
    fname = os.path.basename(path)
    rows = _rows(path)
    hdr = [h.strip() for h in rows[0]]
    if hdr[0] != 'time_sec':
        raise InputError(f'{fname}: first column must be time_sec, got {hdr[0]!r}')
    names = hdr[1:]
    if sorted(names) != sorted(model.joints) or len(set(names)) != len(names):
        raise InputError(f'{fname}: header must name exactly the 7 joints {list(model.joints)}')
    times, values = [], []
    for i, r in enumerate(rows[1:], start=2):
        v = _floats(r, len(hdr), i, fname)
        times.append(v[0])
        by_name = dict(zip(names, v[1:]))
        q = [by_name[j] for j in model.joints]
        for j, val in zip(model.joints, q):
            lim = model.limits[j]
            if not lim.lower <= val <= lim.upper:
                raise InputError(f'{fname} line {i}: {j}={val} outside [{lim.lower}, {lim.upper}]')
        values.append(q)
    _check_times(times, fname)
    check_velocity(times, values, model, start, fname)
    return Waypoints(path, 'joint', times, values)


def check_velocity(times, values, model: Model, start, fname: str) -> None:
    """Zero-velocity cubic segments peak at 1.5*dq/dt in velocity and 6*dq/dt^2 in acceleration."""
    prev_t, prev_q = (0.0, start) if start is not None else (times[0], values[0])
    for k, (t, q) in enumerate(zip(times, values)):
        if prev_q is not None and t > prev_t:
            dt = t - prev_t
            for j, a, b in zip(model.joints, prev_q, q):
                lim = model.limits[j]
                vel = 1.5 * abs(b - a) / dt
                acc = 6.0 * abs(b - a) / (dt * dt)
                if vel > lim.velocity:
                    raise InputError(f'{fname} waypoint {k + 1}: {j} needs {vel:.2f} rad/s, '
                                     f'limit {lim.velocity}; increase time_sec')
                if lim.acceleration and acc > lim.acceleration:
                    raise InputError(f'{fname} waypoint {k + 1}: {j} needs {acc:.2f} rad/s^2, '
                                     f'limit {lim.acceleration}; increase time_sec')
        prev_t, prev_q = t, q


def load_cartesian_csv(path: str) -> Waypoints:
    fname = os.path.basename(path)
    rows = _rows(path)
    hdr = tuple(h.strip() for h in rows[0])
    if hdr != CARTESIAN_HEADER:
        raise InputError(f'{fname}: header must be {",".join(CARTESIAN_HEADER)}')
    times, values = [], []
    for i, r in enumerate(rows[1:], start=2):
        v = _floats(r, 7, i, fname)
        times.append(v[0])
        values.append(v[1:])
    _check_times(times, fname)
    return Waypoints(path, 'cartesian', times, values)


def rpy_to_quat(roll: float, pitch: float, yaw: float):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    q = (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
         cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)
    n = math.sqrt(sum(x * x for x in q))
    return tuple(x / n for x in q)


def segment_samples(start, values, step: float):
    """Joint-space samples along the straight segments between zero-velocity waypoints."""
    prev = list(start)
    for q in values:
        n = max(1, int(math.ceil(max(abs(a - b) for a, b in zip(q, prev)) / step)))
        for k in range(1, n + 1):
            yield [p + (c - p) * k / n for p, c in zip(prev, q)]
        prev = list(q)


class MinJerkReference:
    """q_ref, qd_ref, qdd_ref through zero-velocity waypoints (quintic, rest-to-rest segments)."""

    def __init__(self, start, times, values, hold_sec: float = 0.0) -> None:
        self.knots_t = [0.0] + [t + hold_sec for t in times]
        self.knots_q = [list(start)] + [list(v) for v in values]
        self.hold = hold_sec
        self.duration = self.knots_t[-1]

    def __call__(self, t: float):
        n = len(self.knots_q[0])
        if t <= self.hold:
            return list(self.knots_q[0]), [0.0] * n, [0.0] * n
        if t >= self.duration:
            return list(self.knots_q[-1]), [0.0] * n, [0.0] * n
        for i in range(1, len(self.knots_t)):
            t0 = self.hold if i == 1 else self.knots_t[i - 1]
            t1 = self.knots_t[i]
            if t <= t1:
                break
        T = t1 - t0
        s = (t - t0) / T
        p = 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5
        dp = (30 * s ** 2 - 60 * s ** 3 + 30 * s ** 4) / T
        ddp = (60 * s - 180 * s ** 2 + 120 * s ** 3) / (T * T)
        a, b = self.knots_q[i - 1], self.knots_q[i]
        return ([x + (y - x) * p for x, y in zip(a, b)],
                [(y - x) * dp for x, y in zip(a, b)],
                [(y - x) * ddp for x, y in zip(a, b)])
