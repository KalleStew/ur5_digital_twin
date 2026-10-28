"""Waypoint CSV schemas, validation, file locations and reference interpolation."""
from __future__ import annotations

import csv
import hashlib
import math
import os
import re
from dataclasses import dataclass

from .robot import Model
from .robot import InputError

CARTESIAN_HEADER = ('time_sec', 'x', 'y', 'z', 'roll', 'pitch', 'yaw')
NAME_RE = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$')


def data_dir() -> str:
    """$UR5_DATA_DIR, else <workspace>/ur5_data. Never install/ or $CWD."""
    env = os.environ.get('UR5_DATA_DIR')
    if env:
        return os.path.abspath(os.path.expanduser(env))
    try:
        from ament_index_python.packages import get_package_prefix
        prefix = get_package_prefix('ur5_controller')
        install = os.path.dirname(prefix)
        if os.path.basename(install) == 'install':
            return os.path.join(os.path.dirname(install), 'ur5_data')
    except Exception:  # noqa: BLE001
        pass
    return os.path.join(os.path.expanduser('~'), 'ur5_data')


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
    from .robot import resolve_file
    return resolve_file('ur5_controller', 'waypoints')


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
