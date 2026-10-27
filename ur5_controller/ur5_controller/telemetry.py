"""Raw HDF5 telemetry with run metadata. Filtering is post-processing, never done here."""
from __future__ import annotations

import datetime as dt
import json
import math
import os
import subprocess

import numpy as np

from .scheme import InputError
from .trajectory_io import check_name, data_dir, ensure_writable

DATASETS = ('t', 'q', 'qd', 'tau_measured', 'tau_commanded', 'q_ref', 'qd_ref')
REQUIRED_ATTRS = ('git_sha', 'git_dirty', 'scheme', 'controller', 'gains', 'seed', 'perturbation',
                  'perturbation_side', 'trajectory_file', 'trajectory_sha256', 'rtf',
                  'loop_rate_hz', 'expected_rate_hz', 'time_source', 'joint_names', 'result',
                  'created_utc')


def git_info() -> tuple[str, bool]:
    """Commit and dirty flag from $UR5_GIT_SHA/$UR5_GIT_DIRTY, else the source checkout."""
    sha = os.environ.get('UR5_GIT_SHA')
    if sha:
        return sha, os.environ.get('UR5_GIT_DIRTY', '1') == '1'
    here = os.path.dirname(os.path.realpath(__file__))
    try:
        sha = subprocess.run(['git', '-C', here, 'rev-parse', 'HEAD'], capture_output=True,
                             text=True, timeout=5, check=True).stdout.strip()
        dirty = bool(subprocess.run(['git', '-C', here, 'status', '--porcelain'],
                                    capture_output=True, text=True, timeout=5).stdout.strip())
        return sha, dirty
    except (OSError, subprocess.SubprocessError):
        return 'unknown', True


def telemetry_path(name: str, force: bool = False) -> str:
    check_name(name)
    d = ensure_writable(os.path.join(data_dir(), 'telemetry'))
    path = os.path.join(d, f'{name}.h5')
    if os.path.exists(path) and not force:
        raise InputError(f'{path} exists; pick another --out name or pass --force')
    return path


class Recorder:
    """Accumulates per-sample rows; every array is N x 7 in controller joint order."""

    def __init__(self, joints) -> None:
        self.joints = tuple(joints)
        self.rows: dict[str, list] = {k: [] for k in DATASETS}
        self.time_sources: set[str] = set()
        self.extra: dict[str, list] = {}

    def append(self, t, q, qd, tau_measured, tau_commanded=None, q_ref=None, qd_ref=None,
               t_source='joint_state_header', **extra) -> None:
        nan7 = [math.nan] * len(self.joints)
        self.rows['t'].append(t)
        self.rows['q'].append(q)
        self.rows['qd'].append(qd)
        self.rows['tau_measured'].append(tau_measured)
        self.rows['tau_commanded'].append(nan7 if tau_commanded is None else tau_commanded)
        self.rows['q_ref'].append(nan7 if q_ref is None else q_ref)
        self.rows['qd_ref'].append(nan7 if qd_ref is None else qd_ref)
        self.time_sources.add(t_source)
        for k, v in extra.items():
            self.extra.setdefault(k, []).append(v)

    def __len__(self) -> int:
        return len(self.rows['t'])

    def achieved_rate(self) -> float:
        t = self.rows['t']
        if len(t) < 2 or t[-1] <= t[0]:
            return 0.0
        return (len(t) - 1) / (t[-1] - t[0])


def write_h5(path: str, rec: Recorder, meta: dict) -> str:
    """Write atomically (temp file + rename) so an interrupted export never leaves a partial file."""
    import h5py
    sha, dirty = git_info()
    attrs = {
        'git_sha': sha, 'git_dirty': dirty,
        'joint_names': list(rec.joints),
        'time_source': ','.join(sorted(rec.time_sources)) or 'none',
        'loop_rate_hz': rec.achieved_rate(),
        'created_utc': dt.datetime.now(dt.timezone.utc).isoformat(timespec='seconds'),
        'ros_distro': os.environ.get('ROS_DISTRO', ''),
        'rmw_implementation': os.environ.get('RMW_IMPLEMENTATION', ''),
    }
    attrs.update(meta)
    tmp = path + '.part'
    with h5py.File(tmp, 'w') as f:
        f.create_dataset('t', data=np.asarray(rec.rows['t'], dtype=np.float64))
        for k in DATASETS[1:]:
            ds = f.create_dataset(k, data=np.asarray(rec.rows[k], dtype=np.float64).reshape(-1, len(rec.joints)))
            ds.attrs['joint_names'] = list(rec.joints)
        for k, v in rec.extra.items():
            f.create_dataset(k, data=np.asarray(v))
        for k, v in attrs.items():
            if isinstance(v, (dict, list, tuple)) and not (
                    isinstance(v, (list, tuple)) and all(isinstance(x, str) for x in v)):
                v = json.dumps(v)
            elif v is None:
                v = 'none'
            f.attrs[k] = v
    os.replace(tmp, path)
    return path
