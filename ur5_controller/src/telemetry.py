"""Raw HDF5 telemetry with run metadata. Filtering is post-processing, never done here."""
from __future__ import annotations

import datetime as dt
import json
import math
import os
import subprocess

import numpy as np

from ur5_common import InputError
from ur5_common import check_name, data_dir, ensure_writable

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

NYQUIST_RATIO_MAX = 8.5
NYQUIST_BAND = (0.96, 1.0)
REFERENCE_BAND = (0.7, 0.9)
MIN_SAMPLES = 200
METHOD = {
    'artefact_psd_method': 'periodogram |rfft(hann * (qd - mean))|^2',
    'artefact_window': 'hann',
    'artefact_nyquist_band_fraction': list(NYQUIST_BAND),
    'artefact_reference_band_fraction': list(REFERENCE_BAND),
    'artefact_nyquist_ratio_max': NYQUIST_RATIO_MAX,
}


def nyquist_ratio(qd, rate_hz: float) -> np.ndarray:
    x = np.asarray(qd, dtype=float)
    x = x - x.mean(axis=0)
    f_n = rate_hz / 2.0
    fr = np.fft.rfftfreq(x.shape[0], 1.0 / rate_hz)
    psd = np.abs(np.fft.rfft(x * np.hanning(x.shape[0])[:, None], axis=0)) ** 2
    top = psd[(fr >= NYQUIST_BAND[0] * f_n) & (fr <= NYQUIST_BAND[1] * f_n)].mean(axis=0)
    ref = psd[(fr >= REFERENCE_BAND[0] * f_n) & (fr <= REFERENCE_BAND[1] * f_n)].mean(axis=0)
    return np.divide(top, ref, out=np.zeros_like(top), where=ref > 0)


def nyquist_artefact(qd, rms_per_joint, rate_hz: float) -> dict:
    """Criterion and information metrics; empty when the trace is shorter than MIN_SAMPLES."""
    x = np.asarray(qd, dtype=float)
    if x.shape[0] < MIN_SAMPLES:
        return {}
    ratio = nyquist_ratio(x, rate_hz)
    f_n = rate_hz / 2.0
    fr = np.fft.rfftfreq(x.shape[0], 1.0 / rate_hz)
    spec = np.fft.rfft(x - x.mean(axis=0), axis=0)
    spec[fr < REFERENCE_BAND[1] * f_n] = 0.0
    amp = np.abs(np.fft.irfft(spec, n=x.shape[0], axis=0)).max(axis=0) / (2.0 * np.pi * f_n)
    rms = np.asarray(rms_per_joint, dtype=float)
    frac = np.divide(amp, rms, out=np.full_like(amp, np.inf), where=rms > 0)
    return {
        **METHOD,
        'nyquist_ratio': ratio.tolist(),
        'artefact_amplitude_rad': amp.tolist(),
        'artefact_fraction_of_rms': frac.tolist(),
        'artefact_valid': bool(np.all(ratio <= NYQUIST_RATIO_MAX)),
    }

ALWAYS_FINITE = ('t', 'q', 'qd', 'tau_measured')
RATE_TOLERANCE = 0.02


def check(path: str, tol: float = RATE_TOLERANCE) -> list[str]:
    """First entry is an info line starting with 'n=' when the file opens; the rest are problems."""
    import h5py
    problems = []
    try:
        f = h5py.File(path, 'r')
    except OSError as e:
        return [f'cannot open: {e}']
    with f:
        missing = [k for k in DATASETS if k not in f]
        if missing:
            return [f'missing datasets {missing}']
        t = np.asarray(f['t'])
        n = t.shape[0]
        names = list(f.attrs.get('joint_names', []))
        if len(names) != 7:
            problems.append(f'joint_names attribute has {len(names)} entries')
        for k in DATASETS[1:]:
            if f[k].shape != (n, 7):
                problems.append(f'{k} shape {f[k].shape}, expected ({n}, 7)')
        if n < 2:
            return problems + [f'{n} samples']
        if t[0] <= 0.0:
            problems.append(f'first timestamp {t[0]} is not a sim time')
        d = np.diff(t)
        if np.any(d <= 0):
            problems.append(f'{int(np.sum(d <= 0))} non-increasing timestamps')
        for k in ALWAYS_FINITE:
            a = np.asarray(f[k])
            if not np.all(np.isfinite(a)):
                problems.append(f'{k} has {int(np.sum(~np.isfinite(a)))} NaN/inf values')
        attrs = dict(f.attrs)
        absent = [k for k in REQUIRED_ATTRS if k not in attrs]
        if absent:
            problems.append(f'metadata missing {absent}')
        expected = float(attrs.get('expected_rate_hz', 0.0))
        span = float(t[-1] - t[0])
        rate = (n - 1) / span if span > 0 else 0.0
        if expected > 0:
            if abs(rate - expected) > tol * expected:
                problems.append(f'rate {rate:.2f} Hz outside {expected:.0f} Hz +/- {tol:.0%}')
            gaps = int(np.sum(d > 1.5 / expected))
            dropped = int(round(span * expected)) + 1 - n
            if gaps:
                problems.append(f'{gaps} gaps > 1.5 periods ({max(dropped, 0)} samples dropped)')
            if attrs.get('scheme') == 'torque' and n >= MIN_SAMPLES:
                ratio = nyquist_ratio(np.asarray(f['qd']), expected)
                if np.any(ratio > NYQUIST_RATIO_MAX):
                    j = int(np.argmax(ratio))
                    problems.append(f'sampling artefact: nyquist_ratio {ratio[j]:.2f} > {NYQUIST_RATIO_MAX} '
                                    f'on {names[j] if j < len(names) else j}')
                if 'artefact_valid' in attrs and bool(attrs['artefact_valid']) != bool(
                        np.all(ratio <= NYQUIST_RATIO_MAX)):
                    problems.append('stored artefact_valid disagrees with the recomputed nyquist_ratio')
        problems.insert(0, f'n={n} rate={rate:.2f}Hz time_source={attrs.get("time_source", "?")}')
    return problems


def rejection(path: str, tol: float = RATE_TOLERANCE) -> str:
    """'' when the file passes, otherwise the problems joined into one line."""
    res = check(path, tol)
    issues = res[1:] if res and res[0].startswith('n=') else res
    return '; '.join(issues)
