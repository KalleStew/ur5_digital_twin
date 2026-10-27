"""Telemetry file check shared by validate_h5.py and the torque experiment runner.

Shapes, monotonic non-zero sim time, rate within tolerance, gaps, NaN/inf, required metadata,
and for torque trials the sampling-artefact criterion (oscillation.py).
"""
from __future__ import annotations

import numpy as np

from .oscillation import MIN_SAMPLES, NYQUIST_RATIO_MAX, nyquist_ratio
from .telemetry import DATASETS, REQUIRED_ATTRS

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
