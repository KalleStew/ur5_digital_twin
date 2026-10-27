#!/usr/bin/env python3
"""Validate UR5 telemetry: shapes, monotonic sim time, rate within 2 %, gaps, NaN/inf, metadata.

  validate_h5.py <file.h5 | directory> [--tolerance 0.02]

Exit 0 when every file passes; prints one line per file.
"""
import argparse
import glob
import os
import sys

import numpy as np

from ur5_controller.telemetry import DATASETS, REQUIRED_ATTRS

ALWAYS_FINITE = ('t', 'q', 'qd', 'tau_measured')


def check(path: str, tol: float) -> list[str]:
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
        problems.insert(0, f'n={n} rate={rate:.2f}Hz time_source={attrs.get("time_source", "?")}')
    return problems


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('path')
    ap.add_argument('--tolerance', type=float, default=0.02)
    a = ap.parse_args()
    if os.path.isdir(a.path):
        files = sorted(glob.glob(os.path.join(a.path, '**', '*.h5'), recursive=True))
    elif os.path.isfile(a.path):
        files = [a.path]
    else:
        print(f'FAIL: {a.path} not found', file=sys.stderr)
        return 2
    if not files:
        print(f'FAIL: no .h5 files under {a.path}', file=sys.stderr)
        return 2
    bad = 0
    for p in files:
        res = check(p, a.tolerance)
        info, issues = (res[0], res[1:]) if res and res[0].startswith('n=') else ('', res)
        if issues:
            bad += 1
            print(f'FAIL: {p}: {info} {"; ".join(issues)}', file=sys.stderr)
        else:
            print(f'PASS: {p}: {info}')
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
