#!/usr/bin/env python3
"""Validate UR5 telemetry: shapes, monotonic sim time, rate within 2 %, gaps, NaN/inf, metadata,
and for torque trials the sampling-artefact criterion.

  validate_h5.py <file.h5 | directory> [--tolerance 0.02]

A directory is searched recursively, skipping rejected/ (trials the runner already reran).
Exit 0 when every file passes; prints one line per file.
"""
import argparse
import glob
import os
import sys

from telemetry import RATE_TOLERANCE, check


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('path')
    ap.add_argument('--tolerance', type=float, default=RATE_TOLERANCE)
    a = ap.parse_args()
    if os.path.isdir(a.path):
        files = sorted(p for p in glob.glob(os.path.join(a.path, '**', '*.h5'), recursive=True)
                       if 'rejected' not in os.path.relpath(p, a.path).split(os.sep))
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
