import os
import subprocess
import sys

import numpy as np
import pytest

from conftest import PKG
from ur5_controller import telemetry
from ur5_controller.scheme import InputError

JOINTS = tuple(f'j{i}' for i in range(7))
VALIDATE = os.path.join(PKG, 'scripts', 'validate_h5.py')


def meta(**kw):
    m = {k: 'x' for k in telemetry.REQUIRED_ATTRS}
    m.update(expected_rate_hz=100.0, seed=0, rtf=1.0, git_dirty=False)
    m.update(kw)
    m.pop('loop_rate_hz')
    m.pop('time_source')
    m.pop('joint_names')
    return m


def recording(n=500, t0=12.0, rate=100.0, drop=()):
    rec = telemetry.Recorder(JOINTS)
    for i in range(n):
        if i in drop:
            continue
        rec.append(t0 + i / rate, [0.0] * 7, [0.0] * 7, [1.0] * 7, [1.0] * 7, [0.0] * 7, [0.0] * 7)
    return rec


def validate(path):
    env = dict(os.environ, PYTHONPATH=PKG + os.pathsep + os.environ.get('PYTHONPATH', ''))
    return subprocess.run([sys.executable, VALIDATE, path], capture_output=True, text=True, env=env)


def test_write_and_validate(tmp_path):
    p = str(tmp_path / 'ok.h5')
    telemetry.write_h5(p, recording(), meta())
    assert not os.path.exists(p + '.part')
    r = validate(p)
    assert r.returncode == 0, r.stderr
    import h5py
    with h5py.File(p) as f:
        assert f['q'].shape == (500, 7)
        assert set(telemetry.REQUIRED_ATTRS) <= set(f.attrs)
        assert f.attrs['loop_rate_hz'] == pytest.approx(100.0)


@pytest.mark.parametrize('rec,why', [
    (lambda: recording(t0=0.0), 'not a sim time'),
    (lambda: recording(drop=range(100, 140)), 'gaps'),
    (lambda: recording(rate=90.0), 'outside'),
])
def test_validator_rejects(tmp_path, rec, why):
    p = str(tmp_path / 'bad.h5')
    telemetry.write_h5(p, rec(), meta())
    r = validate(p)
    assert r.returncode == 1 and why in r.stderr


def test_validator_missing_metadata(tmp_path):
    p = str(tmp_path / 'm.h5')
    m = meta()
    m.pop('seed')
    telemetry.write_h5(p, recording(), m)
    r = validate(p)
    assert r.returncode == 1 and 'seed' in r.stderr


@pytest.fixture(scope='module')
def clean_trace():
    """bench_reach reference at 100 Hz; measured qd = qd_ref + white noise, q lags q_ref."""
    from ur5_controller.robot import load_model, load_platform
    from ur5_controller.trajectory_io import MinJerkReference, load_joint_csv, resolve_waypoints
    model = load_model(load_platform())
    wps = load_joint_csv(resolve_waypoints('bench_reach.csv'), model, start=model.candle)
    ref = MinJerkReference(model.candle, wps.times, wps.values, hold_sec=1.0)
    samples = [ref(x) for x in np.arange(0.0, ref.duration + 1.0, 0.01)]
    q_ref = np.array([s[0] for s in samples])
    qd_ref = np.array([s[1] for s in samples])
    qd = qd_ref + np.random.default_rng(0).normal(0.0, 1e-4, qd_ref.shape)
    return q_ref - 0.05 * qd_ref, qd, q_ref, qd_ref


def _inject(qd, target):
    """Add a (-1)^k tone on joint 0, scaled by bisection so its nyquist_ratio equals target."""
    from ur5_controller.oscillation import nyquist_ratio
    tone = (-1.0) ** np.arange(qd.shape[0])
    lo, hi = 0.0, 1e-4
    while nyquist_ratio(qd + np.outer(tone, np.eye(7)[0]) * hi, 100.0)[0] < target:
        hi *= 2.0
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if nyquist_ratio(qd + np.outer(tone, np.eye(7)[0]) * mid, 100.0)[0] < target:
            lo = mid
        else:
            hi = mid
    return qd + np.outer(tone, np.eye(7)[0]) * hi


@pytest.mark.parametrize('target,valid', [(5.0, True), (8.0, True), (9.0, False), (14.0, False)])
def test_validator_nyquist_positive_control(tmp_path, clean_trace, target, valid):
    from ur5_controller.oscillation import nyquist_ratio
    q, qd, q_ref, qd_ref = clean_trace
    assert nyquist_ratio(qd, 100.0).max() < 5.0
    qd = _inject(qd, target)
    assert nyquist_ratio(qd, 100.0)[0] == pytest.approx(target, rel=1e-3)
    rec = telemetry.Recorder(JOINTS)
    for i in range(qd.shape[0]):
        rec.append(12.0 + i / 100.0, q[i].tolist(), qd[i].tolist(), [0.0] * 7, [0.0] * 7,
                   q_ref[i].tolist(), qd_ref[i].tolist())
    p = str(tmp_path / f'nyq{target:g}.h5')
    telemetry.write_h5(p, rec, meta(scheme='torque'))
    r = validate(p)
    if valid:
        assert r.returncode == 0, r.stderr
    else:
        assert r.returncode == 1 and 'sampling artefact' in r.stderr and 'j0' in r.stderr


def test_nyquist_artefact_metadata():
    from ur5_controller.oscillation import METHOD, nyquist_artefact
    assert nyquist_artefact(np.zeros((50, 7)), np.ones(7), 100.0) == {}
    qd = np.random.default_rng(1).normal(0.0, 1e-3, (400, 7))
    m = nyquist_artefact(qd, np.full(7, 0.01), 100.0)
    assert set(METHOD) <= set(m) and m['artefact_valid'] is True
    assert len(m['nyquist_ratio']) == 7 and len(m['artefact_fraction_of_rms']) == 7


def test_telemetry_path_refuses_overwrite(tmp_path, monkeypatch):
    monkeypatch.setenv('UR5_DATA_DIR', str(tmp_path))
    p = telemetry.telemetry_path('run1')
    assert p == str(tmp_path / 'telemetry' / 'run1.h5')
    open(p, 'w').close()
    with pytest.raises(InputError, match='exists'):
        telemetry.telemetry_path('run1')
    assert telemetry.telemetry_path('run1', force=True) == p
    with pytest.raises(InputError):
        telemetry.telemetry_path('../escape')
