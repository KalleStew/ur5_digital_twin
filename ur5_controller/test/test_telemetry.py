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
