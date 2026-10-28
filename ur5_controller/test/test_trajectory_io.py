import os

import pytest

from ur5_controller import trajectory_io as tio
from ur5_controller.robot import load_model, load_platform
from ur5_controller.scheme import InputError

_MODEL = load_model(load_platform())
J = _MODEL.joints
HDR = 'time_sec,' + ','.join(J) + '\n'


@pytest.fixture(scope='module')
def model():
    return _MODEL


def write(tmp_path, name, text):
    p = tmp_path / name
    p.write_text(text)
    return str(p)


@pytest.mark.parametrize('name,text,msg', [
    ('empty', '', 'empty'),
    ('header_only', HDR, 'no waypoints'),
    ('wrong_columns', HDR + '2.0,0,0,0.4,0,0\n', 'columns'),
    ('bad_header', 'time_sec,a,b,c,d,e,f,g\n2,0,0,0,0,0,0,0\n', 'exactly the 7 joints'),
    ('dup_header', 'time_sec,' + ','.join(J[:6] + (J[0],)) + '\n2,0,0,0.4,0,0,0,0\n', 'exactly'),
    ('non_monotonic', HDR + '4.0,0,0,0.4,0,0,0,0\n2.0,0,0,0.4,0,0,0,0\n', 'not after'),
    ('zero_first_time', HDR + '0.0,0,0,-1.4,0,0,0,0\n', 'must be > 0'),
    ('nan', HDR + '2.0,0,0,nan,0,0,0,0\n', 'NaN'),
    ('text', HDR + '2.0,0,0,abc,0,0,0,0\n', 'non-numeric'),
    ('beyond_limit', HDR + '3.0,0,0,0.4,3.5,0,0,0\n', 'outside'),
    ('too_fast', HDR + '0.2,0,0,1.5,0,0,0,0\n', 'rad/s'),
])
def test_joint_csv_rejections(tmp_path, model, name, text, msg):
    p = write(tmp_path, f'{name}.csv', text)
    with pytest.raises(InputError, match=msg):
        tio.load_joint_csv(p, model, start=model.candle)


def test_joint_csv_columns_any_order(tmp_path, model):
    rev = 'time_sec,' + ','.join(reversed(J)) + '\n'
    p = write(tmp_path, 'rev.csv', rev + '4.0,0.1,0,0,0,-1.2,0,0\n')
    w = tio.load_joint_csv(p, model, start=model.candle)
    assert w.values[0] == [0, 0, -1.2, 0, 0, 0, 0.1]


def test_cartesian_csv(tmp_path):
    p = write(tmp_path, 'c.csv', 'time_sec,x,y,z,roll,pitch,yaw\n4,0.1,0.2,1.5,0,0,0\n')
    w = tio.load_cartesian_csv(p)
    assert w.kind == 'cartesian' and w.values[0][:3] == [0.1, 0.2, 1.5]
    with pytest.raises(InputError, match='header'):
        tio.load_cartesian_csv(write(tmp_path, 'd.csv', 'time,x,y,z\n1,2,3,4\n'))


@pytest.mark.parametrize('bad', ['', 'a b', '../x', 'a/b', '-x', 'x' * 80])
def test_check_name_rejects(bad):
    with pytest.raises(InputError):
        tio.check_name(bad)


def test_data_dir_env(monkeypatch, tmp_path):
    monkeypatch.setenv('UR5_DATA_DIR', str(tmp_path))
    assert tio.data_dir() == str(tmp_path)


def test_ensure_writable_rejects(tmp_path):
    ro = tmp_path / 'ro'
    ro.mkdir()
    os.chmod(ro, 0o500)
    try:
        if os.access(ro, os.W_OK):
            pytest.skip('running as root')
        with pytest.raises(InputError, match='writable'):
            tio.ensure_writable(str(ro))
    finally:
        os.chmod(ro, 0o700)


WAYPOINTS = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'waypoints')
SHIPPED = sorted(f for f in os.listdir(WAYPOINTS) if f.endswith('.csv'))


def test_shipped_set_complete():
    assert {'default_joint.csv', 'default_cartesian.csv', 'demo_15s.csv'} <= set(SHIPPED)


@pytest.mark.parametrize('name', SHIPPED)
def test_shipped_csv_loads(model, name):
    path = os.path.join(WAYPOINTS, name)
    if tio.detect_kind(path) == 'cartesian':
        w = tio.load_cartesian_csv(path)
        assert all(v[2] > 1.1 for v in w.values), 'tool0 below the table-top clearance'
    else:
        w = tio.load_joint_csv(path, model, start=model.candle)
        assert any(max(abs(a - b) for a, b in zip(q, model.candle)) > 0.1 for q in w.values)


def test_example_points_file_loads():
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        'planner_cli', os.path.join(os.path.dirname(WAYPOINTS), 'scripts', 'planner_cli.py'))
    cli = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(cli)
    points, home = cli.load_points(os.path.join(WAYPOINTS, 'example.points.yaml'))
    assert len(points) >= 2 and home and all(len(p) in (3, 6) for p in points)


def test_min_jerk_reference_endpoints():
    ref = tio.MinJerkReference([0.0, 0.0], [2.0], [[1.0, -1.0]], hold_sec=1.0)
    q, qd, qdd = ref(0.5)
    assert q == [0.0, 0.0] and qd == [0.0, 0.0]
    q, qd, _ = ref(2.0)
    assert q == pytest.approx([0.5, -0.5]) and qd[0] > 0
    q, qd, _ = ref(3.5)
    assert q == [1.0, -1.0] and qd == [0.0, 0.0]
