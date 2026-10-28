import importlib.util
import os
import struct
import xml.etree.ElementTree as ET

import pytest

from conftest import REPO
import ur5_common as robot
from dynamics import build_chain
from ur5_common import chain_joints, inject_candle_urdf, load_model, load_platform, resolve_file, srdf_state

import PyKDL as kdl


@pytest.fixture(scope='module')
def model():
    return load_model(load_platform())


def test_joint_order_matches_urdf_chain(model):
    cfg = load_platform()
    root = ET.parse(model.urdf_path).getroot()
    names = tuple(j.get('name') for j in chain_joints(
        root, cfg['model']['base_link'], cfg['model']['tip_link']))
    assert model.joints == names


def test_candle_single_source(model):
    cfg = load_platform()
    srdf = resolve_file(cfg['moveit']['srdf_package'], cfg['moveit']['srdf_file'])
    raw = open(model.urdf_path).read()
    for joint in model.joints:
        assert f'<param name="initial_value">__CANDLE_{joint}__</param>' in raw
    assert [model.initial[j] for j in model.joints] == pytest.approx(list(model.candle))
    assert '__CANDLE_' not in model.urdf_xml
    states = [gs.get('name') for gs in ET.parse(srdf).getroot().findall('group_state')]
    assert states == ['candle_pose']
    pose = srdf_state(srdf, 'candle_pose')
    assert [pose[j] for j in model.joints] == pytest.approx(list(model.candle))


def test_launch_inject_matches_loader(model):
    path = os.path.join(REPO, 'ur5_description', 'launch', 'sim.launch.py')
    spec = importlib.util.spec_from_file_location('ur5_sim_launch_under_test', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    raw = open(model.urdf_path).read()
    values = {j: model.candle[i] for i, j in enumerate(model.joints)}
    assert mod.inject_candle_urdf(raw, values) == inject_candle_urdf(raw, values)
    injected = mod.inject_candle_urdf(raw, mod.candle_values(
        resolve_file('ur5_moveit_config', 'config/ur57_arm.srdf')))
    assert '__CANDLE_' not in injected


def test_controller_yaml_joint_order(model):
    import yaml
    path = resolve_file('ur5_moveit_config', 'config/ros2_controllers.yaml')
    with open(path) as f:
        y = yaml.safe_load(f)
    assert tuple(y['ur_manipulator_controller']['ros__parameters']['joints']) == model.joints
    assert tuple(y['forward_effort_controller']['ros__parameters']['joints']) == model.joints


def test_reorder_by_name(model):
    alpha = sorted(model.joints)
    vals = [float(i) for i in range(7)]
    out = robot.reorder(alpha, vals, model.joints)
    assert out == [vals[alpha.index(j)] for j in model.joints]
    with pytest.raises(KeyError):
        robot.reorder(alpha[:-1], vals[:-1], model.joints)


def test_limits_loaded(model):
    for j in model.joints:
        lim = model.limits[j]
        assert lim.lower < lim.upper and lim.velocity > 0 and lim.effort > 0


def _mesh_vertices(path):
    data = open(path, 'rb').read()
    n = struct.unpack_from('<I', data, 80)[0]
    need = 84 + n * 50
    if n <= 0 or need > len(data):
        raise AssertionError(f'{path} is not a binary STL ({len(data)} bytes)')
    verts = []
    off = 84
    for _ in range(n):
        for k in range(3):
            verts.append(struct.unpack_from('<fff', data, off + 12 + 12 * k))
        off += 50
    return verts


def _frame(origin):
    if origin is None:
        return kdl.Frame()
    xyz = [float(x) for x in (origin.get('xyz') or '0 0 0').split()]
    rpy = [float(x) for x in (origin.get('rpy') or '0 0 0').split()]
    return kdl.Frame(kdl.Rotation.RPY(*rpy), kdl.Vector(*xyz))


def _table_top(urdf_xml: str) -> float:
    """Top face of the workbench box. The box is centered on its collision origin."""
    root = ET.fromstring(urdf_xml)
    joint_z = None
    for joint in root.findall('joint'):
        if joint.get('name') == 'world_to_table':
            joint_z = float(joint.find('origin').get('xyz').split()[2])
    link = root.find("link[@name='table_link']")
    col = link.find('collision')
    origin = _frame(col.find('origin'))
    size_z = float(col.find('geometry').find('box').get('size').split()[2])
    top = kdl.Frame(kdl.Vector(0, 0, joint_z)) * origin * kdl.Vector(0, 0, size_z / 2)
    return top.z()


def test_candle_upright_and_clear_of_table(model):
    """tool0 is high, and every moving link's collision mesh clears the workbench."""
    chain, _ = build_chain(model.urdf_xml, 'world', 'tool0')
    fk = kdl.ChainFkSolverPos_recursive(chain)
    q = kdl.JntArray(chain.getNrOfJoints())
    for i, value in enumerate(model.candle):
        q[i] = value
    frame = kdl.Frame()
    fk.JntToCart(q, frame)
    table_top = _table_top(model.urdf_xml)
    assert frame.p.z() - table_top > 0.85

    root = ET.fromstring(model.urdf_xml)
    pkg = os.path.dirname(os.path.dirname(model.urdf_path))
    pose = dict(zip(model.joints, model.candle))
    offenders = []
    for link in root.findall('link'):
        name = link.get('name')
        if name in ('table_link', 'base_link'):
            continue
        col = link.find('collision')
        mesh = None if col is None or col.find('geometry') is None else col.find('geometry').find('mesh')
        if mesh is None:
            continue
        rel = mesh.get('filename').split('package://ur5_description/', 1)[-1]
        chain, names = build_chain(model.urdf_xml, 'world', name)
        solver = kdl.ChainFkSolverPos_recursive(chain)
        jq = kdl.JntArray(chain.getNrOfJoints())
        for i, joint in enumerate(names):
            jq[i] = pose[joint]
        link_frame = kdl.Frame()
        solver.JntToCart(jq, link_frame)
        origin = _frame(col.find('origin'))
        min_z = min((link_frame * origin * kdl.Vector(*v)).z() for v in _mesh_vertices(os.path.join(pkg, rel)))
        clearance = min_z - table_top
        if clearance <= 0.0:
            offenders.append(f'{name} clearance {clearance:.4f} m')
    assert offenders == [], 'candle_pose collides with the workbench: ' + '; '.join(offenders)
