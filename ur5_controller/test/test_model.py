import xml.etree.ElementTree as ET

import numpy as np
import pytest

from ur5_controller import robot
from ur5_controller.robot import load_model, load_platform, resolve_file, srdf_state


@pytest.fixture(scope='module')
def model():
    return load_model(load_platform())


def test_joint_order_matches_urdf_chain(model):
    assert model.joints == robot.JOINTS


def test_candle_single_source(model):
    cfg = load_platform()
    srdf = resolve_file(cfg['moveit']['srdf_package'], cfg['moveit']['srdf_file'])
    assert model.candle == pytest.approx(robot.CANDLE)
    assert [model.initial[j] for j in model.joints] == pytest.approx(list(model.candle))
    ready = srdf_state(srdf, 'ready')
    assert [ready[j] for j in model.joints] == pytest.approx(list(model.candle))


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


def test_candle_upright_and_clear_of_table(model):
    """With the shipped URDF, candle_pose puts tool0 about 0.9 m above the table top."""
    from ur5_controller.dynamics import build_chain
    import PyKDL as kdl
    chain, _ = build_chain(model.urdf_xml, 'world', 'tool0')
    fk = kdl.ChainFkSolverPos_recursive(chain)
    q = kdl.JntArray(7)
    for i, v in enumerate(model.candle):
        q[i] = v
    f = kdl.Frame()
    fk.JntToCart(q, f)
    table_top = 0.99
    assert f.p.z() - table_top > 0.85
