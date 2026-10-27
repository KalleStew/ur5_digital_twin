import os

import numpy as np
import pytest

from ur5_controller.controllers.base import load_controller
from ur5_controller.dynamics import Dynamics, NominalModel
from ur5_controller.perturbations import KINDS, Perturbation
from ur5_controller.robot import load_model, load_platform
from ur5_controller.scheme import InputError

GAINS = {'kp': [300, 300, 600, 300, 50, 30, 10], 'kd': [10, 10, 40, 10, 0.5, 0.3, 0.1]}


@pytest.fixture(scope='module')
def model():
    return load_model(load_platform())


@pytest.fixture(scope='module')
def dyn(model):
    return Dynamics(model)


@pytest.mark.parametrize('plugin', ['pd_gravity', 'pd_id_ff', 'computed_torque'])
def test_model_based_plugins_hold_at_reference(plugin, model, dyn):
    c = load_controller(plugin, dyn, GAINS, model.joints)
    q = np.array(model.candle)
    z = np.zeros(7)
    tau = c.compute(0.0, q, z, q, z, z)
    assert np.allclose(tau, dyn.g(q), atol=1e-6)


def test_nominal_terms_from_urdf(dyn):
    n = NominalModel(dyn)
    assert n.terms == ('rigid_body', 'viscous', 'coulomb') and n.epsilon == 0.05
    assert np.allclose(n.b, dyn.damping) and np.allclose(n.fc, dyn.friction)
    qd = np.full(7, 0.05)
    assert np.allclose(n.joint_friction(qd), dyn.damping * 0.05 + dyn.friction * np.tanh(1.0))
    vis = NominalModel(dyn, ['rigid_body', 'viscous'], None)
    assert vis.epsilon is None and np.allclose(vis.joint_friction(qd), dyn.damping * 0.05)
    assert np.allclose(NominalModel(dyn, ['rigid_body']).joint_friction(qd), 0.0)
    d = n.describe()
    assert d['model_coulomb_epsilon'] == 0.05 and d['model_coulomb_fc'] == dyn.friction.tolist()
    assert d['viscous_velocity_source'] == d['coulomb_velocity_source'] == 'desired'
    assert vis.describe()['coulomb_velocity_source'] == 'none'


@pytest.mark.parametrize('terms,eps,msg', [
    (['viscous'], None, 'rigid_body'),
    (['rigid_body', 'stiction'], None, 'allowed'),
    (['rigid_body', 'coulomb'], None, 'coulomb_epsilon'),
    (['rigid_body', 'coulomb'], 0.0, 'coulomb_epsilon'),
])
def test_nominal_terms_rejected(dyn, terms, eps, msg):
    with pytest.raises(InputError, match=msg):
        NominalModel(dyn, terms, eps)


@pytest.mark.parametrize('plugin', ['computed_torque', 'pd_id_ff'])
def test_joint_terms_use_reference_velocity(model, dyn, plugin):
    """Viscous and Coulomb terms depend on qd_ref only, never on the measured velocity."""
    q = np.array(model.candle)
    z = np.zeros(7)
    qd = np.full(7, 0.1)
    zero_gains = {'kp': 0.0, 'kd': 0.0}
    full = load_controller(plugin, dyn, zero_gains, model.joints, NominalModel(dyn))
    rb = load_controller(plugin, dyn, zero_gains, model.joints, NominalModel(dyn, ['rigid_body']))
    assert np.allclose(full.compute(0, q, qd, q, z, z) - rb.compute(0, q, qd, q, z, z), 0.0)
    assert np.allclose(full.compute(0, q, z, q, qd, z) - rb.compute(0, q, z, q, qd, z),
                       dyn.damping * 0.1 + dyn.friction * np.tanh(2.0))
    assert full.model_metadata()['viscous_velocity_source'] == 'desired'


def test_computed_torque_coriolis_on_measured_velocity(model, dyn):
    q = np.array([0.2, 0.25, -1.3, 0.4, -0.3, 0.3, 0.3])
    z = np.zeros(7)
    qd = np.full(7, 0.3)
    ct = load_controller('computed_torque', dyn, {'kp': 0.0, 'kd': 0.0}, model.joints,
                         NominalModel(dyn, ['rigid_body']))
    assert np.allclose(ct.compute(0, q, qd, q, z, z), dyn.c(q, qd) + dyn.g(q))


def test_pd_gravity_is_gravity_only(model, dyn):
    q = np.array(model.candle)
    qd = np.full(7, 0.1)
    pg = load_controller('pd_gravity', dyn, {'kp': 0.0, 'kd': 0.0}, model.joints, NominalModel(dyn))
    assert np.allclose(pg.compute(0, q, qd, q, qd, np.zeros(7)), dyn.g(q))
    meta = pg.model_metadata()
    assert meta['model_terms_used'] == ['gravity'] and meta['viscous_velocity_source'] == 'none'
    assert meta['model_terms'] == ['rigid_body', 'viscous', 'coulomb']


EXPERIMENTS = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                           'config', 'experiments')


@pytest.mark.parametrize('name', sorted(os.listdir(EXPERIMENTS)))
def test_shipped_experiments_load(dyn, name):
    from ur5_controller.torque_control import load_experiment
    exp = load_experiment(os.path.join(EXPERIMENTS, name))
    assert exp.trials and 'rigid_body' in NominalModel(dyn, exp.model_terms, exp.coulomb_epsilon).terms


@pytest.mark.parametrize('name', sorted(os.listdir(EXPERIMENTS)))
def test_shipped_friction_levels_within_bounds(model, dyn, name):
    from ur5_controller.torque_control import load_experiment, worst_case_inertia
    exp = load_experiment(os.path.join(EXPERIMENTS, name))
    trials = [t for t in exp.trials if t.perturbation == 'joint_friction']
    if not trials:
        return
    m_eff = worst_case_inertia(dyn, model, exp, 0.01)
    for t in trials:
        p = Perturbation(t.perturbation, t.level, t.seed, options=t.options)
        p.bind(model.joints, dyn.damping, dyn.friction, m_eff, 0.01)
        assert not any(p.metadata()['friction_b_p'][4:])


@pytest.mark.parametrize('name', sorted(os.listdir(EXPERIMENTS)))
def test_shipped_gains_follow_pd_gravity(name):
    """Joint-space feedback gains are the PD + gravity values; CT converts them by protocol."""
    import yaml
    with open(os.path.join(EXPERIMENTS, name)) as f:
        ctrls = {c['plugin']: c.get('gains', {}) for c in yaml.safe_load(f)['controllers']}
    ref = ctrls['pd_gravity']
    for plugin, g in ctrls.items():
        assert (g['kp'], g['kd']) == (ref['kp'], ref['kd']), plugin
    if 'computed_torque' in ctrls:
        assert ctrls['computed_torque']['protocol'] == 'joint_gain_match_at_candle'


def test_computed_torque_gain_matching(model, dyn):
    g = {'protocol': 'joint_gain_match_at_candle', **GAINS}
    ct = load_controller('computed_torque', dyn, g, model.joints, gain_pose=model.candle)
    m = np.diag(dyn.M(model.candle))
    assert np.allclose(ct.kp * m, GAINS['kp']) and np.allclose(ct.kd * m, GAINS['kd'])
    meta = ct.gain_metadata()
    assert meta['gain_protocol'] == 'joint_gain_match_at_candle'
    assert np.allclose(meta['gain_match_m_eff'], m) and np.allclose(meta['gains_effective']['kp'], ct.kp)
    q = np.array(model.candle)
    z = np.zeros(7)
    ref = q.copy()
    ref[4] += 0.01
    zero_terms = NominalModel(dyn, ['rigid_body'])
    ct0 = load_controller('computed_torque', dyn, g, model.joints, zero_terms, gain_pose=model.candle)
    tau = ct0.compute(0, q, z, ref, z, z) - dyn.g(q)
    assert tau[4] == pytest.approx(GAINS['kp'][4] * 0.01)
    with pytest.raises(InputError, match='Candle'):
        load_controller('computed_torque', dyn, g, model.joints)
    with pytest.raises(InputError, match='protocol'):
        load_controller('computed_torque', dyn, dict(GAINS, protocol='auto'), model.joints)
    pd = load_controller('pd_id_ff', dyn, GAINS, model.joints)
    assert pd.gain_metadata()['gain_protocol'] == 'none'
    assert np.allclose(pd.kp, GAINS['kp']) and np.allclose(pd.kd, GAINS['kd'])


def test_pd_terms_oppose_error(model, dyn):
    c = load_controller('pd_gravity', dyn, GAINS, model.joints)
    q = np.array(model.candle)
    z = np.zeros(7)
    ref = q.copy()
    ref[2] += 0.01
    assert c.compute(0.0, q, z, ref, z, z)[2] - dyn.g(q)[2] == pytest.approx(6.0)


def test_unknown_plugin_and_bad_gains(model, dyn):
    with pytest.raises(InputError, match='not found'):
        load_controller('nope', dyn, GAINS, model.joints)
    with pytest.raises(InputError, match='kp'):
        load_controller('pd_gravity', dyn, {'kd': 1}, model.joints)
    with pytest.raises(InputError, match='7 values'):
        load_controller('pd_gravity', dyn, {'kp': [1, 2], 'kd': 1}, model.joints)


def test_learned_id_refuses_missing_model(model, dyn, tmp_path, monkeypatch):
    monkeypatch.setenv('UR5_DATA_DIR', str(tmp_path))
    with pytest.raises(InputError, match='not found'):
        load_controller('learned_id', dyn, dict(GAINS, model='absent.npz'), model.joints)


def test_learned_id_loads_npz(model, dyn, tmp_path, monkeypatch):
    monkeypatch.setenv('UR5_DATA_DIR', str(tmp_path))
    (tmp_path / 'models').mkdir()
    rng = np.random.default_rng(0)
    np.savez(tmp_path / 'models' / 'tiny.npz', W0=rng.normal(size=(21, 8)), b0=np.zeros(8),
             W1=np.zeros((8, 7)), b1=np.zeros(7), x_mean=np.zeros(21), x_std=np.ones(21),
             y_mean=np.arange(7.0), y_std=np.ones(7))
    c = load_controller('learned_id', dyn, dict(GAINS, model='tiny.npz'), model.joints)
    q = np.array(model.candle)
    z = np.zeros(7)
    assert np.allclose(c.compute(0.0, q, z, q, z, z), np.arange(7.0))


def test_perturbation_sides():
    assert KINDS['velocity_noise'] == 'controller'
    assert {KINDS[k] for k in ('joint_friction', 'gain_loss', 'tip_payload', 'tip_force')} == {'plant'}


def test_gain_loss_and_friction_act_on_torque():
    tau = np.ones(7) * 10
    qd = np.ones(7)
    assert np.allclose(Perturbation('gain_loss', 0.8, 0).command(tau, qd), 8.0)
    with pytest.raises(InputError):
        Perturbation('gain_loss', 1.5, 0)


PROXIMAL = ['shoulder_pan_joint1', 'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint']


def _friction(level, eps=0.35, joints=PROXIMAL):
    return Perturbation('joint_friction', level, 0, options={'joints': joints, 'epsilon': eps})


def test_joint_friction_scales_urdf_on_listed_joints_from_measured_velocity(model, dyn):
    f = _friction(0.5)
    with pytest.raises(RuntimeError):
        f.command(np.zeros(7), np.zeros(7))
    m_eff = np.diag(dyn.M(model.candle))
    f.bind(model.joints, dyn.damping, dyn.friction, m_eff, 0.01)
    qd = np.full(7, 0.2)
    tau = np.full(7, 10.0)
    prox = np.array([j in PROXIMAL for j in model.joints])
    want = tau - np.where(prox, 0.5 * (dyn.damping * qd + dyn.friction * np.tanh(qd / 0.35)), 0.0)
    assert np.allclose(f.command(tau, qd), want)
    assert np.allclose(f.command(tau, qd)[~prox], 10.0)
    assert np.allclose(f.measured_velocity(qd), qd)
    meta = f.metadata()
    assert meta['perturbed_joints'] == PROXIMAL and meta['friction_velocity_source'] == 'measured'
    assert meta['friction_epsilon'] == 0.35 and meta['friction_fraction'] == 0.5
    zero = _friction(0.0)
    zero.bind(model.joints, dyn.damping, dyn.friction, m_eff, 0.01)
    assert np.allclose(zero.command(tau, qd), tau)


def test_joint_friction_refuses_unstable_levels(model, dyn):
    m_eff = np.diag(dyn.M(model.candle))
    with pytest.raises(InputError, match='wrist_1_joint'):
        _friction(0.25, joints=PROXIMAL + ['wrist_1_joint']).bind(
            model.joints, dyn.damping, dyn.friction, m_eff, 0.01)
    with pytest.raises(InputError, match='epsilon'):
        _friction(0.5, eps=0.05).bind(model.joints, dyn.damping, dyn.friction, m_eff, 0.01)
    with pytest.raises(InputError, match='unknown joints'):
        _friction(0.25, joints=['nope']).bind(model.joints, dyn.damping, dyn.friction, m_eff, 0.01)
    for bad in ({'viscous': 1.0}, -0.1):
        with pytest.raises(InputError):
            _friction(bad)
    with pytest.raises(InputError, match='joints'):
        Perturbation('joint_friction', 0.25, 0, options={'epsilon': 0.35})


def test_velocity_noise_seeded_and_controller_side():
    qd = np.zeros(7)
    a = Perturbation('velocity_noise', 0.01, 3).measured_velocity(qd)
    b = Perturbation('velocity_noise', 0.01, 3).measured_velocity(qd)
    c = Perturbation('velocity_noise', 0.01, 4).measured_velocity(qd)
    assert np.allclose(a, b) and not np.allclose(a, c)
    assert np.allclose(Perturbation('velocity_noise', 0.01, 3).command(np.ones(7), qd), 1.0)


def test_tip_wrench_levels():
    assert Perturbation('tip_payload', 2.0, 0).force == pytest.approx((0, 0, -19.62))
    assert Perturbation('tip_force', [1, 2, 3], 0).force == (1.0, 2.0, 3.0)
    with pytest.raises(InputError):
        Perturbation('tip_force', [1, 2], 0)
    with pytest.raises(InputError, match='unknown'):
        Perturbation('wind', 1, 0)
