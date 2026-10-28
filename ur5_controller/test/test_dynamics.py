import numpy as np
import pytest

from ur5_controller.dynamics import Dynamics
from ur5_controller.robot import load_model, load_platform


@pytest.fixture(scope='module')
def dyn():
    return Dynamics(load_model(load_platform()))


def test_gravity_at_candle_matches_measured_hold_effort(dyn):
    # Effort read from /joint_states with the arm settled at candle_pose under Brain A.
    measured = [0.0, -12.324, -21.521, -4.000, -0.562, 0.022, 0.003]
    g = dyn.g(load_model(load_platform()).candle)
    assert np.allclose(g, measured, atol=0.1)


def test_mass_matrix_symmetric_positive_definite(dyn):
    q = [0.3, 0.2, -1.2, 0.6, 0.1, -0.2, 0.4]
    M = dyn.M(q)
    assert np.allclose(M, M.T, atol=1e-9)
    assert np.all(np.linalg.eigvalsh(M) > 0)


def test_inverse_dynamics_at_rest_is_gravity(dyn):
    q = [0.1, 0.0, -1.0, 0.5, 0.0, 0.3, 0.0]
    z = [0.0] * 7
    assert np.allclose(dyn.inverse(q, z, z), dyn.g(q))


def test_plant_friction_reported_separately(dyn):
    assert dyn.friction[2] == pytest.approx(15.0)
    assert dyn.damping[2] == pytest.approx(30.0)
