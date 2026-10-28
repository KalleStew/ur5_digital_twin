"""Torque controller plug-in interface. Brain B calls compute() once per joint state sample."""
from __future__ import annotations

import os

import numpy as np

from .dynamics import NominalModel
from .robot import InputError
from .trajectory_io import data_dir


class TorqueController:
    """Subclass, set `family`, implement compute(). Module must expose `Controller`."""

    family = 'model-based'

    def __init__(self, dynamics, gains: dict, joints, nominal: NominalModel | None = None,
                 gain_pose=None) -> None:
        self.dyn = dynamics
        self.nominal = nominal if nominal is not None else NominalModel(dynamics)
        self.joints = tuple(joints)
        self.gains = gains
        self.gain_pose = gain_pose
        self.kp = self._vec('kp')
        self.kd = self._vec('kd')

    def _vec(self, key: str) -> np.ndarray:
        if key not in self.gains:
            raise InputError(f'{type(self).__module__}: gains.{key} missing')
        v = self.gains[key]
        v = [float(v)] * len(self.joints) if np.isscalar(v) else [float(x) for x in v]
        if len(v) != len(self.joints):
            raise InputError(f'gains.{key} needs {len(self.joints)} values, got {len(v)}')
        return np.asarray(v)

    def model_description(self) -> str:
        return f'KDL from URDF: {" + ".join(self.nominal.terms)}'

    def model_metadata(self) -> dict:
        """Selected model terms with their URDF values, and the terms this law evaluates."""
        return {**self.nominal.describe(), 'model_terms_used': list(self.nominal.terms),
                'nominal_model': self.model_description()}

    def _without_terms(self, used: str) -> dict:
        return {**self.nominal.describe(), 'model_terms_used': [used],
                'nominal_model': self.model_description(),
                'viscous_velocity_source': 'none', 'coulomb_velocity_source': 'none'}

    def gain_metadata(self) -> dict:
        """Gain protocol and the gains compute() applies, for the HDF5 metadata."""
        return {'gain_protocol': 'none', 'gains_effective': {'kp': self.kp.tolist(), 'kd': self.kd.tolist()}}

    def velocity_feedback_step(self, m_eff, period: float) -> np.ndarray:
        """Per-joint Kd T / m_eff of the measured-velocity feedback (kd in N m s/rad)."""
        return self.kd * period / np.asarray(m_eff, dtype=float)

    def reset(self) -> None:
        """Called before each trial."""

    def compute(self, t, q, qd, q_ref, qd_ref, qdd_ref) -> np.ndarray:
        raise NotImplementedError


class PdGravity(TorqueController):
    family = 'model-based'

    def model_description(self) -> str:
        return 'KDL from URDF: g(q) only (model_terms not used)'

    def model_metadata(self) -> dict:
        return self._without_terms('gravity')

    def compute(self, t, q, qd, q_ref, qd_ref, qdd_ref):
        q, qd = np.asarray(q), np.asarray(qd)
        return self.kp * (np.asarray(q_ref) - q) + self.kd * (np.asarray(qd_ref) - qd) + self.dyn.g(q)

class PdIdFf(TorqueController):
    family = 'model-based'

    def compute(self, t, q, qd, q_ref, qd_ref, qdd_ref):
        e = np.asarray(q_ref) - np.asarray(q)
        de = np.asarray(qd_ref) - np.asarray(qd)
        return (self.kp * e + self.kd * de + self.dyn.inverse(q_ref, qd_ref, qdd_ref)
                + self.nominal.joint_friction(qd_ref))

GAIN_PROTOCOL = 'joint_gain_match_at_candle'


class ComputedTorque(TorqueController):
    family = 'model-based'

    def __init__(self, dynamics, gains, joints, nominal=None, gain_pose=None):
        super().__init__(dynamics, gains, joints, nominal, gain_pose)
        self.protocol = str(gains.get('protocol', 'none'))
        self.m_eff = None
        if self.protocol == GAIN_PROTOCOL:
            if gain_pose is None:
                raise InputError(f'computed_torque: {GAIN_PROTOCOL} needs the Candle pose')
            self.m_eff = np.diag(self.dyn.M(gain_pose)).copy()
            self.kp = self.kp / self.m_eff
            self.kd = self.kd / self.m_eff
        elif self.protocol != 'none':
            raise InputError(f'computed_torque: unknown gains.protocol {self.protocol!r}; '
                             f'use {GAIN_PROTOCOL} or omit it')

    def gain_metadata(self) -> dict:
        meta = super().gain_metadata()
        meta['gain_protocol'] = self.protocol
        if self.m_eff is not None:
            meta['gain_match_m_eff'] = self.m_eff.tolist()
        return meta

    def velocity_feedback_step(self, m_eff, period: float) -> np.ndarray:
        """kd acts in acceleration space; M(q) cancels m_eff on the diagonal."""
        return self.kd * period

    def compute(self, t, q, qd, q_ref, qd_ref, qdd_ref):
        q, qd = np.asarray(q), np.asarray(qd)
        v = np.asarray(qdd_ref) + self.kp * (np.asarray(q_ref) - q) + self.kd * (np.asarray(qd_ref) - qd)
        return (self.dyn.M(q) @ v + self.dyn.c(q, qd) + self.dyn.g(q)
                + self.nominal.joint_friction(qd_ref))

class LearnedId(TorqueController):
    family = 'learned'

    def __init__(self, dynamics, gains, joints, nominal=None, gain_pose=None):
        super().__init__(dynamics, gains, joints, nominal, gain_pose)
        path = gains.get('model')
        if not path:
            raise InputError('learned_id: gains.model (path to .npz weights) is required')
        if not os.path.isabs(path):
            path = os.path.join(data_dir(), 'models', path)
        if not os.path.isfile(path):
            raise InputError(f'learned_id: model file {path} not found; training is not part of the platform')
        z = np.load(path)
        n = len([k for k in z.files if k.startswith('W')])
        if n == 0:
            raise InputError(f'learned_id: {path} has no W0..Wk arrays')
        self.W = [z[f'W{i}'] for i in range(n)]
        self.b = [z[f'b{i}'] for i in range(n)]
        self.x_mean, self.x_std = z['x_mean'], z['x_std']
        self.y_mean, self.y_std = z['y_mean'], z['y_std']
        dof = len(self.joints)
        if self.W[0].shape[0] != 3 * dof or self.W[-1].shape[1] != dof:
            raise InputError(f'learned_id: model must map {3 * dof} inputs to {dof} outputs')
        self.model_path = path

    def model_description(self) -> str:
        return f'learned feedforward {self.model_path} (model_terms not used)'

    def model_metadata(self) -> dict:
        return self._without_terms('learned')

    def feedforward(self, q_ref, qd_ref, qdd_ref) -> np.ndarray:
        h = (np.concatenate([q_ref, qd_ref, qdd_ref]) - self.x_mean) / self.x_std
        for W, b in zip(self.W[:-1], self.b[:-1]):
            h = np.tanh(h @ W + b)
        return (h @ self.W[-1] + self.b[-1]) * self.y_std + self.y_mean

    def compute(self, t, q, qd, q_ref, qd_ref, qdd_ref):
        e = np.asarray(q_ref) - np.asarray(q)
        de = np.asarray(qd_ref) - np.asarray(qd)
        return self.kp * e + self.kd * de + self.feedforward(np.asarray(q_ref), np.asarray(qd_ref),
                                                             np.asarray(qdd_ref))


LAWS = {
    'pd_gravity': PdGravity,
    'pd_id_ff': PdIdFf,
    'computed_torque': ComputedTorque,
    'learned_id': LearnedId,
}


def load_controller(plugin: str, dynamics, gains: dict, joints,
                    nominal: NominalModel | None = None, gain_pose=None) -> TorqueController:
    cls = LAWS.get(plugin)
    if cls is None:
        raise InputError(f'controller plug-in {plugin!r} not found')
    return cls(dynamics, gains or {}, joints, nominal, gain_pose)
