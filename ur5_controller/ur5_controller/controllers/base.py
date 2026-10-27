"""Torque controller plug-in interface. Brain B calls compute() once per joint state sample."""
from __future__ import annotations

import importlib

import numpy as np

from ..dynamics import NominalModel
from ..scheme import InputError


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


def load_controller(plugin: str, dynamics, gains: dict, joints,
                    nominal: NominalModel | None = None, gain_pose=None) -> TorqueController:
    if not plugin.replace('_', '').isalnum():
        raise InputError(f'invalid controller plug-in name {plugin!r}')
    try:
        mod = importlib.import_module(f'ur5_controller.controllers.{plugin}')
    except ModuleNotFoundError:
        raise InputError(f'controller plug-in {plugin!r} not found in ur5_controller/controllers/')
    cls = getattr(mod, 'Controller', None)
    if cls is None or not issubclass(cls, TorqueController):
        raise InputError(f'{plugin}.py must define Controller(TorqueController)')
    return cls(dynamics, gains or {}, joints, nominal, gain_pose)
