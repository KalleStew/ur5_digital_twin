"""Torque controller plug-in interface. Brain B calls compute() once per /joint_states sample."""
from __future__ import annotations

import importlib

import numpy as np

from ..scheme import InputError


class TorqueController:
    """Subclass, set `family`, implement compute(). Module must expose `Controller`."""

    family = 'model-based'

    def __init__(self, dynamics, gains: dict, joints) -> None:
        self.dyn = dynamics
        self.joints = tuple(joints)
        self.gains = gains
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

    def reset(self) -> None:
        """Called before each trial."""

    def compute(self, t, q, qd, q_ref, qd_ref, qdd_ref) -> np.ndarray:
        raise NotImplementedError


def load_controller(plugin: str, dynamics, gains: dict, joints) -> TorqueController:
    if not plugin.replace('_', '').isalnum():
        raise InputError(f'invalid controller plug-in name {plugin!r}')
    try:
        mod = importlib.import_module(f'ur5_controller.controllers.{plugin}')
    except ModuleNotFoundError:
        raise InputError(f'controller plug-in {plugin!r} not found in ur5_controller/controllers/')
    cls = getattr(mod, 'Controller', None)
    if cls is None or not issubclass(cls, TorqueController):
        raise InputError(f'{plugin}.py must define Controller(TorqueController)')
    return cls(dynamics, gains or {}, joints)
