"""Learned inverse-dynamics feedforward plus PD: tau = f(q_ref, qd_ref, qdd_ref) + Kp e + Kd de.

Model file (.npz): W0, b0, ..., Wk, bk for a tanh MLP with a linear output layer, plus
x_mean, x_std (21,) and y_mean, y_std (7,). Input x = [q_ref, qd_ref, qdd_ref] in controller
joint order; output is joint torque in N m. A relative `model` path resolves under
$UR5_DATA_DIR/models/.
"""
import os

import numpy as np

from ..scheme import InputError
from ..trajectory_io import data_dir
from .base import TorqueController


class Controller(TorqueController):
    family = 'learned'

    def __init__(self, dynamics, gains, joints, nominal=None):
        super().__init__(dynamics, gains, joints, nominal)
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
