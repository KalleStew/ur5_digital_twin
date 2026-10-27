"""Computed torque: tau = M(q) (qdd_ref + Kp e + Kd de) + C(q, qd) qd + g(q).

Gains here are natural-frequency style (1/s^2, 1/s) because M(q) scales them.
"""
import numpy as np

from .base import TorqueController


class Controller(TorqueController):
    family = 'model-based'

    def compute(self, t, q, qd, q_ref, qd_ref, qdd_ref):
        q, qd = np.asarray(q), np.asarray(qd)
        v = np.asarray(qdd_ref) + self.kp * (np.asarray(q_ref) - q) + self.kd * (np.asarray(qd_ref) - qd)
        return self.dyn.M(q) @ v + self.dyn.c(q, qd) + self.dyn.g(q)
