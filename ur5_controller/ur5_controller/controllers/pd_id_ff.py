"""PD plus inverse-dynamics feedforward on the reference.

tau = Kp e + Kd de + M(q_ref) qdd_ref + C(q_ref, qd_ref) qd_ref + g(q_ref) + F(qd_ref)

F is the selected joint terms (model_terms): viscous b qd_ref, Coulomb Fc tanh(qd_ref / epsilon).
"""
import numpy as np

from .base import TorqueController


class Controller(TorqueController):
    family = 'model-based'

    def compute(self, t, q, qd, q_ref, qd_ref, qdd_ref):
        e = np.asarray(q_ref) - np.asarray(q)
        de = np.asarray(qd_ref) - np.asarray(qd)
        return (self.kp * e + self.kd * de + self.dyn.inverse(q_ref, qd_ref, qdd_ref)
                + self.nominal.joint_friction(qd_ref))
