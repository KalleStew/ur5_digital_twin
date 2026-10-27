"""PD plus inverse-dynamics feedforward evaluated on the reference: tau = Kp e + Kd de + ID(q_ref, qd_ref, qdd_ref)."""
import numpy as np

from .base import TorqueController


class Controller(TorqueController):
    family = 'model-based'

    def compute(self, t, q, qd, q_ref, qd_ref, qdd_ref):
        e = np.asarray(q_ref) - np.asarray(q)
        de = np.asarray(qd_ref) - np.asarray(qd)
        return self.kp * e + self.kd * de + self.dyn.inverse(q_ref, qd_ref, qdd_ref)
