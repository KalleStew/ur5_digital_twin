"""PD plus gravity compensation: tau = Kp e + Kd de + g(q)."""
import numpy as np

from .base import TorqueController


class Controller(TorqueController):
    family = 'model-based'

    def model_description(self) -> str:
        return 'KDL from URDF: g(q) only (model_terms not used)'

    def model_metadata(self) -> dict:
        return self._without_terms('gravity')

    def compute(self, t, q, qd, q_ref, qd_ref, qdd_ref):
        q, qd = np.asarray(q), np.asarray(qd)
        return self.kp * (np.asarray(q_ref) - q) + self.kd * (np.asarray(qd_ref) - qd) + self.dyn.g(q)
