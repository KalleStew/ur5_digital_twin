"""Computed torque: tau = M(q) (qdd_ref + Kp e + Kd de) + C(q, qd) qd + g(q) + F(qd_ref).

C(q, qd) qd uses the measured velocity. F is the selected joint terms (model_terms) on the
reference velocity: viscous b qd_ref, Coulomb Fc tanh(qd_ref / epsilon).

Gains act in acceleration space (1/s^2, 1/s) because M(q) scales them. With
`gains.protocol: joint_gain_match_at_candle`, kp and kd are given in N m/rad and N m s/rad
(the PD + gravity values) and converted once per experiment:
kp_j = Kp_j / M_jj(q_candle), kd_j = Kd_j / M_jj(q_candle), so the joint-space stiffness and
damping on the diagonal at Candle equal the PD + gravity gains.
"""
import numpy as np

from ..scheme import InputError
from .base import TorqueController

GAIN_PROTOCOL = 'joint_gain_match_at_candle'


class Controller(TorqueController):
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

    def compute(self, t, q, qd, q_ref, qd_ref, qdd_ref):
        q, qd = np.asarray(q), np.asarray(qd)
        v = np.asarray(qdd_ref) + self.kp * (np.asarray(q_ref) - q) + self.kd * (np.asarray(qd_ref) - qd)
        return (self.dyn.M(q) @ v + self.dyn.c(q, qd) + self.dyn.g(q)
                + self.nominal.joint_friction(qd_ref))
