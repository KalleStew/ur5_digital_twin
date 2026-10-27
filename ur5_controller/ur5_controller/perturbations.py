"""Runtime model-mismatch injection; no Gazebo restart and no URDF edit.

side='plant'      acts on the torque reaching the joints or on the physical arm.
side='controller' corrupts what the control law sees; the logged qd stays clean.
"""
from __future__ import annotations

import math
import subprocess

import numpy as np

from .scheme import InputError, PlatformError

KINDS = {
    'none': 'plant',
    'joint_friction': 'plant',
    'gain_loss': 'plant',
    'velocity_noise': 'controller',
    'tip_payload': 'plant',
    'tip_force': 'plant',
}

# joint_friction is fed back from measured qd once per loop period T and held for T.
# Per perturbed joint: b_p T / m_eff <= FRICTION_VISCOUS_STEP_MAX and
# epsilon >= FRICTION_EPSILON_FACTOR * Fc_p T / m_eff, with m_eff the worst-case (smallest) M_jj.
FRICTION_VISCOUS_STEP_MAX = 0.5
FRICTION_EPSILON_FACTOR = 2.0


class Perturbation:
    def __init__(self, kind: str, level, seed: int, cfg: dict | None = None, dof: int = 7,
                 options: dict | None = None) -> None:
        if kind not in KINDS:
            raise InputError(f'unknown perturbation {kind!r}; one of {sorted(KINDS)}')
        self.kind = kind
        self.side = KINDS[kind]
        self.level = level
        self.dof = dof
        self.rng = np.random.default_rng(seed)
        self.cfg = cfg or {}
        self._wrench_on = False
        if kind == 'joint_friction':
            opts = options or {}
            try:
                self.fraction = float(level)
            except (TypeError, ValueError):
                raise InputError('joint_friction level is a fraction of the URDF b and Fc, e.g. 0.25')
            if self.fraction < 0:
                raise InputError('joint_friction level must be >= 0')
            self.joints = [str(j) for j in opts.get('joints') or []]
            if not self.joints:
                raise InputError('joint_friction needs joints: [<joint names>]')
            self.eps = float(opts.get('epsilon', 0.0))
            if self.eps <= 0:
                raise InputError('joint_friction needs epsilon > 0 (rad/s)')
            self.b = np.zeros(dof)
            self.fc = np.zeros(dof)
            self.bound: dict = {}
        elif kind == 'gain_loss':
            self.alpha = self._vec(level)
            if np.any(self.alpha < 0) or np.any(self.alpha > 1):
                raise InputError('gain_loss level must be in [0, 1]')
        elif kind == 'velocity_noise':
            self.sigma = self._vec(level)
        elif kind == 'tip_payload':
            self.force = (0.0, 0.0, -9.81 * float(level))
        elif kind == 'tip_force':
            f = level if isinstance(level, (list, tuple)) else [0.0, 0.0, float(level)]
            if len(f) != 3:
                raise InputError('tip_force level must be a scalar (z, N) or [fx, fy, fz]')
            self.force = tuple(float(x) for x in f)

    def _vec(self, v) -> np.ndarray:
        if np.isscalar(v):
            return np.full(self.dof, float(v))
        a = np.asarray(v, dtype=float)
        if a.shape != (self.dof,):
            raise InputError(f'{self.kind} level needs a scalar or {self.dof} values')
        return a

    def bind(self, joints, damping, friction, m_eff, period: float) -> None:
        """Scale the URDF b and Fc of the perturbed joints; refuse a level the loop cannot hold stably."""
        if self.kind != 'joint_friction':
            return
        joints = list(joints)
        unknown = [j for j in self.joints if j not in joints]
        if unknown:
            raise InputError(f'joint_friction: unknown joints {unknown}; model joints {joints}')
        mask = np.array([j in self.joints for j in joints])
        m_eff = np.asarray(m_eff, dtype=float)
        self.b = np.where(mask, self.fraction * np.asarray(damping, dtype=float), 0.0)
        self.fc = np.where(mask, self.fraction * np.asarray(friction, dtype=float), 0.0)
        step = self.b * period / m_eff
        eps_min = FRICTION_EPSILON_FACTOR * self.fc * period / m_eff
        for i, j in enumerate(joints):
            if step[i] > FRICTION_VISCOUS_STEP_MAX:
                raise InputError(f'joint_friction level {self.fraction}: {j} b_p T/m_eff = {step[i]:.3f} '
                                 f'> {FRICTION_VISCOUS_STEP_MAX} (unstable at {1 / period:.0f} Hz)')
            if self.eps < eps_min[i]:
                raise InputError(f'joint_friction level {self.fraction}: epsilon {self.eps} < '
                                 f'{eps_min[i]:.3f} rad/s needed on {j}')
        self.bound = {
            'perturbed_joints': [j for j in joints if j in self.joints],
            'friction_fraction': self.fraction,
            'friction_b_p': self.b.tolist(),
            'friction_fc_p': self.fc.tolist(),
            'friction_epsilon': self.eps,
            'friction_velocity_source': 'measured',
            'friction_m_eff_min': m_eff.tolist(),
            'friction_viscous_step': step.tolist(),
            'friction_epsilon_min': eps_min.tolist(),
        }

    def describe(self) -> dict:
        return {'type': self.kind, 'level': self.level, 'side': self.side}

    def metadata(self) -> dict:
        return dict(self.bound) if self.kind == 'joint_friction' else {}

    def measured_velocity(self, qd) -> np.ndarray:
        qd = np.asarray(qd, dtype=float)
        if self.kind == 'velocity_noise':
            return qd + self.rng.normal(0.0, 1.0, self.dof) * self.sigma
        return qd

    def command(self, tau, qd) -> np.ndarray:
        tau = np.asarray(tau, dtype=float)
        if self.kind == 'gain_loss':
            return self.alpha * tau
        if self.kind == 'joint_friction':
            if not self.bound:
                raise RuntimeError('joint_friction used before bind()')
            qd = np.asarray(qd, dtype=float)
            return tau - self.b * qd - self.fc * np.tanh(qd / self.eps)
        return tau

    # Payload and tip force use the gz ApplyLinkWrench system (persistent until cleared).
    # Its interface is topics, so a missing subscriber is checked first; a publish alone cannot fail.
    def _gz(self, topic: str, msg_type: str, payload: str) -> None:
        name = f'/world/{self.cfg["gz_world"]}/wrench' + (f'/{topic}' if topic else '')
        info = subprocess.run(['gz', 'topic', '-i', '-t', name], capture_output=True, text=True, timeout=10)
        if 'Subscribers' not in info.stdout:
            raise PlatformError(f'{name} has no subscriber; the world lacks the ApplyLinkWrench system')
        p = subprocess.run(['gz', 'topic', '-t', name, '-m', msg_type, '-p', payload],
                           capture_output=True, text=True, timeout=10)
        if p.returncode != 0:
            raise PlatformError(f'gz publish on {name} failed: {(p.stdout + p.stderr).strip()[:120]}')

    def _entity(self) -> str:
        return f'name: "{self.cfg["gz_model"]}::{self.cfg["gz_tip_link"]}", type: LINK'

    def apply_plant(self) -> None:
        if self.kind in ('tip_payload', 'tip_force') and any(abs(x) > 0 for x in self.force):
            fx, fy, fz = self.force
            self._gz('persistent', 'gz.msgs.EntityWrench',
                     f'entity: {{{self._entity()}}}, wrench: {{force: {{x: {fx}, y: {fy}, z: {fz}}}}}')
            self._wrench_on = True

    def clear_plant(self) -> None:
        if self._wrench_on:
            self._gz('clear', 'gz.msgs.Entity', self._entity())
            self._wrench_on = False


def finite(x) -> bool:
    return all(math.isfinite(float(v)) for v in x)
