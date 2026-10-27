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


class Perturbation:
    def __init__(self, kind: str, level, seed: int, cfg: dict | None = None, dof: int = 7) -> None:
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
            lv = level if isinstance(level, dict) else {}
            self.b = self._vec(lv.get('viscous', 0.0))
            self.fc = self._vec(lv.get('coulomb', 0.0))
            self.eps = float(lv.get('epsilon', 0.01))
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

    def describe(self) -> dict:
        return {'type': self.kind, 'level': self.level, 'side': self.side}

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
            qd = np.asarray(qd, dtype=float)
            return tau - self.b * qd - self.fc * np.tanh(qd / self.eps)
        return tau

    # Payload and tip force use the gz ApplyLinkWrench system (persistent until cleared).
    def _gz(self, service: str, req: str) -> None:
        world = self.cfg['gz_world']
        cmd = ['gz', 'service', '-s', f'/world/{world}/wrench/{service}', '--reqtype',
               'gz.msgs.EntityWrench', '--reptype', 'gz.msgs.Empty', '--timeout', '3000', '--req', req]
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
        if p.returncode != 0 or 'timed out' in (p.stdout + p.stderr).lower():
            raise PlatformError(f'gz wrench/{service} failed: {(p.stdout + p.stderr).strip()[:120]}')

    def _entity(self) -> str:
        return (f'entity: {{name: "{self.cfg["gz_model"]}::{self.cfg["gz_tip_link"]}", '
                f'type: LINK}}')

    def apply_plant(self) -> None:
        if self.kind in ('tip_payload', 'tip_force') and any(abs(x) > 0 for x in self.force):
            fx, fy, fz = self.force
            self._gz('persistent', f'{self._entity()}, wrench: {{force: {{x: {fx}, y: {fy}, z: {fz}}}}}')
            self._wrench_on = True

    def clear_plant(self) -> None:
        if self._wrench_on:
            self._gz('clear', self._entity())
            self._wrench_on = False


def finite(x) -> bool:
    return all(math.isfinite(float(v)) for v in x)
