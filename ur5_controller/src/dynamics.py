"""Nominal model from the URDF: g(q), M(q), C(q, qd) qd via KDL ChainDynParam.

Joint damping and friction from <dynamics> are not part of M, C or g; NominalModel adds the
ones an experiment selects. Torques follow the actuator convention: the effort that holds or
drives the arm.
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET

import numpy as np
import PyKDL as kdl

from ur5_common import Model


def _rpy(origin):
    rpy = [float(x) for x in (origin.get('rpy') or '0 0 0').split()] if origin is not None else [0, 0, 0]
    xyz = [float(x) for x in (origin.get('xyz') or '0 0 0').split()] if origin is not None else [0, 0, 0]
    return kdl.Frame(kdl.Rotation.RPY(*rpy), kdl.Vector(*xyz))


def _inertia(link) -> kdl.RigidBodyInertia:
    inert = link.find('inertial') if link is not None else None
    if inert is None:
        return kdl.RigidBodyInertia()
    m = float(inert.find('mass').get('value'))
    i = inert.find('inertia')
    g = {k: float(i.get(k, 0.0)) for k in ('ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz')}
    f = _rpy(inert.find('origin'))
    rot = kdl.RotationalInertia(g['ixx'], g['iyy'], g['izz'], g['ixy'], g['ixz'], g['iyz'])
    return f * kdl.RigidBodyInertia(m, kdl.Vector.Zero(), rot)


def build_chain(urdf_xml: str, base: str, tip: str):
    """KDL chain base->tip, built the same way kdl_parser builds it. Also returns joint names."""
    root = ET.fromstring(urdf_xml)
    links = {ln.get('name'): ln for ln in root.findall('link')}
    by_child = {j.find('child').get('link'): j for j in root.findall('joint')}
    path = []
    link = tip
    while link != base:
        j = by_child[link]
        path.append(j)
        link = j.find('parent').get('link')
    chain = kdl.Chain()
    names = []
    for j in reversed(path):
        f = _rpy(j.find('origin'))
        child = j.find('child').get('link')
        if j.get('type') in ('revolute', 'continuous'):
            axis = [float(x) for x in (j.find('axis').get('xyz') if j.find('axis') is not None else '1 0 0').split()]
            jnt = kdl.Joint(j.get('name'), f.p, f.M * kdl.Vector(*axis), kdl.Joint.RotAxis)
            names.append(j.get('name'))
        else:
            jnt = kdl.Joint(j.get('name'), kdl.Joint.Fixed)
        chain.addSegment(kdl.Segment(child, jnt, f, _inertia(links.get(child))))
    return chain, names


def joint_dynamics(urdf_xml: str, joints) -> tuple[np.ndarray, np.ndarray]:
    """URDF <dynamics damping, friction> per joint (plant-side, not in the nominal model)."""
    root = ET.fromstring(urdf_xml)
    d = {j.get('name'): j.find('dynamics') for j in root.findall('joint')}
    damp = np.array([float(d[n].get('damping', 0)) if d.get(n) is not None else 0.0 for n in joints])
    fric = np.array([float(d[n].get('friction', 0)) if d.get(n) is not None else 0.0 for n in joints])
    return damp, fric


class Dynamics:
    def __init__(self, model: Model, base: str = 'base_link', tip: str = 'tool0',
                 gravity=(0.0, 0.0, -9.81)) -> None:
        chain, names = build_chain(model.urdf_xml, base, tip)
        if tuple(names) != tuple(model.joints):
            raise ValueError(f'KDL chain order {names} != model order {model.joints}')
        self.joints = tuple(names)
        self.n = len(names)
        self.chain = chain
        self.solver = kdl.ChainDynParam(chain, kdl.Vector(*gravity))
        self.damping, self.friction = joint_dynamics(model.urdf_xml, names)

    def _arr(self, x) -> kdl.JntArray:
        a = kdl.JntArray(self.n)
        for i, v in enumerate(x):
            a[i] = float(v)
        return a

    def g(self, q) -> np.ndarray:
        out = kdl.JntArray(self.n)
        self.solver.JntToGravity(self._arr(q), out)
        return np.array([out[i] for i in range(self.n)])

    def M(self, q) -> np.ndarray:
        out = kdl.JntSpaceInertiaMatrix(self.n)
        self.solver.JntToMass(self._arr(q), out)
        return np.array([[out[i, j] for j in range(self.n)] for i in range(self.n)])

    def c(self, q, qd) -> np.ndarray:
        """Coriolis and centrifugal torque C(q, qd) qd."""
        out = kdl.JntArray(self.n)
        self.solver.JntToCoriolis(self._arr(q), self._arr(qd), out)
        return np.array([out[i] for i in range(self.n)])

    def inverse(self, q, qd, qdd) -> np.ndarray:
        return self.M(q) @ np.asarray(qdd, dtype=float) + self.c(q, qd) + self.g(q)


MODEL_TERMS = ('rigid_body', 'viscous', 'coulomb')
DEFAULT_MODEL_TERMS = ('rigid_body', 'viscous', 'coulomb')
DEFAULT_COULOMB_EPSILON = 0.05


class NominalModel:
    """Joint terms the model-based laws add to M, C, g, with parameters read from the URDF.

    Both are evaluated on the reference velocity: viscous b * qd_ref and Coulomb
    Fc * tanh(qd_ref / epsilon). With b * qd on the measured velocity the 100 Hz loop is unstable
    on the wrists, whose damping dwarfs their inertia (b T / m_eff up to ~600).
    """

    VELOCITY_SOURCE = 'desired'

    def __init__(self, dyn: Dynamics, terms=DEFAULT_MODEL_TERMS,
                 coulomb_epsilon=DEFAULT_COULOMB_EPSILON) -> None:
        from ur5_common import InputError
        terms = tuple(str(t) for t in (terms or ()))
        unknown = [t for t in terms if t not in MODEL_TERMS]
        if unknown or len(set(terms)) != len(terms):
            raise InputError(f'model_terms {list(terms)}: allowed {list(MODEL_TERMS)}, each once')
        if 'rigid_body' not in terms:
            raise InputError('model_terms must include rigid_body')
        self.terms = tuple(t for t in MODEL_TERMS if t in terms)
        self.epsilon = None
        if 'coulomb' in self.terms:
            try:
                self.epsilon = float(coulomb_epsilon)
            except (TypeError, ValueError):
                raise InputError('model_terms includes coulomb: set coulomb_epsilon (rad/s) > 0')
            if not (math.isfinite(self.epsilon) and self.epsilon > 0):
                raise InputError('model_terms includes coulomb: set coulomb_epsilon (rad/s) > 0')
        self.b = dyn.damping.copy() if 'viscous' in self.terms else np.zeros(dyn.n)
        self.fc = dyn.friction.copy() if 'coulomb' in self.terms else np.zeros(dyn.n)

    def joint_friction(self, qd_ref) -> np.ndarray:
        qd_ref = np.asarray(qd_ref, dtype=float)
        tau = self.b * qd_ref
        if self.epsilon is not None:
            tau = tau + self.fc * np.tanh(qd_ref / self.epsilon)
        return tau

    def describe(self) -> dict:
        return {'model_terms': list(self.terms), 'model_viscous_b': self.b.tolist(),
                'model_coulomb_fc': self.fc.tolist(),
                'model_coulomb_epsilon': self.epsilon if self.epsilon is not None else 'none',
                'viscous_velocity_source': self.VELOCITY_SOURCE if 'viscous' in self.terms else 'none',
                'coulomb_velocity_source': self.VELOCITY_SOURCE if 'coulomb' in self.terms else 'none'}


def is_finite(x) -> bool:
    return all(math.isfinite(v) for v in x)
