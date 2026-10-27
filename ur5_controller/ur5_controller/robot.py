"""Joint names, Candle pose, limits and live state, all resolved by name."""
from __future__ import annotations

import math
import os
import threading
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field

import yaml

# Fallbacks only; tests assert these match the URDF chain, SRDF candle_pose and URDF initial_value.
JOINTS = (
    'shoulder_pan_joint1', 'shoulder_pan_joint', 'shoulder_lift_joint',
    'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
)
CANDLE = (0.0, 0.0, -1.5708, 0.0, 0.0, 0.0, 0.0)


def _share(package: str) -> str:
    from ament_index_python.packages import get_package_share_directory
    return get_package_share_directory(package)


def _source_root() -> str:
    """Repository root when running from the source tree (tests, symlink installs)."""
    return os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))


def resolve_file(package: str, relpath: str) -> str:
    try:
        path = os.path.join(_share(package), relpath)
        if os.path.exists(path):
            return path
    except Exception:  # noqa: BLE001
        pass
    path = os.path.join(_source_root(), package, relpath)
    if os.path.exists(path):
        return path
    raise FileNotFoundError(f'{package}/{relpath} not found (is the workspace sourced?)')


def load_platform(path: str | None = None) -> dict:
    path = path or os.environ.get('UR5_PLATFORM_CONFIG') or resolve_file(
        'ur5_controller', 'config/platform.yaml')
    with open(path) as f:
        return yaml.safe_load(f)


@dataclass
class JointLimit:
    lower: float
    upper: float
    velocity: float
    effort: float
    acceleration: float = 0.0


@dataclass
class Model:
    joints: tuple[str, ...]
    limits: dict[str, JointLimit]
    initial: dict[str, float]
    candle: tuple[float, ...]
    urdf_path: str
    urdf_xml: str = field(repr=False)

    def index(self, name: str) -> int:
        return self.joints.index(name)


def chain_joints(urdf_root: ET.Element, base: str, tip: str) -> list[ET.Element]:
    """Movable joints from base to tip, in kinematic order."""
    by_child = {j.find('child').get('link'): j for j in urdf_root.findall('joint')}
    chain = []
    link = tip
    while link != base:
        j = by_child.get(link)
        if j is None:
            raise ValueError(f'no path from {base} to {tip}')
        chain.append(j)
        link = j.find('parent').get('link')
    return [j for j in reversed(chain) if j.get('type') in ('revolute', 'continuous', 'prismatic')]


def srdf_state(srdf_path: str, state: str) -> dict[str, float]:
    root = ET.parse(srdf_path).getroot()
    for gs in root.findall('group_state'):
        if gs.get('name') == state:
            return {j.get('name'): float(j.get('value')) for j in gs.findall('joint')}
    raise KeyError(f'group_state {state!r} not in {srdf_path}')


def load_model(cfg: dict | None = None) -> Model:
    cfg = cfg or load_platform()
    m = cfg['model']
    urdf_path = resolve_file(m['urdf_package'], m['urdf_file'])
    with open(urdf_path) as f:
        xml = f.read()
    root = ET.fromstring(xml)
    chain = chain_joints(root, m['base_link'], m['tip_link'])
    joints = tuple(j.get('name') for j in chain)
    limits = {}
    for j in chain:
        lim = j.find('limit')
        limits[j.get('name')] = JointLimit(float(lim.get('lower')), float(lim.get('upper')),
                                           float(lim.get('velocity')), float(lim.get('effort')),
                                           float(lim.get('acceleration', 0.0)))
    initial = {}
    for rj in root.iter('joint'):
        if rj.find('state_interface') is None:
            continue
        for si in rj.findall('state_interface'):
            if si.get('name') == 'position':
                p = si.find("param[@name='initial_value']")
                if p is not None:
                    initial[rj.get('name')] = float(p.text)
    mv = cfg['moveit']
    state = srdf_state(resolve_file(mv['srdf_package'], mv['srdf_file']), mv['candle_state'])
    missing = [j for j in joints if j not in state]
    if missing:
        raise KeyError(f'candle_pose missing joints {missing}')
    return Model(joints, limits, initial, tuple(state[j] for j in joints), urdf_path, xml)


def reorder(names, values, target) -> list[float]:
    """Map values keyed by `names` onto `target` order; KeyError names the missing joint."""
    idx = {n: i for i, n in enumerate(names)}
    return [float(values[idx[t]]) for t in target]


def max_abs_diff(a, b) -> float:
    return max(abs(x - y) for x, y in zip(a, b))


@dataclass
class StateSample:
    t: float
    t_source: str
    clock_t: float
    q: list[float]
    qd: list[float]
    tau: list[float]


class StateCache:
    """Latest /joint_states in controller order, plus an optional recording window."""

    def __init__(self, node, joints, topic: str = '/joint_states') -> None:
        from sensor_msgs.msg import JointState
        self.node = node
        self.joints = tuple(joints)
        self._lock = threading.Condition()
        self._latest: StateSample | None = None
        self._seq = 0
        self._rec: list[StateSample] | None = None
        self._listeners = []
        node.create_subscription(JointState, topic, self._on_js, 50)

    @property
    def _clock_t(self) -> float | None:
        """Node clock (sim time via use_sim_time); None until /clock has been received."""
        ns = self.node.get_clock().now().nanoseconds
        return ns * 1e-9 if ns > 0 else None

    def _on_js(self, msg) -> None:
        try:
            q = reorder(msg.name, msg.position, self.joints)
            qd = reorder(msg.name, msg.velocity, self.joints) if msg.velocity else [math.nan] * 7
            tau = reorder(msg.name, msg.effort, self.joints) if msg.effort else [math.nan] * 7
        except (KeyError, IndexError):
            return
        hdr = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        clk = self._clock_t if self._clock_t is not None else math.nan
        if hdr > 0.0:
            t, src = hdr, 'joint_state_header'
        else:
            t, src = clk, 'clock_at_receipt'
        s = StateSample(t, src, clk, q, qd, tau)
        with self._lock:
            self._latest = s
            self._seq += 1
            if self._rec is not None:
                self._rec.append(s)
            self._lock.notify_all()
        for cb in self._listeners:
            cb(s)

    def add_listener(self, cb) -> None:
        self._listeners = self._listeners + [cb]

    def remove_listener(self, cb) -> None:
        self._listeners = [x for x in self._listeners if x is not cb]

    def latest(self) -> StateSample | None:
        with self._lock:
            return self._latest

    def clock(self) -> float | None:
        return self._clock_t

    def seq(self) -> int:
        with self._lock:
            return self._seq

    def wait_for_state(self, timeout: float) -> StateSample | None:
        # Wall-clock: give-up timeout only.
        deadline = time.monotonic() + timeout
        with self._lock:
            while self._latest is None:
                rem = deadline - time.monotonic()
                if rem <= 0:
                    return None
                self._lock.wait(rem)
            return self._latest

    def wait_next(self, after_seq: int, timeout: float) -> tuple[int, StateSample | None]:
        # Wall-clock: give-up timeout only.
        deadline = time.monotonic() + timeout
        with self._lock:
            while self._seq <= after_seq:
                rem = deadline - time.monotonic()
                if rem <= 0:
                    return self._seq, None
                self._lock.wait(rem)
            return self._seq, self._latest

    def wait_sim(self, seconds: float, timeout: float) -> bool:
        """Block until `seconds` of sim time pass, measured on /joint_states stamps.

        Returns False on wall-clock give-up (sim paused or not publishing).
        """
        deadline = time.monotonic() + timeout
        with self._lock:
            while self._latest is None:
                rem = deadline - time.monotonic()
                if rem <= 0:
                    return False
                self._lock.wait(rem)
            start = self._latest.t
            while self._latest.t - start < seconds:
                rem = deadline - time.monotonic()
                if rem <= 0:
                    return False
                self._lock.wait(min(rem, 0.5))
            return True

    def start_recording(self) -> None:
        with self._lock:
            self._rec = []

    def stop_recording(self) -> list[StateSample]:
        with self._lock:
            rec, self._rec = self._rec or [], None
            return rec


def call_service(client, request, timeout: float):
    """Blocking call from a worker thread while the executor spins elsewhere. None on timeout."""
    if not client.wait_for_service(timeout_sec=min(timeout, 10.0)):
        return None
    done = threading.Event()
    fut = client.call_async(request)
    fut.add_done_callback(lambda _: done.set())
    # Wall-clock: give-up timeout only.
    if not done.wait(timeout):
        fut.cancel()
        return None
    return fut.result()


_RATE_CACHE: dict[int, float] = {}


def controller_rate(node, cfg: dict, default: float = 100.0) -> float:
    """controller_manager update_rate (Hz), which is also the /joint_states rate."""
    if id(node) in _RATE_CACHE:
        return _RATE_CACHE[id(node)]
    from rcl_interfaces.srv import GetParameters
    cli = node.create_client(GetParameters, f'{cfg["controller_manager"]}/get_parameters')
    res = call_service(cli, GetParameters.Request(names=['update_rate']), 5.0)
    rate = float(res.values[0].integer_value) if res and res.values and res.values[0].integer_value \
        else default
    _RATE_CACHE[id(node)] = rate
    return rate


def wait_future(fut, timeout: float):
    done = threading.Event()
    fut.add_done_callback(lambda _: done.set())
    # Wall-clock: give-up timeout only.
    if not done.wait(timeout):
        return None
    return fut.result()
