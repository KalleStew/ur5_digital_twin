"""Read-only ROS 2 observer used by the acceptance harness."""
from __future__ import annotations

import math
import threading
import time

import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from controller_manager_msgs.srv import ListControllers, SwitchController
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState

JOINTS = [
    'shoulder_pan_joint1', 'shoulder_pan_joint', 'shoulder_lift_joint',
    'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
]


def stamp_to_sec(stamp: TimeMsg) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


class Sample:
    __slots__ = ('header_t', 'clock_t', 'q', 'qd', 'tau')

    def __init__(self, header_t, clock_t, q, qd, tau):
        self.header_t = header_t
        self.clock_t = clock_t
        self.q = q
        self.qd = qd
        self.tau = tau


class Probe(Node):
    """Buffers /joint_states (reordered by name) and /clock; wraps controller_manager queries."""

    def __init__(self, cm: str = '/controller_manager') -> None:
        super().__init__(
            'ur5_harness_probe',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
        )
        self._lock = threading.Lock()
        self._samples: list[Sample] = []
        self._recording = True
        self.last_clock: float | None = None
        self.clock_msgs = 0
        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Clock, '/clock', self._on_clock, qos)
        self.create_subscription(JointState, '/joint_states', self._on_js, 50)
        self.list_cli = self.create_client(ListControllers, f'{cm}/list_controllers')
        self.switch_cli = self.create_client(SwitchController, f'{cm}/switch_controller')

    def _on_clock(self, msg: Clock) -> None:
        self.last_clock = stamp_to_sec(msg.clock)
        self.clock_msgs += 1

    def _on_js(self, msg: JointState) -> None:
        idx = {n: i for i, n in enumerate(msg.name)}
        if any(j not in idx for j in JOINTS):
            return
        q = [msg.position[idx[j]] for j in JOINTS]
        qd = [msg.velocity[idx[j]] if len(msg.velocity) > idx[j] else math.nan for j in JOINTS]
        tau = [msg.effort[idx[j]] if len(msg.effort) > idx[j] else math.nan for j in JOINTS]
        s = Sample(stamp_to_sec(msg.header.stamp), self.last_clock, q, qd, tau)
        with self._lock:
            if self._recording:
                self._samples.append(s)

    def clear(self) -> None:
        with self._lock:
            self._samples = []

    def samples(self) -> list[Sample]:
        with self._lock:
            return list(self._samples)

    def latest(self) -> Sample | None:
        with self._lock:
            return self._samples[-1] if self._samples else None

    def wait_samples(self, n: int, timeout: float) -> bool:
        # Wall-clock: give-up timeout only.
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if len(self.samples()) >= n:
                return True
            time.sleep(0.05)
        return False

    def wait_sim(self, seconds: float, timeout: float) -> bool:
        """Block until `seconds` of /clock time have elapsed."""
        t0 = time.monotonic()
        while self.last_clock is None and time.monotonic() - t0 < timeout:
            time.sleep(0.05)
        if self.last_clock is None:
            return False
        start = self.last_clock
        while time.monotonic() - t0 < timeout:
            if self.last_clock - start >= seconds:
                return True
            time.sleep(0.02)
        return False

    def _call(self, cli, req, timeout: float = 10.0):
        if not cli.wait_for_service(timeout_sec=timeout):
            return None
        fut = cli.call_async(req)
        t0 = time.monotonic()
        while not fut.done() and time.monotonic() - t0 < timeout:
            time.sleep(0.02)
        return fut.result() if fut.done() else None

    def controllers(self, timeout: float = 10.0) -> dict[str, str] | None:
        res = self._call(self.list_cli, ListControllers.Request(), timeout)
        if res is None:
            return None
        return {c.name: c.state for c in res.controller}

    def switch(self, activate: list[str], deactivate: list[str], timeout: float = 10.0):
        req = SwitchController.Request()
        req.activate_controllers = activate
        req.deactivate_controllers = deactivate
        req.strictness = SwitchController.Request.STRICT
        req.timeout.sec = int(timeout)
        return self._call(self.switch_cli, req, timeout + 2.0)


class ProbeRunner:
    """Owns rclpy init, a Probe node, and a background executor."""

    def __enter__(self) -> Probe:
        rclpy.init()
        self.node = Probe()
        self.exec = MultiThreadedExecutor(num_threads=2)
        self.exec.add_node(self.node)
        self.thread = threading.Thread(target=self.exec.spin, daemon=False)
        self.thread.start()
        return self.node

    def __exit__(self, *exc) -> None:
        self.exec.shutdown()
        self.thread.join(timeout=5.0)
        self.node.destroy_node()
        rclpy.try_shutdown()


def hold_stats(samples: list[Sample], target: list[float]) -> dict:
    """Max |q - target| and RMS |qd| per joint over a sample window."""
    if not samples:
        return {'n': 0}
    max_err = [0.0] * 7
    sq = [0.0] * 7
    for s in samples:
        for j in range(7):
            max_err[j] = max(max_err[j], abs(s.q[j] - target[j]))
            sq[j] += s.qd[j] ** 2
    rms = [math.sqrt(v / len(samples)) for v in sq]
    return {'n': len(samples), 'max_err': max_err, 'rms_qd': rms}
