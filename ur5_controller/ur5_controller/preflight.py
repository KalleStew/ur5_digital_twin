"""Candle pre-flight: Brain A active, move to candle_pose, verify arrival and settle."""
from __future__ import annotations

import math
import threading

from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectoryPoint

from .brain import Brains
from .robot import Model, StateCache, max_abs_diff, wait_future
from .scheme import EXIT_PLATFORM, EXIT_RUN_FAILED, PlatformError, RunResult


def seconds_to_duration(msg, seconds: float) -> None:
    sec = int(seconds)
    nsec = int(round((seconds - sec) * 1e9))
    if nsec >= 1_000_000_000:
        sec, nsec = sec + 1, nsec - 1_000_000_000
    msg.sec, msg.nanosec = sec, nsec


def settle_check(state: StateCache, target, cfg_candle: dict, seconds: float | None = None):
    """Sample `seconds` of sim time. Returns (ok, max|q-target|, max per-joint RMS qd)."""
    seconds = cfg_candle['settle_sec'] if seconds is None else seconds
    state.start_recording()
    finished = state.wait_sim(seconds, timeout=seconds * 20 + 10)
    rec = state.stop_recording()
    if not finished or not rec:
        return False, math.inf, math.inf
    err = max(max_abs_diff(s.q, target) for s in rec)
    rms = max(math.sqrt(sum(s.qd[j] ** 2 for s in rec) / len(rec)) for j in range(len(target)))
    ok = err < cfg_candle['max_error_rad'] and rms < cfg_candle['max_rms_velocity']
    return ok, err, rms


class Preflight:
    def __init__(self, node, cfg: dict, model: Model, state: StateCache, brains: Brains) -> None:
        self.node = node
        self.cfg = cfg
        self.model = model
        self.state = state
        self.brains = brains
        self.client = ActionClient(node, FollowJointTrajectory, cfg['brain_a']['action'])
        self._goal = None
        self._lock = threading.Lock()

    def run(self) -> RunResult:
        c = self.cfg['candle']
        try:
            self.brains.switch_to('a')
        except PlatformError as e:
            return RunResult.fail(EXIT_PLATFORM, str(e))
        if self.state.wait_for_state(10.0) is None:
            return RunResult.fail(EXIT_PLATFORM, 'no /joint_states; is the sim running?')
        if not self.client.wait_for_server(timeout_sec=15.0):
            return RunResult.fail(EXIT_PLATFORM, f'{self.cfg["brain_a"]["action"]} not available')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(self.model.joints)
        pt = JointTrajectoryPoint()
        pt.positions = list(self.model.candle)
        pt.velocities = [0.0] * len(self.model.joints)
        seconds_to_duration(pt.time_from_start, c['move_sec'])
        goal.trajectory.points = [pt]
        for n in self.model.joints:
            goal.goal_tolerance.append(
                JointTolerance(name=n, position=c['max_error_rad'], velocity=0.0))
        seconds_to_duration(goal.goal_time_tolerance, c['goal_time_tolerance_sec'])
        handle = wait_future(self.client.send_goal_async(goal), 15.0)
        if handle is None or not handle.accepted:
            return RunResult.fail(EXIT_RUN_FAILED, 'Candle goal rejected by the trajectory controller')
        with self._lock:
            self._goal = handle
        res = wait_future(handle.get_result_async(), c['timeout_sec'])
        with self._lock:
            self._goal = None
        if res is None:
            return RunResult.fail(EXIT_RUN_FAILED, f'Candle goal did not finish in {c["timeout_sec"]}s')
        code = res.result.error_code
        if code != FollowJointTrajectory.Result.SUCCESSFUL:
            return RunResult.fail(EXIT_RUN_FAILED,
                                  f'Candle goal failed: error_code={code} {res.result.error_string}')
        ok, err, rms = settle_check(self.state, self.model.candle, c)
        if not ok:
            return RunResult.fail(
                EXIT_RUN_FAILED,
                f'Candle not settled: max|dq|={err:.4f} rad (limit {c["max_error_rad"]}), '
                f'RMS qd={rms:.2e} rad/s (limit {c["max_rms_velocity"]})')
        return RunResult.success(f'Candle reached: max|dq|={err:.4f} rad, RMS qd={rms:.1e} rad/s')

    def abort(self) -> None:
        with self._lock:
            g = self._goal
        if g is not None:
            g.cancel_goal_async()
