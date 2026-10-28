"""Brain A scheme: position-reference tracking through ur_manipulator_controller.

The reference is joint position, but the hardware command interface is effort: the JTC
closes a PID loop and writes torque. The URDF also exports a position command interface;
nothing in this platform claims it.
"""
from __future__ import annotations

import datetime as dt
import math
import os
import threading
import time
from dataclasses import dataclass

from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance, JointTrajectoryControllerState
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionIK, GetStateValidity
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from ur5_common import (Model, StateCache, call_service, max_abs_diff, reorder,
                    wait_future)
from ur5_common import (EXIT_INTERRUPTED, EXIT_PLATFORM, EXIT_RUN_FAILED, InputError,
                     PlatformError, RunResult, Scheme)
from telemetry import Recorder, telemetry_path
from ur5_common import (Waypoints, check_velocity, detect_kind, load_cartesian_csv,
                            load_joint_csv, resolve_waypoints, rpy_to_quat, segment_samples, sha256)

class Brains:
    def __init__(self, node, cfg: dict) -> None:
        cm = cfg['controller_manager']
        self.a = cfg['brain_a']['controller']
        self.b = cfg['brain_b']['controller']
        self.jsb = cfg['joint_state_broadcaster']
        self.list_cli = node.create_client(ListControllers, f'{cm}/list_controllers')
        self.switch_cli = node.create_client(SwitchController, f'{cm}/switch_controller')

    def states(self, timeout: float = 10.0) -> dict[str, str]:
        res = call_service(self.list_cli, ListControllers.Request(), timeout)
        if res is None:
            raise PlatformError('controller_manager not reachable; is the sim running? (ur5 ctl)')
        return {c.name: c.state for c in res.controller}

    def wait_ready(self, timeout: float, abort=None) -> None:
        """Block until the broadcaster is active and both brains are loaded and configured (bring-up)."""
        ok = ('active', 'inactive')
        # Wall-clock: give-up timeout only; the spawner chain runs on its own schedule.
        deadline = time.monotonic() + timeout
        last = 'controller_manager not reachable'
        while time.monotonic() < deadline and not (abort and abort.is_set()):
            try:
                st = self.states(timeout=5.0)
                if st.get(self.jsb) == 'active' and st.get(self.a) in ok and st.get(self.b) in ok:
                    return
                last = f'controllers {st}'
            except PlatformError as e:
                last = str(e)
            time.sleep(0.5)
        if abort and abort.is_set():
            return
        raise PlatformError(f'bring-up not ready after {timeout:.0f}s: {last}')

    def active(self) -> str | None:
        """'a', 'b', or None when neither brain holds authority."""
        st = self.states()
        a, b = st.get(self.a) == 'active', st.get(self.b) == 'active'
        if a and b:
            raise PlatformError(f'{self.a} and {self.b} are both active')
        return 'a' if a else 'b' if b else None

    def switch_to(self, brain: str, timeout: float = 10.0) -> bool:
        """Give `brain` exclusive authority in one STRICT switch. Returns False if already held."""
        if brain not in ('a', 'b'):
            raise ValueError(f'brain must be a or b, got {brain!r}')
        on, off = (self.a, self.b) if brain == 'a' else (self.b, self.a)
        st = self.states()
        for name in (on, off):
            if name not in st:
                raise PlatformError(f'{name} is not loaded (ur5 ctl)')
        if st[on] == 'active' and st[off] != 'active':
            return False
        if st[on] not in ('active', 'inactive'):
            raise PlatformError(f'{on} is {st[on]}, expected inactive')
        req = SwitchController.Request()
        req.activate_controllers = [on] if st[on] != 'active' else []
        req.deactivate_controllers = [off] if st[off] == 'active' else []
        req.strictness = SwitchController.Request.STRICT
        req.activate_asap = True
        req.timeout.sec = int(timeout)
        res = call_service(self.switch_cli, req, timeout + 5.0)
        if res is None:
            raise PlatformError(f'switch_controller to brain {brain} timed out')
        if not res.ok:
            raise PlatformError(f'switch_controller to brain {brain} refused: {res.message}')
        after = self.states()
        if after.get(on) != 'active' or after.get(off) == 'active':
            raise PlatformError(f'post-condition failed after switch to {brain}: {after}')
        return True

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
            return RunResult.fail(EXIT_PLATFORM, 'no joint states; is the sim running?')
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

ERROR_NAMES = {
    0: 'SUCCESSFUL', -1: 'INVALID_GOAL', -2: 'INVALID_JOINTS', -3: 'OLD_HEADER_TIMESTAMP',
    -4: 'PATH_TOLERANCE_VIOLATED', -5: 'GOAL_TOLERANCE_VIOLATED',
}


@dataclass
class PositionSpec:
    csv: str
    mode: str = ''
    out: str = ''
    force: bool = False


def load_spec(spec: PositionSpec, model: Model) -> tuple[Waypoints, str]:
    """Validate the CSV before any motion; needs no ROS. Returns (waypoints, telemetry output path)."""
    path = resolve_waypoints(spec.csv)
    kind = detect_kind(path)
    if spec.mode and spec.mode != kind:
        raise InputError(f'{os.path.basename(path)} is a {kind} CSV but --mode {spec.mode} was given')
    wps = (load_cartesian_csv(path) if kind == 'cartesian'
           else load_joint_csv(path, model, start=model.candle))
    out = spec.out or (f'{os.path.splitext(os.path.basename(path))[0]}_'
                       f'{dt.datetime.now().strftime("%Y%m%d_%H%M%S")}')
    return wps, telemetry_path(out, spec.force)


class PositionScheme(Scheme):
    def __init__(self, node, cfg: dict, model: Model, state: StateCache, brains: Brains,
                 preflight: Preflight) -> None:
        self.node = node
        self.cfg = cfg
        self.model = model
        self.state = state
        self.brains = brains
        self.pre = preflight
        self.ex = cfg['brain_a_execution']
        self.client = ActionClient(node, FollowJointTrajectory, cfg['brain_a']['action'])
        self.ik_cli = node.create_client(GetPositionIK, cfg['moveit']['compute_ik'])
        self.valid_cli = node.create_client(GetStateValidity, cfg['moveit']['check_state_validity'])
        self._ctrl_state = None
        self._lock = threading.Lock()
        self._goal = None
        self._abort = threading.Event()

    def _on_ctrl_state(self, msg) -> None:
        with self._lock:
            self._ctrl_state = msg

    def preflight(self) -> RunResult:
        return self.pre.run()

    def abort(self) -> None:
        self._abort.set()
        with self._lock:
            g = self._goal
        if g is not None:
            g.cancel_goal_async()
        self.pre.abort()

    # ---- input ----
    def load(self, spec: PositionSpec) -> tuple[Waypoints, str]:
        return load_spec(spec, self.model)

    def solve_cartesian(self, wps: Waypoints) -> list[list[float]]:
        """IK per row, seeded by the previous solution. Any failure aborts the whole run."""
        seed = list(self.model.candle)
        out = []
        for t, v in zip(wps.times, wps.values):
            req = GetPositionIK.Request()
            req.ik_request.group_name = self.cfg['moveit']['group']
            req.ik_request.avoid_collisions = True
            req.ik_request.robot_state = RobotState(
                joint_state=JointState(name=list(self.model.joints), position=seed))
            ps = req.ik_request.pose_stamped
            ps.header.frame_id = self.cfg['moveit']['frame']
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = v[:3]
            q = rpy_to_quat(*v[3:])
            ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = q
            req.ik_request.timeout.sec = 1
            res = call_service(self.ik_cli, req, 10.0)
            if res is None:
                raise PlatformError(f'{self.cfg["moveit"]["compute_ik"]} unavailable; is move_group running?')
            if res.error_code.val != 1:
                raise InputError(f'IK failed at t={t}s (x={v[0]}, y={v[1]}, z={v[2]}); '
                                 f'error {res.error_code.val}; run aborted, no rows skipped')
            sol = res.solution.joint_state
            seed = reorder(sol.name, sol.position, self.model.joints)
            out.append(seed)
        return out

    def check_path(self, start, values) -> None:
        """Reject the run if any sample of the joint-space path is in collision."""
        for i, q in enumerate(segment_samples(start, values, self.ex['collision_step_rad'])):
            req = GetStateValidity.Request()
            req.group_name = self.cfg['moveit']['group']
            req.robot_state = RobotState(joint_state=JointState(name=list(self.model.joints),
                                                                position=list(q)))
            res = call_service(self.valid_cli, req, 10.0)
            if res is None:
                raise PlatformError(f'{self.cfg["moveit"]["check_state_validity"]} unavailable; '
                                    'is move_group running?')
            if not res.valid:
                pair = (f'{res.contacts[0].contact_body_1}/{res.contacts[0].contact_body_2}'
                        if res.contacts else 'constraint')
                raise InputError(f'path in collision ({pair}) at q={[round(x, 3) for x in q]}; '
                                 'run rejected before motion')

    # ---- execution ----
    def run(self, spec) -> RunResult:
        wps, out_path = spec
        try:
            start = list(self.model.candle)
            if wps.kind == 'cartesian':
                values = self.solve_cartesian(wps)
                check_velocity(wps.times, values, self.model, start, os.path.basename(wps.path))
            else:
                values = wps.values
            self.check_path(start, values)
            self.brains.switch_to('a')
        except (InputError, PlatformError) as e:
            return RunResult.fail(e.code, str(e))
        if not self.client.wait_for_server(timeout_sec=15.0):
            return RunResult.fail(EXIT_PLATFORM, f'{self.cfg["brain_a"]["action"]} not available')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(self.model.joints)
        for t, q in zip(wps.times, values):
            pt = JointTrajectoryPoint()
            pt.positions = list(q)
            pt.velocities = [0.0] * len(q)
            seconds_to_duration(pt.time_from_start, t)
            goal.trajectory.points.append(pt)
        for n in self.model.joints:
            goal.goal_tolerance.append(JointTolerance(
                name=n, position=self.ex['goal_position_tol'], velocity=self.ex['goal_velocity_tol']))
        seconds_to_duration(goal.goal_time_tolerance, self.ex['goal_time_tolerance_sec'])
        rec = Recorder(self.model.joints)
        listener = self._make_listener(rec)
        # The trajectory controller publishes its state at the controller rate (1 kHz), so the
        # subscription exists only while a Brain A trajectory runs; it would starve the Brain B loop.
        with self._lock:
            self._ctrl_state = None
        sub = self.node.create_subscription(JointTrajectoryControllerState,
                                            self.cfg['brain_a']['state_topic'], self._on_ctrl_state, 10)
        self.state.add_listener(listener)
        # Wall-clock: RTF is a measurement only.
        wall0, sim0 = time.monotonic(), self.state.clock()
        handle = wait_future(self.client.send_goal_async(goal), 15.0)
        if handle is None or not handle.accepted:
            self.state.remove_listener(listener)
            self.node.destroy_subscription(sub)
            return RunResult.fail(EXIT_RUN_FAILED, 'trajectory goal rejected by the controller')
        with self._lock:
            self._goal = handle
        budget = (wps.times[-1] + self.ex['goal_time_tolerance_sec']) * 20 + 30
        # Wall-clock: give-up timeout; the trajectory itself runs on /clock.
        res = wait_future(handle.get_result_async(), budget)
        with self._lock:
            self._goal = None
        self.state.remove_listener(listener)
        self.node.destroy_subscription(sub)
        wall1, sim1 = time.monotonic(), self.state.clock()
        rtf = ((sim1 - sim0) / (wall1 - wall0) if sim0 is not None and sim1 is not None
               and wall1 > wall0 else math.nan)
        final = self.state.latest()
        err = max_abs_diff(final.q, values[-1]) if final else math.inf
        if self._abort.is_set():
            outcome = RunResult.fail(EXIT_INTERRUPTED, 'interrupted; trajectory cancelled')
        elif res is None:
            handle.cancel_goal_async()
            outcome = RunResult.fail(EXIT_RUN_FAILED, f'no result within {budget:.0f}s; goal cancelled')
        else:
            code = res.result.error_code
            if code != FollowJointTrajectory.Result.SUCCESSFUL:
                outcome = RunResult.fail(EXIT_RUN_FAILED, f'trajectory failed: '
                                         f'{ERROR_NAMES.get(code, code)} {res.result.error_string}')
            elif err > self.ex['goal_position_tol']:
                outcome = RunResult.fail(EXIT_RUN_FAILED, f'final error {err:.4f} rad exceeds '
                                         f'{self.ex["goal_position_tol"]}')
            else:
                outcome = RunResult.success(f'goal reached, final error {err:.4f} rad')
        meta = {
            'scheme': 'position', 'controller': self.cfg['brain_a']['controller'],
            'controller_family': 'joint_trajectory_controller (effort PID)',
            'gains': 'see ur5_moveit_config/config/ros2_controllers.yaml', 'seed': 'none',
            'perturbation': 'none', 'perturbation_side': 'none', 'mode': wps.kind,
            'trajectory_file': os.path.abspath(wps.path), 'trajectory_sha256': sha256(wps.path),
            'rtf': rtf, 'expected_rate_hz': self._expected_rate(),
            'goal_position_tol': self.ex['goal_position_tol'], 'final_error_rad': err,
            'result': outcome.reason,
        }
        if len(rec) >= 2:
            outcome.export = (out_path, rec, meta)
        return outcome

    def _make_listener(self, rec: Recorder):
        joints = self.model.joints

        def on_sample(s) -> None:
            with self._lock:
                cs = self._ctrl_state
            q_ref = qd_ref = tau_cmd = None
            if cs is not None:
                try:
                    q_ref = reorder(cs.joint_names, cs.reference.positions, joints)
                    qd_ref = reorder(cs.joint_names, cs.reference.velocities, joints)
                    tau_cmd = reorder(cs.joint_names, cs.output.effort, joints)
                except (KeyError, IndexError):
                    pass
            rec.append(s.t, s.q, s.qd, s.tau, tau_cmd, q_ref, qd_ref, t_source=s.t_source)
        return on_sample

    def _expected_rate(self) -> float:
        return float(self.cfg['topics']['joint_states_rate_hz'])
