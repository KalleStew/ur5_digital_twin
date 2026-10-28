"""Brain B scheme: atomic A->B handoff, joint-state-locked torque loop, watchdogs, B->A exit.

Contains no control law; control_laws.py computes tau. The loop runs once per
joint state sample, which StateCache delivers on an exact sim-time grid (topics.joint_states_rate_hz).
"""
from __future__ import annotations

import datetime as dt
import itertools
import json
import math
import os
import threading
import time
from dataclasses import dataclass, field

import numpy as np
import yaml
from rcl_interfaces.srv import GetParameters
from std_msgs.msg import Float64MultiArray

from .position_control import Brains
from .control_laws import load_controller
from .dynamics import DEFAULT_COULOMB_EPSILON, DEFAULT_MODEL_TERMS, Dynamics, NominalModel
from .position_control import Preflight
from .robot import Model, StateCache, call_service
from .robot import (EXIT_INTERRUPTED, EXIT_PLATFORM, EXIT_RUN_FAILED, EXIT_SAFETY,
                     InputError, PlatformError, RunResult, Scheme)
from .telemetry import (NYQUIST_RATIO_MAX, Recorder, nyquist_artefact, rejection, write_h5)
from .trajectory_io import (MinJerkReference, check_name, data_dir, ensure_writable,
                            load_joint_csv, resolve_waypoints, sha256)

MAX_ATTEMPTS = 3


@dataclass
class Trial:
    trial_id: str
    trajectory: str
    plugin: str
    gains: dict
    perturbation: str
    level: object
    level_index: int
    seed: int
    options: dict = field(default_factory=dict)


@dataclass
class Experiment:
    name: str
    path: str
    trials: list[Trial]
    hold_sec: float
    post_hold_sec: float
    model_terms: tuple = DEFAULT_MODEL_TERMS
    coulomb_epsilon: float | None = DEFAULT_COULOMB_EPSILON
    out_dir: str = ''
    force: bool = False
    raw: dict = field(default_factory=dict)


def load_experiment(path: str, trial: str | None = None, seed: int | None = None) -> Experiment:
    if not os.path.isfile(path):
        from .robot import resolve_file
        try:
            path = resolve_file('ur5_controller', os.path.join('config', 'experiments', path))
        except FileNotFoundError:
            raise InputError(f'experiment file not found: {path}')
    try:
        with open(path) as f:
            raw = yaml.safe_load(f) or {}
    except yaml.YAMLError as e:
        raise InputError(f'{os.path.basename(path)}: invalid YAML ({e.__class__.__name__})')
    for key in ('name', 'trajectories', 'controllers', 'seeds'):
        if key not in raw:
            raise InputError(f'{os.path.basename(path)}: missing key {key!r}')
    name = check_name(str(raw['name']))
    perts = raw.get('perturbations') or [{'type': 'none', 'levels': [0]}]
    seeds = [int(seed)] if seed is not None else [int(s) for s in raw['seeds']]
    trials = []
    for traj, ctrl, pert, s in itertools.product(raw['trajectories'], raw['controllers'], perts, seeds):
        options = {k: v for k, v in pert.items() if k not in ('type', 'levels')}
        for li, level in enumerate(pert.get('levels', [0])):
            stem = os.path.splitext(os.path.basename(traj))[0]
            tid = f'{stem}-{ctrl["plugin"]}-{pert["type"]}{li}-s{s}'
            trials.append(Trial(tid, traj, ctrl['plugin'], ctrl.get('gains', {}), pert['type'],
                                level, li, s, options))
    if trial is not None:
        sel = [t for i, t in enumerate(trials) if t.trial_id == trial or str(i) == str(trial)]
        if not sel:
            raise InputError(f'trial {trial!r} not in {name}; ids: {[t.trial_id for t in trials][:4]}...')
        trials = sel
    terms = raw.get('model_terms', list(DEFAULT_MODEL_TERMS))
    if not isinstance(terms, list):
        raise InputError(f'{os.path.basename(path)}: model_terms must be a list, e.g. [rigid_body, viscous]')
    eps = raw.get('coulomb_epsilon', None if 'model_terms' in raw else DEFAULT_COULOMB_EPSILON)
    return Experiment(name, path, trials, float(raw.get('hold_sec', 1.0)),
                      float(raw.get('post_hold_sec', 1.0)), tuple(terms), eps, raw=raw)


def worst_case_inertia(dyn: Dynamics, model: Model, exp: Experiment, period: float) -> np.ndarray:
    """Smallest M_jj per joint over Candle and every trajectory of the experiment (not only --trial)."""
    m = np.diag(dyn.M(model.candle)).copy()
    for traj in dict.fromkeys(exp.raw.get('trajectories') or [t.trajectory for t in exp.trials]):
        wps = load_joint_csv(resolve_waypoints(traj), model, start=model.candle)
        ref = MinJerkReference(model.candle, wps.times, wps.values, hold_sec=exp.hold_sec)
        for t in np.arange(0.0, ref.duration + period, period):
            m = np.minimum(m, np.diag(dyn.M(np.asarray(ref(t)[0]))))
    return m


class LoopAbort(Exception):
    def __init__(self, code: int, reason: str) -> None:
        super().__init__(reason)
        self.code = code
        self.reason = reason


class TorqueLoop(threading.Thread):
    """One compute+publish per new joint state sample."""

    def __init__(self, scheme: 'TorqueScheme', controller, pert: Perturbation, ref, rec: Recorder):
        super().__init__(name='brain_b_loop', daemon=False)
        self.s = scheme
        self.ctrl = controller
        self.pert = pert
        self.ref = ref
        self.rec = rec
        self.lc = scheme.cfg['brain_b_loop']
        self.stop_evt = threading.Event()
        self.done_evt = threading.Event()
        self.failure: LoopAbort | None = None
        self.t0: float | None = None
        self.recording = False
        self.published = 0
        self.overruns = 0
        self.gaps = 0
        self.saturated = 0
        self.max_latency_ms = 0.0
        hold_q = np.asarray(scheme.model.candle)
        self.hold_ref = (hold_q, np.zeros_like(hold_q), np.zeros_like(hold_q))
        lim = [scheme.model.limits[j] for j in scheme.model.joints]
        margin = self.lc['position_margin_rad']
        self.q_lo = np.array([x.lower + margin for x in lim])
        self.q_hi = np.array([x.upper - margin for x in lim])
        self.qd_max = np.array([x.velocity for x in lim]) * self.lc['velocity_fraction']
        self.tau_max = np.array([x.effort for x in lim])

    def start_run(self, t0: float) -> None:
        self.t0 = t0
        self.recording = True

    def _publish(self, tau) -> None:
        msg = Float64MultiArray()
        msg.data = [float(tau[i]) for i in self.s.cmd_index]
        self.s.pub.publish(msg)

    def run(self) -> None:
        try:
            self._loop()
        except LoopAbort as e:
            self.failure = e
            self._publish(np.zeros(len(self.s.model.joints)))
        except Exception as e:  # noqa: BLE001
            self.failure = LoopAbort(EXIT_RUN_FAILED, f'torque loop error: {e!r}')
            self._publish(np.zeros(len(self.s.model.joints)))
        finally:
            self.done_evt.set()

    def _loop(self) -> None:
        st = self.s.state
        seq = st.seq()
        prev_t = None
        stale = self.lc['stale_ms'] / 1000.0
        dt_nom = 1.0 / self.lc['rate_hz']
        while not self.stop_evt.is_set():
            if self.s.abort_evt.is_set():
                raise LoopAbort(EXIT_INTERRUPTED, 'interrupted')
            seq, smp = st.wait_next(seq, self.lc['clock_stall_sec'])
            if smp is None:
                raise LoopAbort(EXIT_SAFETY, f'no joint states for {self.lc["clock_stall_sec"]}s '
                                             '(sim paused or stopped)')
            # Wall-clock: compute latency is a measurement, not a timing source.
            w0 = time.perf_counter()
            if prev_t is not None:
                gap = smp.t - prev_t
                if gap > stale:
                    raise LoopAbort(EXIT_SAFETY, f'joint states stale: {gap * 1000:.0f} ms gap')
                if gap > 1.5 * dt_nom:
                    self.gaps += 1
            prev_t = smp.t
            q, qd = np.asarray(smp.q), np.asarray(smp.qd)
            if np.any(q < self.q_lo) or np.any(q > self.q_hi):
                j = self.s.model.joints[int(np.argmax((q < self.q_lo) | (q > self.q_hi)))]
                raise LoopAbort(EXIT_SAFETY, f'position watchdog: {j} near its limit')
            if np.any(np.abs(qd) > self.qd_max):
                j = self.s.model.joints[int(np.argmax(np.abs(qd) - self.qd_max))]
                raise LoopAbort(EXIT_SAFETY, f'velocity watchdog: {j} above '
                                             f'{self.lc["velocity_fraction"]:.0%} of its limit')
            if self.t0 is None:
                t_rel, (q_ref, qd_ref, qdd_ref) = 0.0, self.hold_ref
            else:
                t_rel = smp.t - self.t0
                q_ref, qd_ref, qdd_ref = (np.asarray(x) for x in self.ref(t_rel))
            qd_seen = self.pert.measured_velocity(qd)
            tau = np.asarray(self.ctrl.compute(t_rel, q, qd_seen, q_ref, qd_ref, qdd_ref), dtype=float)
            if tau.shape != q.shape or not np.all(np.isfinite(tau)):
                raise LoopAbort(EXIT_SAFETY, f'{self.ctrl.__module__} returned NaN/inf or wrong shape')
            tau_sent = self.pert.command(tau, qd)
            sat = np.abs(tau_sent) > self.tau_max
            if np.any(sat):
                self.saturated += 1
                tau_sent = np.clip(tau_sent, -self.tau_max, self.tau_max)
            self._publish(tau_sent)
            self.published += 1
            latency_ms = (time.perf_counter() - w0) * 1000.0
            self.max_latency_ms = max(self.max_latency_ms, latency_ms)
            if st.seq() > seq:
                self.overruns += 1
            if self.recording:
                self.rec.append(smp.t, smp.q, smp.qd, smp.tau, tau_sent.tolist(), q_ref.tolist(),
                                qd_ref.tolist(), t_source=smp.t_source, qd_used=qd_seen.tolist(),
                                tau_law=tau.tolist(), saturated=sat.tolist(),
                                latency_ms=latency_ms)
                if t_rel >= self.ref.duration + self.s.post_hold:
                    self.done_evt.set()


class TorqueScheme(Scheme):
    def __init__(self, node, cfg: dict, model: Model, state: StateCache, brains: Brains,
                 preflight: Preflight) -> None:
        self.node = node
        self.cfg = cfg
        self.model = model
        self.state = state
        self.brains = brains
        self.pre = preflight
        self.pub = node.create_publisher(Float64MultiArray, cfg['brain_b']['command_topic'], 10)
        self.param_cli = node.create_client(
            GetParameters, f'/{cfg["brain_b"]["controller"]}/get_parameters')
        m = cfg['model']
        self.dyn = Dynamics(model, m['base_link'], m['tip_link'])
        self.abort_evt = threading.Event()
        self.cmd_index: list[int] = []
        self.post_hold = 1.0
        self.loop: TorqueLoop | None = None

    def preflight(self) -> RunResult:
        return self.pre.run()

    def abort(self) -> None:
        self.abort_evt.set()
        self.pre.abort()

    def _resolve_command_order(self) -> None:
        req = GetParameters.Request(names=['joints'])
        res = call_service(self.param_cli, req, 10.0)
        if res is None or not res.values or not res.values[0].string_array_value:
            raise PlatformError(f'cannot read {self.cfg["brain_b"]["controller"]} joints parameter')
        order = list(res.values[0].string_array_value)
        if sorted(order) != sorted(self.model.joints):
            raise PlatformError(f'{self.cfg["brain_b"]["controller"]} joints {order} != model joints')
        self.cmd_index = [self.model.joints.index(j) for j in order]

    def run(self, exp: Experiment) -> RunResult:
        stamp = dt.datetime.now().strftime('%Y%m%d_%H%M%S')
        try:
            exp.out_dir = ensure_writable(exp.out_dir or os.path.join(
                data_dir(), 'telemetry', f'{exp.name}_{stamp}'))
            self._resolve_command_order()
            nominal = NominalModel(self.dyn, exp.model_terms, exp.coulomb_epsilon)
            m_eff = (self._worst_case_inertia(exp)
                     if any(t.perturbation == 'viscous_friction' for t in exp.trials) else None)
            prepared = [self._prepare(exp, t, nominal, m_eff) for t in exp.trials]
        except (InputError, PlatformError) as e:
            return RunResult.fail(e.code, str(e))
        outputs, rows = [], []
        failed: list[tuple[str, RunResult]] = []
        for trial, prep in zip(exp.trials, prepared):
            # A failed trial is recorded and the run moves on; it stops when the arm cannot be
            # brought back to Candle before the next trial, or on interrupt. A trial whose file
            # fails validation is rerun from a fresh controller and seed, up to MAX_ATTEMPTS.
            rejected: list[str] = []
            attempts = 0
            while True:
                pre = self.pre.run()
                if not pre.ok:
                    res = RunResult.fail(pre.code, f'pre-flight: {pre.reason}')
                    break
                attempts += 1
                res = self.run_trial(exp, trial, *prep, rejected=rejected)
                problem = rejection(res.outputs[0]) if res.ok and res.outputs else ''
                if not problem:
                    break
                kept = self._keep_rejected(exp, res.outputs[0], trial.trial_id, attempts)
                rejected.append(problem)
                self.node.get_logger().warning(
                    f'{trial.trial_id}: attempt {attempts} rejected ({problem}); kept {kept}')
                if attempts >= MAX_ATTEMPTS:
                    res = RunResult.fail(EXIT_RUN_FAILED, f'rejected by validation in all {attempts} '
                                                          f'attempts, last: {problem}')
                    break
                prep = self._prepare(exp, trial, nominal, m_eff)
            rows.append((trial.trial_id, res, attempts))
            outputs += res.outputs
            self.node.get_logger().info(f'{trial.trial_id}: {res.reason}')
            if res.ok:
                continue
            failed.append((trial.trial_id, res))
            if not pre.ok or res.code == EXIT_INTERRUPTED or self.abort_evt.is_set():
                self._write_summary(exp, rows)
                return RunResult.fail(res.code, f'{trial.trial_id}: {res.reason} '
                                                f'({len(rows)}/{len(exp.trials)} trials run)', outputs)
        self._write_summary(exp, rows)
        if failed:
            tid, first = failed[0]
            return RunResult.fail(first.code, f'{len(failed)}/{len(rows)} trials failed, first {tid}: '
                                              f'{first.reason} -> {exp.out_dir}', outputs)
        return RunResult.success(f'{len(rows)} trials -> {exp.out_dir}', outputs)

    def _worst_case_inertia(self, exp: Experiment) -> np.ndarray:
        return worst_case_inertia(self.dyn, self.model, exp, 1.0 / self.cfg['brain_b_loop']['rate_hz'])

    def _prepare(self, exp: Experiment, trial: Trial, nominal: NominalModel, m_eff=None):
        path = resolve_waypoints(trial.trajectory)
        wps = load_joint_csv(path, self.model, start=self.model.candle)
        ctrl = load_controller(trial.plugin, self.dyn, trial.gains, self.model.joints, nominal,
                               gain_pose=self.model.candle)
        pert = Perturbation(trial.perturbation, trial.level, trial.seed, self.cfg['model'],
                            len(self.model.joints), trial.options)
        if m_eff is not None:
            period = 1.0 / self.cfg['brain_b_loop']['rate_hz']
            pert.bind(self.model.joints, self.dyn.damping, m_eff, period,
                      ctrl.velocity_feedback_step(m_eff, period))
        return path, wps, ctrl, pert

    def _write_summary(self, exp: Experiment, rows) -> None:
        with open(os.path.join(exp.out_dir, 'summary.csv'), 'w') as f:
            f.write('trial_id,ok,code,attempts,reason\n')
            for tid, r, attempts in rows:
                f.write(f'{tid},{int(r.ok)},{r.code},{attempts},"{r.reason}"\n')

    @staticmethod
    def _keep_rejected(exp: Experiment, path: str, trial_id: str, attempt: int) -> str:
        dest_dir = ensure_writable(os.path.join(data_dir(), 'rejected', os.path.basename(exp.out_dir)))
        dest = os.path.join(dest_dir, f'{trial_id}.attempt{attempt}.h5')
        os.replace(path, dest)
        return dest

    def run_trial(self, exp: Experiment, trial: Trial, path, wps, ctrl, pert,
                  rejected: list[str] | None = None) -> RunResult:
        """One trial from a pre-flighted Candle; the caller runs the pre-flight and validation."""
        np.random.seed(trial.seed)
        ctrl.reset()
        self.post_hold = exp.post_hold_sec
        ref = MinJerkReference(self.model.candle, wps.times, wps.values, hold_sec=exp.hold_sec)
        rec = Recorder(self.model.joints)
        loop = TorqueLoop(self, ctrl, pert, ref, rec)
        self.loop = loop
        failure: LoopAbort | None = None
        # Wall-clock: RTF is a measurement only.
        wall0, sim0 = time.monotonic(), self.state.clock()
        try:
            pert.apply_plant()
            loop.start()
            seq0 = self.state.seq()
            self.state.wait_next(seq0 + 2, 5.0)
            if loop.published < 2 or loop.done_evt.is_set():
                raise LoopAbort(EXIT_PLATFORM, loop.failure.reason if loop.failure else
                                'torque loop did not start; are joint states publishing?')
            self.brains.switch_to('b')
            latest = self.state.latest()
            loop.start_run(latest.t)
            budget = (ref.duration + exp.post_hold_sec) * 20 + 30
            self._supervise(loop, budget)
            failure = loop.failure
        except LoopAbort as e:
            failure = e
        except PlatformError as e:
            failure = LoopAbort(EXIT_PLATFORM, str(e))
        finally:
            back = self._safe_exit(loop, pert)
        wall = time.monotonic() - wall0
        sim1 = self.state.clock()
        rtf = (sim1 - sim0) / wall if sim0 is not None and sim1 is not None and wall > 0 else math.nan
        if back is not None and failure is None:
            failure = back
        post = self.pre.run() if not self.abort_evt.is_set() else RunResult.fail(
            EXIT_INTERRUPTED, 'interrupted')
        out = os.path.join(exp.out_dir, f'{trial.trial_id}.h5')
        metrics = self._metrics(rec)
        artefact = (nyquist_artefact(rec.rows['qd'], metrics['rms_tracking_error_per_joint'],
                                     self.cfg['brain_b_loop']['rate_hz'])
                    if 'rms_tracking_error_per_joint' in metrics else {})
        meta = {
            'scheme': 'torque', 'experiment': exp.name, 'experiment_file': os.path.abspath(exp.path),
            'trial_id': trial.trial_id, 'controller': trial.plugin, 'controller_family': ctrl.family,
            'gains': trial.gains, 'seed': trial.seed,
            'perturbation': {'type': trial.perturbation, 'level': trial.level,
                             'level_index': trial.level_index},
            'perturbation_type': trial.perturbation, 'perturbation_side': pert.side, **pert.metadata(),
            'trajectory_file': os.path.abspath(path),
            'trajectory_sha256': sha256(path), 'rtf': rtf,
            'expected_rate_hz': self.cfg['brain_b_loop']['rate_hz'],
            'hold_sec': exp.hold_sec, 'post_hold_sec': exp.post_hold_sec,
            'overruns': loop.overruns, 'gaps': loop.gaps, 'saturated_samples': loop.saturated,
            'max_compute_latency_ms': loop.max_latency_ms,
            **ctrl.model_metadata(), **ctrl.gain_metadata(),
            'result': 'ok' if failure is None else failure.reason,
            'preflight_after': post.reason, **metrics, **artefact,
            'attempt': len(rejected or []) + 1, 'rejected_attempts': json.dumps(rejected or []),
        }
        if len(rec) >= 2:
            write_h5(out, rec, meta)
        outputs = [out] if len(rec) >= 2 else []
        if failure is not None:
            return RunResult.fail(failure.code, failure.reason, outputs)
        if not post.ok:
            return RunResult.fail(post.code, f'Candle reset after trial: {post.reason}', outputs)
        flag = '' if artefact.get('artefact_valid', True) else (
            f' INVALID: nyquist_ratio {max(artefact["nyquist_ratio"]):.1f} > {NYQUIST_RATIO_MAX}')
        return RunResult.success(
            f'rate={rec.achieved_rate():.1f}Hz rms_err={metrics["rms_tracking_error"]:.4f}rad '
            f'effort={metrics["effort_integral"]:.1f}{flag}', outputs)

    def _supervise(self, loop: TorqueLoop, budget: float) -> None:
        """Wait for the loop to finish; abort if another client takes control authority."""
        # Wall-clock: give-up timeout and authority poll period; trial length is measured on /clock.
        deadline = time.monotonic() + budget
        while not loop.done_evt.wait(1.0):
            if time.monotonic() > deadline:
                raise LoopAbort(EXIT_RUN_FAILED, f'trial did not finish in {budget:.0f}s wall')
            if self.brains.active() != 'b':
                raise LoopAbort(EXIT_SAFETY, 'control authority taken by another client; Brain B stopped')

    def _safe_exit(self, loop: TorqueLoop, pert: Perturbation) -> LoopAbort | None:
        """B->A while the loop still holds the arm, then stop it. Returns a failure, if any."""
        problem = None
        try:
            if self.brains.active() == 'b':
                self.brains.switch_to('a')
        except PlatformError as e:
            problem = LoopAbort(EXIT_PLATFORM, f'B->A handoff failed: {e}')
        loop.stop_evt.set()
        if loop.is_alive():
            loop.join(timeout=5.0)
        try:
            pert.clear_plant()
        except PlatformError as e:
            problem = problem or LoopAbort(EXIT_PLATFORM, str(e))
        return problem

    @staticmethod
    def _metrics(rec: Recorder) -> dict:
        if len(rec) < 2:
            return {'rms_tracking_error': math.nan, 'effort_integral': math.nan}
        t = np.asarray(rec.rows['t'])
        e = np.asarray(rec.rows['q_ref']) - np.asarray(rec.rows['q'])
        tau = np.asarray(rec.rows['tau_commanded'])
        dt_ = np.diff(t, prepend=t[0])
        return {
            'rms_tracking_error': float(np.sqrt(np.mean(np.sum(e ** 2, axis=1)))),
            'rms_tracking_error_per_joint': np.sqrt(np.mean(e ** 2, axis=0)).tolist(),
            'effort_integral': float(np.sum(np.sum(tau ** 2, axis=1) * dt_)),
        }


def hold_under_b(scheme: TorqueScheme, seconds: float | None) -> RunResult:
    """Hold the current pose under Brain B (PD + gravity) for `seconds` sim, or until abort."""
    try:
        scheme._resolve_command_order()
    except PlatformError as e:
        return RunResult.fail(EXIT_PLATFORM, str(e))
    smp = scheme.state.wait_for_state(10.0)
    if smp is None:
        return RunResult.fail(EXIT_PLATFORM, 'no joint states; is the sim running?')
    ctrl = load_controller('pd_gravity', scheme.dyn, scheme.cfg['brain_b_hold_gains'],
                           scheme.model.joints)
    q = list(smp.q)
    zero = [0.0] * len(q)
    dur = seconds if seconds is not None else math.inf

    class _Hold:
        duration = dur

        def __call__(self, t):
            return q, zero, zero

    scheme.post_hold = 0.0
    loop = TorqueLoop(scheme, ctrl, Perturbation('none', 0, 0, scheme.cfg['model']), _Hold(),
                      Recorder(scheme.model.joints))
    loop.hold_ref = (np.asarray(q), np.zeros(len(q)), np.zeros(len(q)))
    failure = None
    try:
        loop.start()
        scheme.state.wait_next(scheme.state.seq() + 2, 5.0)
        if loop.failure:
            raise LoopAbort(loop.failure.code, loop.failure.reason)
        scheme.brains.switch_to('b')
        loop.start_run(scheme.state.latest().t)
        loop.recording = seconds is not None
        scheme._supervise(loop, math.inf)
        failure = loop.failure
    except LoopAbort as e:
        failure = e
    except PlatformError as e:
        failure = LoopAbort(EXIT_PLATFORM, str(e))
    finally:
        back = scheme._safe_exit(loop, Perturbation('none', 0, 0, scheme.cfg['model']))
    failure = failure or back
    if failure is not None and not (failure.code == EXIT_INTERRUPTED and seconds is None):
        return RunResult.fail(failure.code, failure.reason)
    return RunResult.success('held under Brain B, returned to Brain A')

KINDS = {
    'none': 'plant',
    'viscous_friction': 'plant',
    'gain_loss': 'plant',
    'velocity_noise': 'controller',
    'tip_payload': 'plant',
    'tip_force': 'plant',
}

# viscous_friction is fed back from measured qd once per loop period T and held for T.
# Per perturbed joint, with m_eff the worst-case (smallest) M_jj:
# b_p T / m_eff <= VISCOUS_STEP_MAX and (Kd + b_p) T / m_eff <= VISCOUS_COMBINED_MAX.
# Coulomb is not perturbed: its slope at qd = 0 against the plant's stiction gives a
# period-2 stick-slip cycle at any epsilon that keeps it sharp at speed.
VISCOUS_STEP_MAX = 0.5
VISCOUS_COMBINED_MAX = 1.0


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
        if kind == 'viscous_friction':
            opts = options or {}
            extra = sorted(set(opts) - {'joints'})
            if extra:
                raise InputError(f'viscous_friction: unknown keys {extra}; only joints and levels')
            try:
                self.fraction = float(level)
            except (TypeError, ValueError):
                raise InputError('viscous_friction level is a fraction of the URDF damping, e.g. 0.25')
            if self.fraction < 0:
                raise InputError('viscous_friction level must be >= 0')
            self.joints = [str(j) for j in opts.get('joints') or []]
            if not self.joints:
                raise InputError('viscous_friction needs joints: [<joint names>]')
            self.b = np.zeros(dof)
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

    def bind(self, joints, damping, m_eff, period: float, kd_step) -> None:
        """Scale the URDF damping of the perturbed joints; refuse a level the loop cannot hold stably.

        kd_step is the controller's own Kd T / m_eff per joint (velocity_feedback_step).
        """
        if self.kind != 'viscous_friction':
            return
        joints = list(joints)
        unknown = [j for j in self.joints if j not in joints]
        if unknown:
            raise InputError(f'viscous_friction: unknown joints {unknown}; model joints {joints}')
        mask = np.array([j in self.joints for j in joints])
        m_eff = np.asarray(m_eff, dtype=float)
        self.b = np.where(mask, self.fraction * np.asarray(damping, dtype=float), 0.0)
        step = self.b * period / m_eff
        combined = np.asarray(kd_step, dtype=float) + step
        for i, j in enumerate(joints):
            if not mask[i]:
                continue
            if step[i] > VISCOUS_STEP_MAX:
                raise InputError(f'viscous_friction level {self.fraction}: {j} b_p T/m_eff = {step[i]:.3f} '
                                 f'> {VISCOUS_STEP_MAX} (unstable at {1 / period:.0f} Hz)')
            if combined[i] > VISCOUS_COMBINED_MAX:
                raise InputError(f'viscous_friction level {self.fraction}: {j} (Kd + b_p) T/m_eff = '
                                 f'{combined[i]:.3f} > {VISCOUS_COMBINED_MAX} for this controller')
        self.bound = {
            'perturbation_type': self.kind,
            'perturbed_joints': [j for j in joints if j in self.joints],
            'friction_fraction': self.fraction,
            'b_p': self.b.tolist(),
            'velocity_source': 'measured',
            'friction_m_eff_min': m_eff.tolist(),
            'friction_viscous_step': np.where(mask, step, 0.0).tolist(),
            'friction_combined_step': np.where(mask, combined, 0.0).tolist(),
        }

    def describe(self) -> dict:
        return {'type': self.kind, 'level': self.level, 'side': self.side}

    def metadata(self) -> dict:
        return dict(self.bound) if self.kind == 'viscous_friction' else {}

    def measured_velocity(self, qd) -> np.ndarray:
        qd = np.asarray(qd, dtype=float)
        if self.kind == 'velocity_noise':
            return qd + self.rng.normal(0.0, 1.0, self.dof) * self.sigma
        return qd

    def command(self, tau, qd) -> np.ndarray:
        tau = np.asarray(tau, dtype=float)
        if self.kind == 'gain_loss':
            return self.alpha * tau
        if self.kind == 'viscous_friction':
            if not self.bound:
                raise RuntimeError('viscous_friction used before bind()')
            return tau - self.b * np.asarray(qd, dtype=float)
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
