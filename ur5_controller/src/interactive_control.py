#!/usr/bin/env python3
"""UR5 operator entry point: interactive menu or flags, one stage order for every scheme.

  interactive_control.py
  interactive_control.py --scheme position --mode joint|cartesian --csv <file> [--out <name>]
  interactive_control.py --scheme torque --experiment <file.yaml> [--trial <id>] [--seed <n>]

Exit codes: 0 ok, 1 run failed, 2 input rejected (no motion), 3 platform not ready,
4 safety abort, 130 interrupted.
"""
from __future__ import annotations

import argparse
import glob
import os
import signal
import sys
import threading

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions

from position_control import Brains
from position_control import PositionScheme, PositionSpec, load_spec
from position_control import Preflight
from ur5_common import load_model, load_platform, resolve_file, state_cache
from ur5_common import (EXIT_INPUT, EXIT_INTERRUPTED, EXIT_OK, EXIT_PLATFORM, InputError,
                                   PlatformError, RunResult)
from telemetry import write_h5
from ur5_common import data_dir, shipped_waypoints_dir

MODES = {'1': 'joint', '2': 'cartesian', 'joint': 'joint', 'cartesian': 'cartesian', '': ''}


class App:
    def __init__(self, node: Node, args, loaded=None) -> None:
        self.node = node
        self.args = args
        self.loaded = loaded
        self.cfg = load_platform()
        self.model = load_model(self.cfg)
        self.state = state_cache(node, self.model, self.cfg)
        self.brains = Brains(node, self.cfg)
        self.pre = Preflight(node, self.cfg, self.model, self.state, self.brains)
        self.position = PositionScheme(node, self.cfg, self.model, self.state, self.brains, self.pre)
        self._torque = None
        self.active = None
        self.result = RunResult.fail(EXIT_INTERRUPTED, 'interrupted')
        self.in_prompt = False

    @property
    def torque(self):
        if self._torque is None:
            from torque_control import TorqueScheme
            self._torque = TorqueScheme(self.node, self.cfg, self.model, self.state, self.brains,
                                        self.pre)
        return self._torque

    def abort(self) -> None:
        if self.active is not None:
            self.active.abort()
        self.pre.abort()

    def prompt(self, text: str) -> str:
        self.in_prompt = True
        try:
            return input(text).strip()
        finally:
            self.in_prompt = False

    def main(self) -> None:
        try:
            a = self.args
            if a.scheme is None and not (a.csv or a.mode or a.experiment):
                self.result = self.menu()
            elif (a.scheme or ('torque' if a.experiment else 'position')) == 'torque':
                self.result = self.torque_stages(a.experiment, a.trial, a.seed, a.force)
            else:
                self.result = self.position_stages(PositionSpec(a.csv, MODES[a.mode or ''], a.out, a.force),
                                                   self.loaded)
        except (InputError, PlatformError) as e:
            self.result = RunResult.fail(e.code, str(e))
        except EOFError:
            self.result = RunResult.fail(EXIT_INPUT, 'no input on stdin; pass --csv or --experiment')

    def menu(self) -> RunResult:
        print('UR5 digital twin\n  [1] position (Brain A, waypoint CSV)\n  [2] torque (Brain B, experiment YAML)')
        scheme = self.prompt('Scheme [1]: ') or '1'
        if scheme == '2':
            exps = sorted(glob.glob(os.path.join(resolve_file('ur5_controller', 'config/experiments'), '*.yaml')))
            print('Experiments: ' + ', '.join(os.path.basename(e) for e in exps))
            exp = self.prompt('Experiment file: ')
            trial = self.prompt('Trial id (Enter for all): ') or None
            return self.torque_stages(exp, trial, None, False)
        mode = MODES.get(self.prompt('Mode [1] joint  [2] cartesian (Enter for joint): ') or '1')
        if mode is None:
            raise InputError('mode must be 1 or 2')
        default = 'default_cartesian.csv' if mode == 'cartesian' else 'default_joint.csv'
        files = sorted({os.path.basename(p) for d in (shipped_waypoints_dir(),
                        os.path.join(data_dir(), 'waypoints')) for p in glob.glob(os.path.join(d, '*.csv'))})
        print('Waypoint files: ' + ', '.join(files))
        csv = self.prompt(f'Waypoint file [{default}]: ') or default
        out = self.prompt('Telemetry name (Enter for automatic): ')
        spec = PositionSpec(csv, mode, out, False)
        loaded = self.position.load(spec)
        self.prompt('Press Enter to run pre-flight and execute ')
        return self.position_stages(spec, loaded)

    def position_stages(self, spec: PositionSpec, loaded=None) -> RunResult:
        self.active = self.position
        loaded = loaded or self.position.load(spec)
        pre = self.pre.run()
        self.node.get_logger().info(f'pre-flight: {pre.reason}')
        if not pre.ok:
            return pre
        res = self.position.run(loaded)
        if res.export:
            path, rec, meta = res.export
            write_h5(path, rec, meta)
            res.outputs.append(path)
            self.node.get_logger().info(f'telemetry: {path}')
        if res.code == EXIT_INTERRUPTED:
            return res
        reset = self.pre.run()
        self.node.get_logger().info(f'reset: {reset.reason}')
        if res.ok and not reset.ok:
            return RunResult.fail(reset.code, f'Candle reset failed: {reset.reason}', res.outputs)
        return res

    def torque_stages(self, experiment, trial, seed, force) -> RunResult:
        from torque_control import load_experiment
        if not experiment:
            raise InputError('--experiment <file.yaml> is required for --scheme torque')
        exp = load_experiment(experiment, trial, seed)
        exp.force = force
        self.active = self.torque
        return self.torque.run(exp)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description='UR5 operator entry point', allow_abbrev=False)
    ap.add_argument('--scheme', choices=['position', 'torque'])
    ap.add_argument('--mode', default='', choices=['', '1', '2', 'joint', 'cartesian'])
    ap.add_argument('--csv', default='', help='waypoint CSV: path, or a name in $UR5_DATA_DIR/waypoints or the shipped set')
    ap.add_argument('--out', default='', help='telemetry name; default <csv>_<timestamp>')
    ap.add_argument('--experiment', default='', help='experiment YAML for --scheme torque')
    ap.add_argument('--trial', default=None)
    ap.add_argument('--seed', type=int, default=None)
    ap.add_argument('--force', action='store_true', help='overwrite an existing --out file')
    args, ros_args = ap.parse_known_args(argv)
    if args.scheme == 'torque' and (args.csv or args.mode):
        print('--csv/--mode belong to --scheme position', file=sys.stderr)
        return EXIT_INPUT
    loaded = None
    if args.csv and args.scheme != 'torque':
        try:
            loaded = load_spec(PositionSpec(args.csv, MODES[args.mode or ''], args.out, args.force),
                               load_model(load_platform()))
        except InputError as e:
            print(f'FAIL: {e}', file=sys.stderr)
            return e.code
        except (FileNotFoundError, KeyError, ValueError) as e:
            print(f'cannot load platform model: {e}', file=sys.stderr)
            return EXIT_PLATFORM

    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    node = Node('ur5_interactive_control',
                parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    try:
        app = App(node, args, loaded)
    except (FileNotFoundError, KeyError, ValueError) as e:
        print(f'cannot load platform model: {e}', file=sys.stderr)
        node.destroy_node()
        rclpy.try_shutdown()
        return EXIT_PLATFORM
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    interrupted = threading.Event()

    def on_sigint(signum, frame):
        if interrupted.is_set() or app.in_prompt:
            os._exit(EXIT_INTERRUPTED)
        interrupted.set()
        app.abort()

    signal.signal(signal.SIGINT, on_sigint)
    signal.signal(signal.SIGTERM, on_sigint)
    worker = threading.Thread(target=app.main, name='ur5_worker')
    worker.start()
    while worker.is_alive():
        executor.spin_once(timeout_sec=0.05)
    executor.shutdown()
    node.destroy_node()
    rclpy.try_shutdown()
    res = app.result
    if interrupted.is_set() and res.ok:
        res = RunResult.fail(EXIT_INTERRUPTED, 'interrupted')
    for path in res.outputs:
        print(path)
    stream = sys.stdout if res.ok else sys.stderr
    print(('OK: ' if res.ok else 'FAIL: ') + res.reason, file=stream)
    return EXIT_OK if res.ok else res.code


if __name__ == '__main__':
    sys.exit(main())
