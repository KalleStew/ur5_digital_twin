#!/usr/bin/env python3
"""Atomic control-authority switch.

  switch_brain.py a            Brain A (JTC) takes authority and holds the current pose.
  switch_brain.py b [--sec N]  Brain B holds the current pose with PD + gravity for N s
                               (default: until Ctrl-C), then hands back to Brain A.

Brain B is never left active without a torque stream: a bare effort controller commands
zero torque and the arm falls.
"""
import argparse
import math
import signal
import sys
import threading

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions

from ur5_controller.brain import Brains
from ur5_controller.preflight import Preflight
from ur5_controller.robot import load_model, load_platform, state_cache
from ur5_controller.scheme import EXIT_INPUT, EXIT_PLATFORM, PlatformError, RunResult


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('brain', nargs='?', default='')
    ap.add_argument('--sec', type=float, default=None, help='Brain B hold duration (sim seconds)')
    args, ros_args = ap.parse_known_args()
    if args.brain not in ('a', 'b', 'A', 'B'):
        print('usage: switch_brain.py a | b [--sec N]', file=sys.stderr)
        return EXIT_INPUT
    brain = args.brain.lower()
    if args.sec is not None and (not math.isfinite(args.sec) or args.sec <= 0):
        print('--sec must be a positive number', file=sys.stderr)
        return EXIT_INPUT
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    node = Node('ur5_switch_brain',
                parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    cfg = load_platform()
    model = load_model(cfg)
    state = state_cache(node, model, cfg)
    brains = Brains(node, cfg)
    out = {'res': None}
    scheme = None

    if brain == 'b':
        from ur5_controller.torque_control import TorqueScheme
        scheme = TorqueScheme(node, cfg, model, state, brains, Preflight(node, cfg, model, state, brains))

    def work():
        try:
            if brain == 'a':
                changed = brains.switch_to('a')
                out['res'] = RunResult.success(
                    f'{cfg["brain_a"]["controller"]} active' + ('' if changed else ' (already)'))
            else:
                from ur5_controller.torque_control import hold_under_b
                out['res'] = hold_under_b(scheme, args.sec)
        except PlatformError as e:
            out['res'] = RunResult.fail(EXIT_PLATFORM, str(e))

    def on_sigint(*_):
        if scheme is not None:
            scheme.abort_evt.set()

    signal.signal(signal.SIGINT, on_sigint)
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    t = threading.Thread(target=work)
    t.start()
    while t.is_alive():
        ex.spin_once(timeout_sec=0.05)
    ex.shutdown()
    node.destroy_node()
    rclpy.try_shutdown()
    res = out['res']
    print(('OK: ' if res.ok else 'FAIL: ') + res.reason, file=sys.stdout if res.ok else sys.stderr)
    return 0 if res.ok else res.code


if __name__ == '__main__':
    sys.exit(main())
