#!/usr/bin/env python3
"""Candle pre-flight: Brain A active, move to candle_pose, verify arrival and settle. Exit 0 on success.

  preflight.py [--wait-for-bringup SEC]   wait for the spawner chain first (used by bringup.launch.py)
"""
import argparse
import signal
import sys
import threading

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions

from position_control import Brains
from position_control import Preflight
from ur5_common import load_model, load_platform, state_cache
from ur5_common import EXIT_INTERRUPTED, EXIT_PLATFORM, PlatformError, RunResult


def main() -> int:
    ap = argparse.ArgumentParser(description='Candle pre-flight')
    ap.add_argument('--wait-for-bringup', type=float, default=0.0, metavar='SEC')
    args, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    node = Node('ur5_preflight',
                parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    cfg = load_platform()
    model = load_model(cfg)
    state = state_cache(node, model, cfg)
    brains = Brains(node, cfg)
    pre = Preflight(node, cfg, model, state, brains)
    out = {'res': RunResult.fail(EXIT_INTERRUPTED, 'interrupted')}
    stop = threading.Event()

    def work():
        try:
            if args.wait_for_bringup > 0:
                brains.wait_ready(args.wait_for_bringup, abort=stop)
                if stop.is_set():
                    return
            out['res'] = pre.run()
        except PlatformError as e:
            out['res'] = RunResult.fail(EXIT_PLATFORM, str(e))

    def on_sigint(*_):
        stop.set()
        pre.abort()

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
    if stop.is_set() and res.ok:
        res = RunResult.fail(EXIT_INTERRUPTED, 'interrupted')
    print(('OK: ' if res.ok else 'FAIL: ') + res.reason, file=sys.stdout if res.ok else sys.stderr)
    return 0 if res.ok else res.code


if __name__ == '__main__':
    sys.exit(main())
