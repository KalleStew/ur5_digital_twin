#!/usr/bin/env python3
"""Command-line front end for multi_waypoint_planner (topics /planner_command, /planner_status).

  planner_cli.py                                    interactive; lines go to the planner
  planner_cli.py --export <name>                    interactive; 'export joints' saves <name>.csv
  planner_cli.py --points <file.points.yaml> --export <name> [--force]
                                                    headless: clear, add points, plan, export

Points file (world frame, metres; optional roll pitch yaw in radians):
  points:
    - [0.35, 0.10, 1.60]
    - {xyz: [0.35, -0.10, 1.60], rpy: [0.0, 0.0, 0.0]}
  return_home: true

Exit codes: 0 exported, 1 plan or export refused, 2 bad input, 3 planner not reachable, 130 interrupted.
"""
from __future__ import annotations

import argparse
import math
import os
import queue
import signal
import sys
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

READY_TIMEOUT_SEC = 90.0
REPLY_TIMEOUT_SEC = 60.0


class InputError(Exception):
    pass


def load_points(path: str):
    import yaml
    try:
        with open(path) as f:
            doc = yaml.safe_load(f) or {}
    except OSError as e:
        raise InputError(f'cannot read {path}: {e.strerror}')
    except yaml.YAMLError as e:
        raise InputError(f'{path}: not valid YAML ({str(e).splitlines()[0]})')
    pts = doc.get('points') if isinstance(doc, dict) else None
    if not isinstance(pts, list) or not pts:
        raise InputError(f'{path}: needs a non-empty "points:" list')
    out = []
    for i, p in enumerate(pts, 1):
        if isinstance(p, dict):
            vals = list(p.get('xyz', [])) + list(p.get('rpy', []))
        else:
            vals = list(p) if isinstance(p, (list, tuple)) else []
        try:
            vals = [float(v) for v in vals]
        except (TypeError, ValueError):
            raise InputError(f'{path}: point {i} has a non-numeric value')
        if len(vals) not in (3, 6) or not all(math.isfinite(v) for v in vals):
            raise InputError(f'{path}: point {i} needs [x, y, z] or [x, y, z, roll, pitch, yaw]')
        out.append(vals)
    return out, bool(doc.get('return_home', False))


class PlannerCli(Node):
    def __init__(self) -> None:
        super().__init__('planner_cli',
                         parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.pub = self.create_publisher(String, '/planner_command', 10)
        self.replies: queue.Queue[str] = queue.Queue()
        self.echo = False
        self.create_subscription(String, '/planner_status', self.on_status, 10)

    def on_status(self, msg: String) -> None:
        self.replies.put(msg.data)
        if self.echo:
            print(f'\n[planner] {msg.data}\nplanner> ', end='', flush=True)

    def connected(self) -> bool:
        return (self.pub.get_subscription_count() > 0
                and self.count_publishers('/planner_status') > 0)

    def send(self, command: str) -> None:
        self.pub.publish(String(data=command))

    def request(self, command: str, stop: threading.Event) -> tuple[bool, str]:
        """Publish one command and wait for its /planner_status reply."""
        while not self.replies.empty():
            self.replies.get_nowait()
        self.send(command)
        key = command.split()[0] if not command.startswith(('export', 'return')) else ' '.join(command.split()[:2])
        # Wall-clock give-up: the planner answers in wall time even if sim time stalls.
        deadline = time.monotonic() + REPLY_TIMEOUT_SEC
        while time.monotonic() < deadline and not stop.is_set():
            try:
                r = self.replies.get(timeout=0.2)
            except queue.Empty:
                continue
            ok, _, rest = r.partition(' ')
            if rest.startswith(key + ':') or rest.startswith('command:'):
                return ok == 'OK', rest
        return False, f'{key}: no reply within {REPLY_TIMEOUT_SEC:.0f} s'


def wait_connected(cli: PlannerCli, stop: threading.Event) -> bool:
    # Wall-clock give-up for planner start-up (move_group connection).
    deadline = time.monotonic() + READY_TIMEOUT_SEC
    while time.monotonic() < deadline and not stop.is_set():
        if cli.connected():
            return True
        time.sleep(0.2)
    return False


def headless(cli: PlannerCli, args, stop: threading.Event, out: dict) -> None:
    points, home = out.pop('points')
    if not wait_connected(cli, stop):
        out['code'], out['msg'] = 3, 'multi_waypoint_planner not reachable on /planner_command (ur5 planner)'
        return
    deadline = time.monotonic() + READY_TIMEOUT_SEC
    while True:
        ok, msg = cli.request('clear', stop)
        if ok or stop.is_set() or time.monotonic() > deadline:
            break
        time.sleep(1.0)
    if not ok:
        out['code'], out['msg'] = 3, f'planner not ready: {msg}'
        return
    steps = [('point ' + ' '.join(f'{v:.6f}' for v in p)) for p in points]
    if home:
        steps.append('return home')
    steps += ['plan', f'export joints {args.export}' + (' force' if args.force else '')]
    for s in steps:
        if stop.is_set():
            out['code'], out['msg'] = 130, 'interrupted'
            return
        ok, msg = cli.request(s, stop)
        print(('+ ' if ok else '') + msg, file=sys.stdout if ok else sys.stderr)
        if not ok:
            out['code'], out['msg'] = 1, msg
            return
    out['code'], out['msg'] = 0, msg


def interactive(cli: PlannerCli, args, stop: threading.Event, out: dict) -> None:
    if not wait_connected(cli, stop):
        out['code'], out['msg'] = 3, 'multi_waypoint_planner not reachable on /planner_command (ur5 planner)'
        return
    cli.echo = True
    print('Planner connected. Commands: point x y z [r p y], plan, execute, return home, '
          'export joints <name>, export cartesian <name>, clear, quit')
    print('RViz Publish Point also adds waypoints.')
    out['code'], out['msg'] = 0, 'planner session closed'
    while not stop.is_set():
        try:
            line = input('planner> ').strip()
        except EOFError:
            break
        if not line:
            continue
        if line.lower() in ('quit', 'exit'):
            break
        words = line.split()
        if args.export and words[:2] in (['export', 'joints'], ['export', 'cartesian']) and len(words) == 2:
            line += f' {args.export}' + (' force' if args.force else '')
        cli.send(line)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--points', default='')
    ap.add_argument('--export', default='', help='output name in $UR5_DATA_DIR/waypoints')
    ap.add_argument('--force', action='store_true')
    args, ros_args = ap.parse_known_args()
    out: dict = {'code': 130, 'msg': 'interrupted'}
    if args.points:
        if not args.export:
            print('ERROR: --points needs --export <name>', file=sys.stderr)
            return 2
        try:
            out['points'] = load_points(args.points)
        except InputError as e:
            print(f'ERROR: {e}', file=sys.stderr)
            return 2
    elif not sys.stdin.isatty():
        print('ERROR: no terminal on stdin; pass --points <file> --export <name>', file=sys.stderr)
        return 2

    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    cli = PlannerCli()
    stop = threading.Event()

    def on_sigint(*_):
        if not args.points:
            os._exit(130)
        stop.set()

    signal.signal(signal.SIGINT, on_sigint)
    target = headless if args.points else interactive
    worker = threading.Thread(target=target, args=(cli, args, stop, out), daemon=not args.points)
    worker.start()
    ex = SingleThreadedExecutor()
    ex.add_node(cli)
    while worker.is_alive():
        ex.spin_once(timeout_sec=0.05)
    ex.shutdown()
    cli.destroy_node()
    rclpy.try_shutdown()
    if out['code'] != 0:
        print(f'ERROR: {out["msg"]}', file=sys.stderr)
    return out['code']


if __name__ == '__main__':
    sys.exit(main())
