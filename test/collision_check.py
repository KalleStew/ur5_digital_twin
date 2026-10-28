#!/usr/bin/env python3
"""Check a waypoint CSV against the MoveIt planning scene (needs move_group running).

The path between zero-velocity waypoints is a straight line in joint space, so the
segment is sampled linearly. Exit 0 when every sample is valid.
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import sys
import threading
import time

import rclpy
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionIK, GetStateValidity
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState

def _arm():
    repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    pkg = os.path.join(repo, 'ur5_controller')
    if pkg not in sys.path:
        sys.path.insert(0, pkg)
    from ur5_controller.robot import load_model, load_platform
    return load_model(load_platform())


_MODEL = _arm()
JOINTS = list(_MODEL.joints)
CANDLE = list(_MODEL.candle)
STEP = 0.02


def rpy_to_quat(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r / 2), math.sin(r / 2), math.cos(p / 2),
                              math.sin(p / 2), math.cos(y / 2), math.sin(y / 2))
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


class Checker(Node):
    def __init__(self):
        super().__init__('ur5_collision_check',
                         parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.valid = self.create_client(GetStateValidity, '/check_state_validity')
        self.ik = self.create_client(GetPositionIK, '/compute_ik')

    def call(self, cli, req, timeout=10.0):
        if not cli.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f'{cli.srv_name} unavailable')
        fut = cli.call_async(req)
        t0 = time.monotonic()
        while not fut.done() and time.monotonic() - t0 < timeout:
            time.sleep(0.005)
        if not fut.done():
            raise RuntimeError(f'{cli.srv_name} timed out')
        return fut.result()

    def state_ok(self, q):
        req = GetStateValidity.Request()
        req.group_name = 'ur_manipulator'
        req.robot_state = RobotState(joint_state=JointState(name=JOINTS, position=list(q)))
        res = self.call(self.valid, req)
        return res.valid, [f'{c.contact_body_1}/{c.contact_body_2}' for c in res.contacts]

    def solve(self, xyz, quat, seed):
        req = GetPositionIK.Request()
        req.ik_request.group_name = 'ur_manipulator'
        req.ik_request.pose_stamped.header.frame_id = 'world'
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = xyz
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = quat
        req.ik_request.robot_state = RobotState(joint_state=JointState(name=JOINTS, position=seed))
        req.ik_request.timeout.sec = 1
        res = self.call(self.ik, req)
        if res.error_code.val != 1:
            return None
        m = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [m[j] for j in JOINTS]


def load(path, kind, node):
    with open(path) as f:
        rows = [r for r in csv.reader(f) if r]
    hdr = [h.strip() for h in rows[0]]
    out = []
    seed = list(CANDLE)
    for r in rows[1:]:
        if kind == 'joint':
            out.append([float(r[hdr.index(j)]) for j in JOINTS])
        else:
            v = [float(x) for x in r[1:7]]
            q = node.solve(v[:3], rpy_to_quat(*v[3:]), seed)
            if q is None:
                raise RuntimeError(f'IK failed for row t={r[0]}')
            out.append(q)
            seed = q
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csv')
    ap.add_argument('--kind', choices=['joint', 'cartesian'], default='joint')
    ap.add_argument('--start', default='candle', help="'candle' or comma list of 7 values")
    a = ap.parse_args()
    rclpy.init()
    node = Checker()
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    result = {'rc': 1}

    def work():
        try:
            pts = load(a.csv, a.kind, node)
            start = CANDLE if a.start == 'candle' else [float(x) for x in a.start.split(',')]
            prev = start
            checked = 0
            for i, q in enumerate(pts):
                n = max(1, int(math.ceil(max(abs(x - y) for x, y in zip(q, prev)) / STEP)))
                for k in range(1, n + 1):
                    s = [p + (c - p) * k / n for p, c in zip(prev, q)]
                    ok, contacts = node.state_ok(s)
                    checked += 1
                    if not ok:
                        print(f'COLLISION segment {i} sample {k}/{n}: {contacts[:3]} '
                              f'q={[round(x, 3) for x in s]}')
                        return
                prev = q
            print(f'collision-free: {len(pts)} waypoints, {checked} states')
            result['rc'] = 0
        except Exception as e:  # noqa: BLE001
            print(f'ERROR: {e}')
            result['rc'] = 2
        finally:
            ex.shutdown(timeout_sec=0)

    t = threading.Thread(target=work)
    t.start()
    try:
        ex.spin()
    except Exception:  # noqa: BLE001
        pass
    t.join()
    node.destroy_node()
    rclpy.try_shutdown()
    return result['rc']


if __name__ == '__main__':
    sys.exit(main())
