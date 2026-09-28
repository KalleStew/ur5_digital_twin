#!/usr/bin/env python3
"""Command Candle Pose (upright) once JTC is available.

URDF initial_value alone is not enough: when the effort-mode JTC activates it
often holds a zero command and lays the arm on the table. This runs after the
spawner chain so spawn ends upright.
"""
from __future__ import annotations

import sys
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from trajectory_msgs.msg import JointTrajectoryPoint


JOINTS = [
    'shoulder_pan_joint1',
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]
# Vertical / clears table — must match URDF initial_value + SRDF candle_pose.
CANDLE = [0.0, 0.0, -1.5708, 0.0, 0.0, 0.0, 0.0]


class HoldCandle(Node):
    def __init__(self) -> None:
        super().__init__(
            'ur5_hold_candle',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
        )
        self.ac = ActionClient(
            self, FollowJointTrajectory, '/ur_manipulator_controller/follow_joint_trajectory'
        )
        self.done = threading.Event()
        self.code = None

    def run(self) -> int:
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < 90.0:
            if self.ac.wait_for_server(timeout_sec=1.0):
                break
        if not self.ac.server_is_ready():
            self.get_logger().error('FollowJointTrajectory not available')
            return 1

        # Two passes: lift, then settle hold.
        for attempt, dur in ((1, 3), (2, 4)):
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = list(JOINTS)
            goal.trajectory.header.stamp.sec = 0
            goal.trajectory.header.stamp.nanosec = 0
            pt = JointTrajectoryPoint()
            pt.positions = list(CANDLE)
            pt.velocities = [0.0] * 7
            pt.time_from_start.sec = int(dur)
            pt.time_from_start.nanosec = int((dur % 1) * 1e9)
            goal.trajectory.points = [pt]
            for n in JOINTS:
                goal.goal_tolerance.append(
                    JointTolerance(name=n, position=0.2, velocity=1.0)
                )

            self.done.clear()
            self.code = None

            def _acc(fut, _a=attempt):
                h = fut.result()
                if not h.accepted:
                    self.get_logger().error(f'Candle #{_a} REJECTED')
                    self.done.set()
                    return

                def _res(rf):
                    self.code = rf.result().result.error_code
                    self.get_logger().info(f'Candle #{_a} error_code={self.code}')
                    self.done.set()

                h.get_result_async().add_done_callback(_res)

            self.ac.send_goal_async(goal).add_done_callback(_acc)
            deadline = time.time() + 45.0
            while not self.done.is_set() and time.time() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
            if not self.done.is_set():
                self.get_logger().error(f'Candle #{attempt} timed out')
                return 1
            time.sleep(0.3)

        self.get_logger().info('Candle upright hold complete')
        return 0


def main() -> int:
    rclpy.init()
    node = HoldCandle()
    try:
        return node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
