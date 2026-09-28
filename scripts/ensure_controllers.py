#!/usr/bin/env python3
"""Ensure JSB + JTC are active, FEC configured inactive, then hold Candle Pose.

Handles the common gz_ros2_control case where controllers are already loaded
(or partially configured) and `spawner` would exit with 'Failed loading'.
"""
from __future__ import annotations

import sys
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from controller_manager_msgs.srv import (
    ConfigureController,
    ListControllers,
    LoadController,
    SwitchController,
)
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
CANDLE = [0.0, 0.0, -1.5708, 0.0, 0.0, 0.0, 0.0]


class Ensure(Node):
    def __init__(self) -> None:
        super().__init__(
            'ur5_ensure_controllers',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
        )
        self._cm = '/controller_manager'
        self.list_cli = self.create_client(ListControllers, f'{self._cm}/list_controllers')
        self.load_cli = self.create_client(LoadController, f'{self._cm}/load_controller')
        self.cfg_cli = self.create_client(ConfigureController, f'{self._cm}/configure_controller')
        self.sw_cli = self.create_client(SwitchController, f'{self._cm}/switch_controller')
        self.ac = ActionClient(
            self, FollowJointTrajectory, '/ur_manipulator_controller/follow_joint_trajectory'
        )
        self.done = threading.Event()
        self.result_code = None

    def _wait(self, cli, name: str, timeout: float = 90.0) -> bool:
        t0 = time.time()
        while rclpy.ok() and not cli.wait_for_service(timeout_sec=1.0):
            if time.time() - t0 > timeout:
                self.get_logger().error(f'Timeout waiting for {name}')
                return False
        return True

    def _call(self, cli, req, timeout: float = 45.0):
        fut = cli.call_async(req)
        t0 = time.time()
        while rclpy.ok() and not fut.done() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not fut.done():
            raise RuntimeError('service call timeout')
        return fut.result()

    def list_map(self) -> dict:
        if not self._wait(self.list_cli, 'list_controllers'):
            return {}
        res = self._call(self.list_cli, ListControllers.Request())
        out = {c.name: c.state for c in res.controller}
        self.get_logger().info(f'controllers: {out}')
        return out

    def ensure(self, name: str) -> str:
        """Return final state string for controller."""
        states = self.list_map()
        st = states.get(name)

        if st is None:
            self.get_logger().info(f'Loading {name}')
            if not self._wait(self.load_cli, 'load_controller'):
                raise RuntimeError('no load_controller')
            req = LoadController.Request()
            req.name = name
            res = self._call(self.load_cli, req)
            if not res.ok:
                # Race: may have been loaded by peer — refresh
                states = self.list_map()
                st = states.get(name)
                if st is None:
                    raise RuntimeError(f'load {name} failed and still missing')
            else:
                states = self.list_map()
                st = states.get(name)

        if st == 'unconfigured':
            self.get_logger().info(f'Configuring {name}')
            if not self._wait(self.cfg_cli, 'configure_controller'):
                raise RuntimeError('no configure_controller')
            req = ConfigureController.Request()
            req.name = name
            res = self._call(self.cfg_cli, req)
            if not res.ok:
                self.get_logger().warn(f'configure {name} returned ok=False')
            states = self.list_map()
            st = states.get(name)

        return st or 'unknown'

    def switch(self, activate: list[str], deactivate: list[str]) -> None:
        if not activate and not deactivate:
            return
        if not self._wait(self.sw_cli, 'switch_controller'):
            raise RuntimeError('no switch_controller')
        req = SwitchController.Request()
        req.activate_controllers = activate
        req.deactivate_controllers = deactivate
        # BEST_EFFORT: don't fail if one is already in the requested state
        req.strictness = SwitchController.Request.BEST_EFFORT
        req.activate_asap = True
        res = self._call(self.sw_cli, req, timeout=60.0)
        self.get_logger().info(
            f'switch activate={activate} deactivate={deactivate} -> ok={res.ok} {res.message}'
        )
        if not res.ok and activate:
            # Retry activating only those still inactive
            states = self.list_map()
            still = [n for n in activate if states.get(n) != 'active']
            if still:
                req2 = SwitchController.Request()
                req2.activate_controllers = still
                req2.deactivate_controllers = []
                req2.strictness = SwitchController.Request.BEST_EFFORT
                req2.activate_asap = True
                res2 = self._call(self.sw_cli, req2, timeout=60.0)
                self.get_logger().info(f'retry activate {still} -> ok={res2.ok} {res2.message}')
                if not res2.ok:
                    raise RuntimeError(f'switch failed: {res2.message}')

    def go_candle(self) -> None:
        if not self.ac.wait_for_server(timeout_sec=45.0):
            raise RuntimeError('FollowJointTrajectory unavailable')
        # Send twice: first pulls toward candle, second settles hold.
        for attempt, dur in enumerate((5, 4), start=1):
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = list(JOINTS)
            goal.trajectory.header.stamp.sec = 0
            goal.trajectory.header.stamp.nanosec = 0
            pt = JointTrajectoryPoint()
            pt.positions = list(CANDLE)
            pt.velocities = [0.0] * 7
            pt.time_from_start.sec = dur
            goal.trajectory.points = [pt]
            for n in JOINTS:
                goal.goal_tolerance.append(
                    JointTolerance(name=n, position=0.15, velocity=0.8)
                )

            self.done.clear()
            self.result_code = None

            def _acc(fut, _attempt=attempt):
                h = fut.result()
                if not h.accepted:
                    self.get_logger().error(f'Candle goal #{_attempt} REJECTED')
                    self.done.set()
                    return

                def _res(rf):
                    r = rf.result()
                    self.result_code = r.result.error_code
                    self.get_logger().info(
                        f'Candle #{_attempt} error_code={self.result_code}'
                    )
                    self.done.set()

                h.get_result_async().add_done_callback(_res)

            self.ac.send_goal_async(goal).add_done_callback(_acc)
            t0 = time.time()
            while not self.done.is_set() and time.time() - t0 < 60.0:
                rclpy.spin_once(self, timeout_sec=0.05)
            if not self.done.is_set():
                raise RuntimeError(f'Candle goal #{attempt} timed out')
            time.sleep(0.5)


def main() -> int:
    rclpy.init()
    node = Ensure()
    try:
        # Wait for CM
        t0 = time.time()
        while time.time() - t0 < 90.0:
            try:
                if node.list_map() is not None and node._wait(node.list_cli, 'list', timeout=3.0):
                    break
            except Exception:  # noqa: BLE001
                pass
            time.sleep(0.5)

        for name in (
            'joint_state_broadcaster',
            'ur_manipulator_controller',
            'forward_effort_controller',
        ):
            try:
                node.ensure(name)
            except Exception as exc:  # noqa: BLE001
                node.get_logger().warn(f'{name}: {exc}')

        states = node.list_map()
        to_activate = [
            n
            for n in ('joint_state_broadcaster', 'ur_manipulator_controller')
            if states.get(n) != 'active'
        ]
        to_deactivate = [
            n for n in ('forward_effort_controller',) if states.get(n) == 'active'
        ]
        node.switch(to_activate, to_deactivate)

        states = node.list_map()
        if states.get('ur_manipulator_controller') != 'active':
            raise RuntimeError(f'JTC not active after switch: {states}')

        node.go_candle()
        node.get_logger().info('Candle Pose hold complete')
        return 0
    except Exception as exc:  # noqa: BLE001
        node.get_logger().error(f'ensure_controllers failed: {exc}')
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
