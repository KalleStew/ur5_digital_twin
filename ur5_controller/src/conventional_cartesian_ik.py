#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState

from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState as SensorJointState

import csv, os, threading, time
import numpy as np
import h5py

class CartesianIKController(Node):
    def __init__(self):
        super().__init__('cartesian_ik_controller', 
                         parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        
        self.action_client = ActionClient(self, FollowJointTrajectory, '/ur_manipulator_controller/follow_joint_trajectory')
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.state_sub = self.create_subscription(JointState, '/joint_states', self.sensor_callback, 10)
        
        self.is_recording = False 
        self.telemetry_data = []
        self.joint_names = [
            'shoulder_pan_joint1', 'shoulder_pan_joint', 'shoulder_lift_joint',
            'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        
        self.current_joint_positions = [0.0] * 7
        self.state_received = False

        self.control_thread = threading.Thread(target=self.run_control_scheme)
        self.control_thread.start()

    def solve_ik(self, x, y, z, qx, qy, qz, qw, seed_positions):
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /compute_ik service...')

        req = GetPositionIK.Request()
        req.ik_request.group_name = 'ur_manipulator'
        req.ik_request.avoid_collisions = True
        
        seed = RobotState()
        seed.joint_state.name = self.joint_names
        seed.joint_state.position = seed_positions
        req.ik_request.robot_state = seed

        pose = PoseStamped()
        pose.header.frame_id = 'base_link'
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = float(x), float(y), float(z)
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = float(qx), float(qy), float(qz), float(qw)
        req.ik_request.pose_stamped = pose

        future = self.ik_client.call_async(req)
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        event.wait() 

        res = future.result()
        if res.error_code.val == res.error_code.SUCCESS:
            return list(res.solution.joint_state.position)
        return None

    def run_control_scheme(self):
        self.action_client.wait_for_server()
        while not self.state_received: time.sleep(0.1)

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        goal_msg.trajectory.header.stamp = self.get_clock().now().to_msg()
        
        self.get_logger().info('Processing Cartesian Waypoints...')
        goal_msg.trajectory.points = self.build_ik_trajectory()
        
        if not goal_msg.trajectory.points:
            self.get_logger().error("Trajectory generation failed. Aborting.")
            rclpy.shutdown()
            return
            
        self.is_recording = True
        self.get_logger().info('Executing Cartesian Trajectory and Recording Data...')
        future = self.action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def build_ik_trajectory(self):
        filepath = os.path.join(os.getcwd(), 'cartesian_waypoints.csv')
        points = []
        seed_state = self.current_joint_positions # Start IK seeded at current physical location
        
        if not os.path.exists(filepath):
            self.get_logger().info('cartesian_waypoints.csv missing. Using hardcoded test points...')
            raw_wps = [
                (4.0, 0.4, 0.1, 0.5, 0.0, 0.0, 0.0, 1.0),
                (8.0, 0.4, -0.1, 0.5, 0.0, 0.0, 0.0, 1.0)
            ]
        else:
            raw_wps = []
            with open(filepath, 'r') as f:
                reader = csv.reader(f)
                next(reader)
                for r in reader:
                    if r: raw_wps.append([float(x) for x in r])

        for wp in raw_wps:
            t_sec, x, y, z, qx, qy, qz, qw = wp
            solved_joints = self.solve_ik(x, y, z, qx, qy, qz, qw, seed_state)
            
            if solved_joints:
                pt = JointTrajectoryPoint()
                pt.time_from_start.sec = int(t_sec)
                pt.positions = solved_joints
                pt.velocities = [0.0] * 7
                points.append(pt)
                seed_state = solved_joints # Next waypoint calculated relative to this one
            else:
                self.get_logger().error(f"IK Failed for WP at time {t_sec}s. Stopping planner.")
                return []
        return points

    def sensor_callback(self, msg):
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                self.current_joint_positions[i] = msg.position[msg.name.index(name)]
        self.state_received = True

        if not self.is_recording: return
        current_time = msg.header.stamp.sec + (msg.header.stamp.nanosec * 1e-9)
        row = [current_time]
        for name in self.joint_names:
            if name in msg.name:
                idx = msg.name.index(name)
                row.extend([msg.position[idx], msg.velocity[idx]])
                row.append(msg.effort[idx] if len(msg.effort) > idx else float('nan'))
            else:
                row.extend([0.0, 0.0, 0.0])
        self.telemetry_data.append(row)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Gazebo REJECTED the trajectory.')
            self.is_recording = False
            return
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.execution_finished_callback)

    def execution_finished_callback(self, future):
        self.is_recording = False
        self.get_logger().info('Execution Complete!')
        self.export_data('cartesian_space_log')

    def export_data(self, filename_prefix):
        if len(self.telemetry_data) < 2: return rclpy.shutdown()
        data_matrix = np.array(self.telemetry_data)
        
        csv_filepath = os.path.join(os.getcwd(), f'{filename_prefix}.csv')
        headers = ['timestamp_sec']
        for name in self.joint_names:
            headers.extend([f'{name}_pos', f'{name}_vel', f'{name}_torque'])
            
        with open(csv_filepath, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(headers)
            writer.writerows(self.telemetry_data)

        h5_filepath = os.path.join(os.getcwd(), f'{filename_prefix}.h5')
        with h5py.File(h5_filepath, 'w') as hf:
            hf.create_dataset('timestamp_sec', data=data_matrix[:, 0])
            for i, name in enumerate(self.joint_names):
                hf.create_dataset(f'position/{name}', data=data_matrix[:, (i*3)+1])
                hf.create_dataset(f'velocity/{name}', data=data_matrix[:, (i*3)+2])
                hf.create_dataset(f'effort/{name}', data=data_matrix[:, (i*3)+3])

        self.get_logger().info(f'SUCCESS: Data exported. Shutting down.')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    controller = CartesianIKController()
    try: rclpy.spin(controller)
    except KeyboardInterrupt: pass

if __name__ == '__main__': main()