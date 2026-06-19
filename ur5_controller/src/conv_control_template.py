#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from controller_manager_msgs.srv import SwitchController
from sensor_msgs.msg import JointState

# New IK Imports
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import PoseStamped

import os, threading, time
import numpy as np
import h5py

class ConventionalResearch(Node):
    def __init__(self):
        super().__init__('conventional_research', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        
        # 1. Action & Service Clients
        self.action_client = ActionClient(self, FollowJointTrajectory, '/ur_manipulator_controller/follow_joint_trajectory')
        self.switch_client = self.create_client(SwitchController, '/controller_manager/switch_controller')
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        
        # 2. Telemetry Subscription
        self.state_sub = self.create_subscription(JointState, '/joint_states', self.sensor_callback, 10)
        
        # Exact 7-DOF joints from your YAML
        self.joint_names = [
            'shoulder_pan_joint1', 'shoulder_pan_joint', 'shoulder_lift_joint', 
            'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        self.telemetry_data = []
        self.is_recording = False
        
        # 3. Threading Control
        self.trajectory_finished_event = threading.Event()
        self.control_thread = threading.Thread(target=self.research_pipeline)
        self.control_thread.start()

    def solve_cartesian_ik(self, x, y, z, qx, qy, qz, qw):
        """
        Queries MoveIt 2 IK service thread-safely.
        Returns a list of joint positions, or None if no valid/collision-free solution exists.
        """
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for MoveIt 2 /compute_ik service...')

        request = GetPositionIK.Request()
        request.ik_request.group_name = 'ur_manipulator' # Update if your MoveIt group is named differently
        request.ik_request.avoid_collisions = True
        request.ik_request.timeout.sec = 2

        pose = PoseStamped()
        pose.header.frame_id = 'base_link'
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        pose.pose.orientation.x = float(qx)
        pose.pose.orientation.y = float(qy)
        pose.pose.orientation.z = float(qz)
        pose.pose.orientation.w = float(qw)

        request.ik_request.pose_stamped = pose

        # Async call with Threading Event to prevent Deadlock
        future = self.ik_client.call_async(request)
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        
        # Block ONLY the control_thread until IK computes
        event.wait(timeout=5.0) 

        if future.done():
            response = future.result()
            if response.error_code.val == response.error_code.SUCCESS:
                return list(response.solution.joint_state.position)
            else:
                self.get_logger().error(f"IK Failed (Collision/Singularity). Code: {response.error_code.val}")
                return None
        return None

    def execute_trajectory(self, target_joints, duration_sec=3.0):
        """ Dispatches Joint Targets to the Action Server """
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = target_joints
        point.time_from_start.sec = int(duration_sec)
        point.time_from_start.nanosec = int((duration_sec % 1) * 1e9)
        
        goal_msg.trajectory.points.append(point)
        
        self.trajectory_finished_event.clear()
        
        future = self.action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)
        
        # Block thread until Action Server physically finishes the movement
        self.trajectory_finished_event.wait(timeout=duration_sec + 2.0)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Trajectory rejected by server.')
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda _: self.trajectory_finished_event.set())

    def sensor_callback(self, msg):
        """ Telemetry Recorder """
        if not self.is_recording: return
        t = msg.header.stamp.sec + (msg.header.stamp.nanosec * 1e-9)
        row = [t]
        for name in self.joint_names:
            if name in msg.name:
                idx = msg.name.index(name)
                row.extend([msg.position[idx], msg.velocity[idx]])
                row.append(msg.effort[idx] if len(msg.effort) > idx else float('nan'))
            else:
                row.extend([0.0, 0.0, 0.0])
        self.telemetry_data.append(row)

    def research_pipeline(self):
        """ Main Execution Thread """
        # 1. Wait for Action Server
        self.action_client.wait_for_server()
        self.get_logger().info("Action server connected.")

        # 2. Pre-Flight Candle Pose (Example Joint Space Override)
        self.get_logger().info("Executing Pre-Flight Reset...")
        candle_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
        self.execute_trajectory(candle_pose, duration_sec=4.0)
        
        # 3. Cartesian Execution Phase
        self.get_logger().info("Calculating Cartesian IK for target waypoint...")
        target_joints = self.solve_cartesian_ik(x=0.4, y=0.1, z=0.5, qx=0.0, qy=1.0, qz=0.0, qw=0.0)
        
        if target_joints:
            self.get_logger().info("IK Found! Executing Trajectory & Recording Telemetry...")
            self.is_recording = True
            self.execute_trajectory(target_joints, duration_sec=5.0)
            self.is_recording = False
            
            self.export_hdf5()
            self.get_logger().info("Experiment Complete. System safe.")
        else:
            self.get_logger().error("Aborting experiment due to IK failure.")

    def export_hdf5(self):
        filepath = os.path.join(os.getcwd(), 'conventional_log.h5')
        data = np.array(self.telemetry_data)
        if len(data) == 0:
            self.get_logger().warn("No telemetry recorded.")
            return
            
        with h5py.File(filepath, 'w') as hf:
            hf.create_dataset('timestamp_sec', data=data[:, 0])
            for i, name in enumerate(self.joint_names):
                hf.create_dataset(f'position/{name}', data=data[:, (i*3)+1])
                hf.create_dataset(f'velocity/{name}', data=data[:, (i*3)+2])
                hf.create_dataset(f'effort/{name}', data=data[:, (i*3)+3])
        self.get_logger().info(f"Telemetry exported to {filepath}")

def main(args=None):
    rclpy.init(args=args)
    node = ConventionalResearch()
    rclpy.spin(node) # Main thread strictly processes callbacks
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()