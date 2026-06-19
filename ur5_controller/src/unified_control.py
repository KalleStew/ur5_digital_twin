#!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from rclpy.action import ActionClient
# from rclpy.parameter import Parameter
# from control_msgs.action import FollowJointTrajectory
# from trajectory_msgs.msg import JointTrajectoryPoint
# from sensor_msgs.msg import JointState
# import csv
# import os
# import threading

# class UnifiedConventionalController(Node):
#     def __init__(self):
#         # 1. FORCE GAZEBO SIMULATION TIME AT BOOT
#         super().__init__(
#             'unified_controller', 
#             parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)]
#         )
        
#         # 2. ACTION CLIENT (The Commander)
#         self.action_client = ActionClient(self, FollowJointTrajectory, '/ur_manipulator_controller/follow_joint_trajectory')
        
#         # 3. SENSOR SUBSCRIBER (The Recorder)
#         self.state_sub = self.create_subscription(JointState, '/joint_states', self.sensor_callback, 10)
        
#         # Telemetry Storage
#         self.is_recording = False
#         self.telemetry_data = []
#         self.joint_names = [
#             'shoulder_pan_joint1', 'shoulder_pan_joint', 'shoulder_lift_joint',
#             'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
#         ]

#         # 4. PREVENT DEADLOCKS WITH A BACKGROUND THREAD
#         # This allows ROS 2 to process network traffic while our custom logic runs
#         self.control_thread = threading.Thread(target=self.run_experiment)
#         self.control_thread.start()

#     def run_experiment(self):
#         """
#         --- MAIN CONTROL PIPELINE ---
#         This function runs independently. It waits for the physics engine, 
#         generates the trajectory, and triggers the data logging.
#         """
#         self.get_logger().info('Searching for Gazebo Trajectory Controller...')
        
#         # Because we are in a background thread, this will NO LONGER freeze the script!
#         self.action_client.wait_for_server()
#         self.get_logger().info('Controller Connected! Generating Waypoints...')
        
#         goal_msg = FollowJointTrajectory.Goal()
#         goal_msg.trajectory.joint_names = self.joint_names
        
#         # Sync the trajectory to the physics engine's clock
#         goal_msg.trajectory.header.stamp = self.get_clock().now().to_msg()
        
#         # Generate and attach the waypoints
#         goal_msg.trajectory.points = self.generate_trajectory()
        
#         self.get_logger().info('Executing Trajectory and Recording Telemetry...')
#         self.is_recording = True # Turn on the sensor data vacuum
        
#         # Send the command to Gazebo
#         future = self.action_client.send_goal_async(goal_msg)
#         future.add_done_callback(self.goal_response_callback)

#     def generate_trajectory(self):
#         """
#         --- RESEARCH MODIFICATION ZONE ---
#         Drop your custom control schemes, CSV parsers, or IK solvers here.
#         Just ensure this function returns a list of 'JointTrajectoryPoint' objects.
#         """
#         points = []
        
#         # Waypoint 1: Move to horizontal (0.0 rads) over 4 seconds
#         pt1 = JointTrajectoryPoint()
#         pt1.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#         pt1.velocities = [0.0] * 7 # Apply brakes at destination
#         pt1.time_from_start.sec = 4
#         points.append(pt1)
        
#         # Waypoint 2: Return to Candle Pose (-1.5708 rads) over the next 4 seconds
#         pt2 = JointTrajectoryPoint()
#         pt2.positions = [0.0, 0.0, -1.5708, 0.0, 0.0, 0.0, 0.0]
#         pt2.velocities = [0.0] * 7 # Apply brakes at destination
#         pt2.time_from_start.sec = 8
#         points.append(pt2)
        
#         return points

#     def sensor_callback(self, msg):
#         """ Runs continuously at high frequency, capturing Gazebo's physics state """
#         if not self.is_recording:
#             return

#         current_time = msg.header.stamp.sec + (msg.header.stamp.nanosec * 1e-9)
        
#         row = [current_time]
#         for name in self.joint_names:
#             if name in msg.name:
#                 idx = msg.name.index(name)
#                 row.append(msg.position[idx])
#                 row.append(msg.velocity[idx])
#                 # msg.effort captures the raw Torque your PID gains are currently fighting with
#                 row.append(msg.effort[idx]) 
#             else:
#                 row.extend([0.0, 0.0, 0.0])
                
#         self.telemetry_data.append(row)

#     def goal_response_callback(self, future):
#         goal_handle = future.result()
#         if not goal_handle.accepted:
#             self.get_logger().error('Gazebo REJECTED the trajectory. Check joint limits.')
#             self.is_recording = False
#             return

#         self._get_result_future = goal_handle.get_result_async()
#         self._get_result_future.add_done_callback(self.execution_finished_callback)

#     def execution_finished_callback(self, future):
#         """ Triggered the exact millisecond the arm finishes the final waypoint """
#         self.is_recording = False
#         self.get_logger().info('Trajectory Execution Complete!')
#         self.export_telemetry_to_csv()

#     def export_telemetry_to_csv(self):
#         filename = 'robot_performance_log.csv'
#         filepath = os.path.join(os.getcwd(), filename)
        
#         headers = ['timestamp_sec']
#         for name in self.joint_names:
#             headers.extend([f'{name}_pos', f'{name}_vel', f'{name}_torque'])
            
#         with open(filepath, mode='w', newline='') as file:
#             writer = csv.writer(file)
#             writer.writerow(headers)
#             writer.writerows(self.telemetry_data)
            
#         self.get_logger().info(f'SUCCESS: Exported {len(self.telemetry_data)} frames of data to {filepath}')
#         rclpy.shutdown() # Safely shut down the node

# def main(args=None):
#     rclpy.init(args=args)
#     controller = UnifiedConventionalController()
    
#     try:
#         # rclpy.spin processes the ROS 2 network traffic. 
#         # Our custom code runs safely above it in the background thread!
#         rclpy.spin(controller)
#     except KeyboardInterrupt:
#         pass

# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
import csv
import os
import threading
import numpy as np
import h5py

class ConventionalResearchController(Node):
    def __init__(self):
        super().__init__('conventional_research_controller', 
                         parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        
        self.action_client = ActionClient(self, FollowJointTrajectory, '/ur_manipulator_controller/follow_joint_trajectory')
        self.state_sub = self.create_subscription(JointState, '/joint_states', self.sensor_callback, 10)
        
        self.is_recording = True 
        self.telemetry_data = []
        self.joint_names = [
            'shoulder_pan_joint1', 'shoulder_pan_joint', 'shoulder_lift_joint',
            'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]

        # Background thread prevents ROS 2 networking deadlocks
        self.control_thread = threading.Thread(target=self.run_control_scheme)
        self.control_thread.start()

    def run_control_scheme(self):
        """ PHASE 1: INITIALIZATION & INGESTION """
        self.get_logger().info('Searching for Gazebo Trajectory Controller...')
        self.action_client.wait_for_server()
        
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        goal_msg.trajectory.header.stamp = self.get_clock().now().to_msg()
        
        # Load waypoints (from CSV, or later from MoveIt IK)
        goal_msg.trajectory.points = self.load_waypoints()
        
        """ PHASE 2: EXECUTION """
        self.get_logger().info('Executing Trajectory and Recording Data...')
        future = self.action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def load_waypoints(self):
        """ Modifiable Zone: Replace this logic with IK solvers or custom parsers as needed """
        filepath = os.path.join(os.getcwd(), 'waypoints.csv')
        
        # Auto-generate a safe panning wave if no file exists
        if not os.path.exists(filepath):
            self.get_logger().info('waypoints.csv missing. Auto-generating test trajectory...')
            with open(filepath, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['time_sec'] + self.joint_names)
                writer.writerow([4.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) # Pan Left
                writer.writerow([8.0, -0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) # Pan Right
                writer.writerow([12.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) # Center

        points = []
        with open(filepath, 'r') as f:
            reader = csv.reader(f)
            next(reader)  # Skip header
            for row in reader:
                if not row: continue
                pt = JointTrajectoryPoint()
                pt.time_from_start.sec = int(float(row[0]))
                pt.positions = [float(x) for x in row[1:8]]
                pt.velocities = [0.0] * 7 # Stop gracefully at each waypoint
                points.append(pt)
                
        return points

    def sensor_callback(self, msg):
        """ PHASE 3: SENSOR FEEDBACK (Runs at 100Hz) """
        if not self.is_recording:
            return

        current_time = msg.header.stamp.sec + (msg.header.stamp.nanosec * 1e-9)
        row = [current_time]
        
        for name in self.joint_names:
            if name in msg.name:
                idx = msg.name.index(name)
                row.append(msg.position[idx])
                row.append(msg.velocity[idx])
                # Safely capture effort (torque) data
                if len(msg.effort) > idx:
                    row.append(msg.effort[idx]) 
                else:
                    row.append(float('nan'))
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
        result = future.result().result
        self.get_logger().info(f'Execution Complete! Goal Status: {result.error_code}')
        self.export_data()

    def export_data(self):
        """ PHASE 4: ANALYSIS EXPORT """
        if len(self.telemetry_data) < 2:
            self.get_logger().warning("Not enough data collected!")
            rclpy.shutdown()
            return

        data_matrix = np.array(self.telemetry_data)
        
        # Export CSV
        csv_filepath = os.path.join(os.getcwd(), 'robot_performance_log.csv')
        headers = ['timestamp_sec']
        for name in self.joint_names:
            headers.extend([f'{name}_pos', f'{name}_vel', f'{name}_torque'])
            
        with open(csv_filepath, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(headers)
            writer.writerows(self.telemetry_data)

        # Export HDF5 
        h5_filepath = os.path.join(os.getcwd(), 'robot_performance_log.h5')
        with h5py.File(h5_filepath, 'w') as hf:
            hf.create_dataset('timestamp_sec', data=data_matrix[:, 0])
            pos_group = hf.create_group('position')
            vel_group = hf.create_group('velocity')
            eff_group = hf.create_group('effort')

            col_idx = 1
            for name in self.joint_names:
                pos_group.create_dataset(name, data=data_matrix[:, col_idx])
                vel_group.create_dataset(name, data=data_matrix[:, col_idx+1])
                eff_group.create_dataset(name, data=data_matrix[:, col_idx+2])
                col_idx += 3

        self.get_logger().info(f'SUCCESS: HDF5 and CSV exported to working directory.')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    controller = ConventionalResearchController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()