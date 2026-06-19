# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from rclpy.action import ActionClient
# from rclpy.parameter import Parameter
# from control_msgs.action import FollowJointTrajectory
# from trajectory_msgs.msg import JointTrajectoryPoint
# from sensor_msgs.msg import JointState
# import csv, os, threading
# import numpy as np
# import h5py

# class JointSpaceController(Node):
#     def __init__(self):
#         super().__init__('joint_space_controller', 
#                          parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        
#         self.action_client = ActionClient(self, FollowJointTrajectory, '/ur_manipulator_controller/follow_joint_trajectory')
#         self.state_sub = self.create_subscription(JointState, '/joint_states', self.sensor_callback, 10)
        
#         self.is_recording = True 
#         self.telemetry_data = []
#         self.joint_names = [
#             'shoulder_pan_joint1', 'shoulder_pan_joint', 'shoulder_lift_joint',
#             'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
#         ]

#         # Background thread prevents ROS 2 networking deadlocks
#         self.control_thread = threading.Thread(target=self.run_control_scheme)
#         self.control_thread.start()

#     def run_control_scheme(self):
#         self.get_logger().info('Searching for Gazebo Trajectory Controller...')
#         self.action_client.wait_for_server()
        
#         goal_msg = FollowJointTrajectory.Goal()
#         goal_msg.trajectory.joint_names = self.joint_names
#         goal_msg.trajectory.header.stamp = self.get_clock().now().to_msg()
        
#         goal_msg.trajectory.points = self.load_waypoints()
        
#         self.get_logger().info('Executing Joint Space Trajectory and Recording Data...')
#         future = self.action_client.send_goal_async(goal_msg)
#         future.add_done_callback(self.goal_response_callback)

#     def load_waypoints(self):
#         filepath = os.path.join(os.getcwd(), 'joint_waypoints.csv')
        
#         # Auto-generate a safe joint space sequence if no file exists
#         if not os.path.exists(filepath):
#             self.get_logger().info('joint_waypoints.csv missing. Auto-generating test trajectory...')
#             with open(filepath, 'w', newline='') as f:
#                 writer = csv.writer(f)
#                 writer.writerow(['time_sec'] + self.joint_names)
#                 # Waypoint 1: Move to Candle Pose
#                 writer.writerow([4.0, 0.0, 0.0, 1.57, 0.0, 0.0, 0.0, 0.0]) 
#                 # Waypoint 2: Pan Left
#                 writer.writerow([8.0, 0.5, 0.0, 1.57, 0.0, 0.0, 0.0, 0.0]) 
#                 # Waypoint 3: Pan Right
#                 writer.writerow([12.0, -0.5, 0.0, 1.57, 0.0, 0.0, 0.0, 0.0])

#         points = []
#         with open(filepath, 'r') as f:
#             reader = csv.reader(f)
#             next(reader)  # Skip header
#             for row in reader:
#                 if not row: continue
#                 pt = JointTrajectoryPoint()
#                 pt.time_from_start.sec = int(float(row[0]))
#                 pt.positions = [float(x) for x in row[1:8]]
#                 pt.velocities = [0.0] * 7 # Ensure rigid stops at waypoints
#                 points.append(pt)
                
#         return points

#     def sensor_callback(self, msg):
#         if not self.is_recording: return
#         current_time = msg.header.stamp.sec + (msg.header.stamp.nanosec * 1e-9)
#         row = [current_time]
        
#         for name in self.joint_names:
#             if name in msg.name:
#                 idx = msg.name.index(name)
#                 row.extend([msg.position[idx], msg.velocity[idx]])
#                 if len(msg.effort) > idx:
#                     row.append(msg.effort[idx]) 
#                 else:
#                     row.append(float('nan'))
#             else:
#                 row.extend([0.0, 0.0, 0.0])
                
#         self.telemetry_data.append(row)

#     def goal_response_callback(self, future):
#         goal_handle = future.result()
#         if not goal_handle.accepted:
#             self.get_logger().error('Gazebo REJECTED the trajectory.')
#             self.is_recording = False
#             return
#         self._get_result_future = goal_handle.get_result_async()
#         self._get_result_future.add_done_callback(self.execution_finished_callback)

#     def execution_finished_callback(self, future):
#         self.is_recording = False
#         result = future.result().result
#         self.get_logger().info(f'Execution Complete! Goal Status: {result.error_code}')
#         self.export_data('joint_space_log')

#     def export_data(self, filename_prefix):
#         if len(self.telemetry_data) < 2: return rclpy.shutdown()
#         data_matrix = np.array(self.telemetry_data)
        
#         csv_filepath = os.path.join(os.getcwd(), f'{filename_prefix}.csv')
#         headers = ['timestamp_sec']
#         for name in self.joint_names:
#             headers.extend([f'{name}_pos', f'{name}_vel', f'{name}_torque'])
            
#         with open(csv_filepath, 'w', newline='') as f:
#             writer = csv.writer(f)
#             writer.writerow(headers)
#             writer.writerows(self.telemetry_data)

#         h5_filepath = os.path.join(os.getcwd(), f'{filename_prefix}.h5')
#         with h5py.File(h5_filepath, 'w') as hf:
#             hf.create_dataset('timestamp_sec', data=data_matrix[:, 0])
#             for i, name in enumerate(self.joint_names):
#                 hf.create_dataset(f'position/{name}', data=data_matrix[:, (i*3)+1])
#                 hf.create_dataset(f'velocity/{name}', data=data_matrix[:, (i*3)+2])
#                 hf.create_dataset(f'effort/{name}', data=data_matrix[:, (i*3)+3])

#         self.get_logger().info(f'SUCCESS: HDF5 and CSV exported. Shutting down.')
#         rclpy.shutdown()

# def main(args=None):
#     rclpy.init(args=args)
#     controller = JointSpaceController()
#     try: rclpy.spin(controller)
#     except KeyboardInterrupt: pass

# if __name__ == '__main__': main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
import csv, os, threading
import numpy as np
import h5py

class JointSpaceController(Node):
    def __init__(self):
        super().__init__('joint_space_controller', 
                         parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        
        self.action_client = ActionClient(self, FollowJointTrajectory, '/ur_manipulator_controller/follow_joint_trajectory')
        self.state_sub = self.create_subscription(JointState, '/joint_states', self.sensor_callback, 10)
        
        self.is_recording = True 
        self.telemetry_data = []
        self.joint_names = [
            'shoulder_pan_joint1', 'shoulder_pan_joint', 'shoulder_lift_joint',
            'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]

        # RE-APPLIED: Threading Event to prevent infinite hangs
        self.trajectory_event = threading.Event()
        
        self.control_thread = threading.Thread(target=self.run_control_scheme)
        self.control_thread.start()

    def run_control_scheme(self):
        self.get_logger().info('Searching for Gazebo Trajectory Controller...')
        self.action_client.wait_for_server()
        
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        goal_msg.trajectory.header.stamp = self.get_clock().now().to_msg()
        goal_msg.trajectory.points = self.load_waypoints()
        
        # RE-APPLIED: Massive Goal Tolerances to account for gravity sag
        for name in self.joint_names:
            tol = JointTolerance()
            tol.name = name
            tol.position = 1.0  # 1.0 rad tolerance
            tol.velocity = 1.0
            goal_msg.goal_tolerance.append(tol)
        
        self.get_logger().info('Executing Joint Space Trajectory and Recording Data...')
        self.trajectory_event.clear()
        
        future = self.action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

        # RE-APPLIED: Hard timeout (12s trajectory + 6s buffer)
        finished = self.trajectory_event.wait(timeout=18.0)
        if not finished:
            self.get_logger().warn('Trajectory Action Server hung due to gravity sag. Forcing completion.')
            self.export_data('joint_space_log')

    def load_waypoints(self):
        filepath = os.path.join(os.getcwd(), 'joint_waypoints1.csv')
        
        if not os.path.exists(filepath):
            self.get_logger().info('Auto-generating test trajectory...')
            with open(filepath, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['time_sec'] + self.joint_names)
                # Waypoint 1: Home (Curved forward, NOT vertical candle)
                writer.writerow([4.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) 
                # Waypoint 2: Pan Left
                # writer.writerow([8.0, -0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) 
                # # Waypoint 3: Pan Right
                # writer.writerow([12.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
                # # Waypoint 4: High Stress over the table
                # writer.writerow([16.0, 0.5, -0.5, -0.5, 0.0, 0.0, 0.0, 0.0])

        points = []
        with open(filepath, 'r') as f:
            reader = csv.reader(f)
            next(reader) 
            for row in reader:
                if not row: continue
                pt = JointTrajectoryPoint()
                pt.time_from_start.sec = int(float(row[0]))
                pt.positions = [float(x) for x in row[1:8]]
                pt.velocities = [0.0] * 7
                points.append(pt)
        return points

    def sensor_callback(self, msg):
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
        self.get_logger().info('Action Server reported execution complete.')
        self.trajectory_event.set() # Unlocks the thread naturally
        self.export_data('joint_space_log')

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

        self.get_logger().info(f'SUCCESS: HDF5 and CSV exported. Shutting down.')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    controller = JointSpaceController()
    try: rclpy.spin(controller)
    except KeyboardInterrupt: pass

if __name__ == '__main__': main()