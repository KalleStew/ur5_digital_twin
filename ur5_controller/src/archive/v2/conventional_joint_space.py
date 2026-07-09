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
from scipy.signal import butter, filtfilt

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
                writer.writerow([8.0, -0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) 
                # Waypoint 3: Pan Right
                writer.writerow([12.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
                # Waypoint 4: High Stress over the table
                writer.writerow([16.0, 1.57, 0.0, -1.57, 0.0, 0.0, 0.0, 0.0])

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
        if len(self.telemetry_data) < 10: 
            self.get_logger().error("Not enough data to filter/export.")
            return rclpy.shutdown()
            
        data_matrix = np.array(self.telemetry_data)
        
        # --- BUTTERWORTH LOW-PASS FILTER DESIGN ---
        # Sample rate = 100Hz (0.01s). Nyquist = 50Hz. Cutoff = 4Hz.
        b, a = butter(N=3, Wn=4.0/50.0, btype='low')
        
        # We will keep raw position, but filter Velocity and Effort (Torque)
        filtered_data = np.copy(data_matrix)
        for i in range(len(self.joint_names)):
            vel_col = (i * 3) + 2
            eff_col = (i * 3) + 3
            
            # Apply zero-phase filter to smooth the violent chatter
            filtered_data[:, vel_col] = filtfilt(b, a, data_matrix[:, vel_col])
            
            # Handle NaN values in effort before filtering
            eff_raw = data_matrix[:, eff_col]
            if not np.isnan(eff_raw).all():
                eff_raw = np.nan_to_num(eff_raw) # Convert NaNs to 0 temporarily
                filtered_data[:, eff_col] = filtfilt(b, a, eff_raw)
        
        # --- EXPORT ---
        csv_filepath = os.path.join(os.getcwd(), f'{filename_prefix}_filtered.csv')
        headers = ['timestamp_sec']
        for name in self.joint_names:
            headers.extend([f'{name}_pos', f'{name}_vel', f'{name}_torque'])
            
        with open(csv_filepath, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(headers)
            writer.writerows(filtered_data)

        h5_filepath = os.path.join(os.getcwd(), f'{filename_prefix}_filtered.h5')
        with h5py.File(h5_filepath, 'w') as hf:
            hf.create_dataset('timestamp_sec', data=filtered_data[:, 0])
            for i, name in enumerate(self.joint_names):
                hf.create_dataset(f'position/{name}', data=filtered_data[:, (i*3)+1])
                hf.create_dataset(f'velocity/{name}', data=filtered_data[:, (i*3)+2])
                hf.create_dataset(f'effort/{name}', data=filtered_data[:, (i*3)+3])

        self.get_logger().info(f'SUCCESS: Filtered HDF5 and CSV exported. Shutting down.')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    controller = JointSpaceController()
    try: rclpy.spin(controller)
    except KeyboardInterrupt: pass

if __name__ == '__main__': main()