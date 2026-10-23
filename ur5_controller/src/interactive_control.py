import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import PoseStamped
import csv
import os
import threading
import numpy as np
import math
import h5py
from scipy.signal import butter, filtfilt

class InteractiveHILController(Node):
    def __init__(self):
        super().__init__('interactive_hil_controller', 
                         parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        
        # --- Core ROS 2 Interfaces ---
        self.action_client = ActionClient(self, FollowJointTrajectory, '/ur_manipulator_controller/follow_joint_trajectory')
        self.state_sub = self.create_subscription(JointState, '/joint_states', self.sensor_callback, 10)
        
        # IK Service Client for Cartesian Mode
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        
        # --- Data Recording Variables ---
        self.is_recording = False # Do not start recording until trajectory begins
        self.telemetry_data = []
        self.joint_names = [
            'shoulder_pan_joint1', 'shoulder_pan_joint', 'shoulder_lift_joint',
            'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]

        # --- Threading Events ---
        self.trajectory_event = threading.Event()
        self.ik_event = threading.Event()
        self.ik_result_msg = None
        
        # Launch the interactive CLI in a background thread to prevent ROS executor deadlocks
        self.control_thread = threading.Thread(target=self.run_interactive_cli)
        self.control_thread.start()

    def run_interactive_cli(self):
        """ The main interactive loop run in the background thread. """
        print("\n" + "="*50)
        print("   UR5 DIGITAL TWIN - INTERACTIVE HIL CONTROL")
        print("="*50)

        # 1. Select Control Mode
        print("\n[1] Joint Space Waypoints")
        print("[2] Cartesian Space Waypoints (Requires IK)")
        mode = input("Select Mode (1 or 2): ").strip()
        is_cartesian = (mode == '2')

        # 2. Select Input File
        default_file = 'default_cartesian.csv' if is_cartesian else 'default_joint.csv'
        filename = input(f"\nEnter waypoint file name in 'waypoints/' directory \n[Press Enter for {default_file}]: ").strip()
        if not filename:
            filename = default_file

        filepath = os.path.join(os.getcwd(), 'waypoints', filename)
        if not os.path.exists(filepath):
            self.get_logger().error(f"File not found: {filepath}")
            return rclpy.shutdown()

        # 3. Load & Process Waypoints
        self.get_logger().info(f"Loading waypoints from: {filename}")
        if is_cartesian:
            points = self.load_cartesian_waypoints(filepath)
        else:
            points = self.load_joint_waypoints(filepath)

        if not points:
            self.get_logger().error("No valid waypoints loaded. Aborting.")
            return rclpy.shutdown()

        # 4. Connect to Action Server
        self.get_logger().info('Connecting to Gazebo Trajectory Controller...')
        self.action_client.wait_for_server()
        
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        goal_msg.trajectory.header.stamp = self.get_clock().now().to_msg()
        goal_msg.trajectory.points = points
        
        # Inject relaxed tolerances to prevent Action Server aborts from micro-sag
        for name in self.joint_names:
            tol = JointTolerance(name=name, position=1.0, velocity=1.0)
            goal_msg.goal_tolerance.append(tol)
        
        # 5. Execute & Record
        input("\n[READY] Press Enter to execute trajectory and begin telemetry recording...")
        self.trajectory_event.clear()
        self.is_recording = True  # START VACUUMING SENSOR DATA
        
        future = self.action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

        # Hard timeout calculation (longest waypoint time + 10s buffer)
        max_time = max([pt.time_from_start.sec for pt in points]) + 10.0
        self.get_logger().info(f'Execution started. Timeout set to {max_time}s.')
        
        finished = self.trajectory_event.wait(timeout=max_time)
        self.is_recording = False # STOP RECORDING

        if not finished:
            self.get_logger().warn('Trajectory Action Server hung (likely due to gravity sag). Forcing completion.')
        
        # 6. Save Data
        print("\n" + "="*50)
        out_name = input("Execution Complete. Enter export filename (without extension) \n[Press Enter for 'run_log']: ").strip()
        if not out_name:
            out_name = "run_log"
            
        self.export_data(out_name)

    # PARSERS & SOLVERS

    def load_joint_waypoints(self, filepath):
        """ Reads direct 7-DOF joint angles from a CSV """
        points = []
        with open(filepath, 'r') as f:
            reader = csv.reader(f)
            next(reader) # Skip Header
            for row in reader:
                if not row: continue
                pt = JointTrajectoryPoint()
                pt.time_from_start.sec = int(float(row[0]))
                pt.positions = [float(x) for x in row[1:8]]
                pt.velocities = [0.0] * 7 # Stop at each waypoint
                points.append(pt)
        return points

    def load_cartesian_waypoints(self, filepath):
        """ Reads XYZ RPY from CSV and calls MoveIt IK service to find Joint Angles """
        self.ik_client.wait_for_service()
        points = []
        
        with open(filepath, 'r') as f:
            reader = csv.reader(f)
            next(reader) # Skip Header
            for row in reader:
                if not row: continue
                
                time_sec = int(float(row[0]))
                x, y, z = float(row[1]), float(row[2]), float(row[3])
                roll, pitch, yaw = float(row[4]), float(row[5]), float(row[6])
                
                # Math: Convert Euler to Quaternion
                qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
                qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
                qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
                qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

                # Query the IK Service
                joint_positions = self.solve_ik(x, y, z, qx, qy, qz, qw)
                if joint_positions:
                    pt = JointTrajectoryPoint()
                    pt.time_from_start.sec = time_sec
                    pt.positions = joint_positions
                    pt.velocities = [0.0] * 7
                    points.append(pt)
                else:
                    self.get_logger().error(f"IK Failed for Waypoint at {time_sec}s. Skipping.")
        return points

    def solve_ik(self, x, y, z, qx, qy, qz, qw):
        """ Thread-safe helper to call the ROS 2 compute_ik service """
        req = GetPositionIK.Request()
        req.ik_request.group_name = "ur_manipulator"
        req.ik_request.pose_stamped.header.frame_id = "world"
        req.ik_request.pose_stamped.pose.position.x = x
        req.ik_request.pose_stamped.pose.position.y = y
        req.ik_request.pose_stamped.pose.position.z = z
        req.ik_request.pose_stamped.pose.orientation.x = qx
        req.ik_request.pose_stamped.pose.orientation.y = qy
        req.ik_request.pose_stamped.pose.orientation.z = qz
        req.ik_request.pose_stamped.pose.orientation.w = qw
        req.ik_request.timeout.sec = 1

        self.ik_event.clear()
        
        def ik_callback(future):
            self.ik_result_msg = future.result()
            self.ik_event.set()
            
        future = self.ik_client.call_async(req)
        future.add_done_callback(ik_callback)
        self.ik_event.wait() # Pause thread until IK response returns

        if self.ik_result_msg.error_code.val == 1: # SUCCESS
            # Return exactly 7 joints to match our action client expectations
            return list(self.ik_result_msg.solution.joint_state.position)[:7]
        return None

    # CORE CALLBACKS & EXPORT

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
        self.get_logger().info('Action Server reported execution complete.')
        self.trajectory_event.set()

    def export_data(self, filename_prefix):
        if len(self.telemetry_data) < 10: 
            self.get_logger().error("Not enough data to export.")
            return rclpy.shutdown()
            
        data_matrix = np.array(self.telemetry_data)
        
        # Butterworth Filter (Zero-Phase) to clean physics chatter
        b, a = butter(N=3, Wn=4.0/50.0, btype='low')
        filtered_data = np.copy(data_matrix)
        
        for i in range(len(self.joint_names)):
            vel_col = (i * 3) + 2
            eff_col = (i * 3) + 3
            
            filtered_data[:, vel_col] = filtfilt(b, a, data_matrix[:, vel_col])
            
            eff_raw = data_matrix[:, eff_col]
            if not np.isnan(eff_raw).all():
                eff_raw = np.nan_to_num(eff_raw) 
                filtered_data[:, eff_col] = filtfilt(b, a, eff_raw)
        
        # Save to the specific 'data' directory
        data_dir = os.path.join(os.getcwd(), 'data')
        csv_filepath = os.path.join(data_dir, f'{filename_prefix}_filtered.csv')
        h5_filepath = os.path.join(data_dir, f'{filename_prefix}_filtered.h5')

        # Export CSV
        headers = ['timestamp_sec']
        for name in self.joint_names:
            headers.extend([f'{name}_pos', f'{name}_vel', f'{name}_torque'])
            
        with open(csv_filepath, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(headers)
            writer.writerows(filtered_data)

        # Export HDF5
        with h5py.File(h5_filepath, 'w') as hf:
            hf.create_dataset('timestamp_sec', data=filtered_data[:, 0])
            for i, name in enumerate(self.joint_names):
                hf.create_dataset(f'position/{name}', data=filtered_data[:, (i*3)+1])
                hf.create_dataset(f'velocity/{name}', data=filtered_data[:, (i*3)+2])
                hf.create_dataset(f'effort/{name}', data=filtered_data[:, (i*3)+3])

        print(f"\n[SUCCESS] Files saved to:\n- {csv_filepath}\n- {h5_filepath}")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    controller = InteractiveHILController()
    try: 
        rclpy.spin(controller)
    except KeyboardInterrupt: 
        pass

if __name__ == '__main__': main()