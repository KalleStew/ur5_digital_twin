#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import csv
import time

class ResearchController(Node):
    def __init__(self):
        super().__init__('research_controller', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.parameter.Parameter.Type.BOOL, True)])
        
        # Publisher to drive the robot
        self.pub = self.create_publisher(JointTrajectory, '/ur_manipulator_controller/joint_trajectory', 10)
        
        # Subscriber to record performance (Data Logging)
        self.sub = self.create_subscription(JointState, '/joint_states', self.log_performance, 10)
        
        # Define Path Waypoints (Hardcoded for now - replace with CSV loader logic as needed)
        # Format: [shoulder_pan, shoulder_lift, elbow, wrist1, wrist2, wrist3]
        self.waypoints = [
            [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
            [0.5, -1.0, 0.5, -1.0, 0.5, 0.0],
            [1.0, -0.5, 1.0, -0.5, 1.0, 0.0]
        ]
        
        self.log_file = open('performance_log.csv', 'w', newline='')
        self.writer = csv.writer(self.log_file)
        self.writer.writerow(['timestamp', 'joint_pos_1', 'joint_pos_2', 'joint_pos_3'])
        
        self.timer = self.create_timer(2.0, self.send_trajectory)
        self.get_logger().info("Research Controller Ready. Broadcasting path...")

    def log_performance(self, msg):
        # Extract and log joint data for analysis
        data = [self.get_clock().now().nanoseconds] + list(msg.position[:3])
        self.writer.writerow(data)

    def send_trajectory(self):
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        
        for i, pos in enumerate(self.waypoints):
            pt = JointTrajectoryPoint()
            pt.positions = pos
            pt.time_from_start = rclpy.duration.Duration(seconds=float(i+2)).to_msg()
            traj.points.append(pt)
            
        self.pub.publish(traj)
        self.get_logger().info("Trajectory path published to Gazebo.")

def main():
    rclpy.init()
    node = ResearchController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.log_file.close()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()