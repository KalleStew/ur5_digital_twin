#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class ConventionalController(Node):
    def __init__(self):
        super().__init__('conventional_controller')
        
        # 1. ACTION SPACE (Publisher)
        # We talk directly to the trajectory controller, bypassing MoveIt's strict GUI
        self.publisher_ = self.create_publisher(
            JointTrajectory, 
            '/ur_manipulator_controller/joint_trajectory', 
            10
        )
        
        # Give the ROS 2 network 2 seconds to discover this publisher
        self.timer = self.create_timer(2.0, self.execute_trajectory)
        self.trajectory_sent = False
        
        self.get_logger().info('Conventional Control Node Initialized. Preparing to move arm...')

    def execute_trajectory(self):
        # We only want to send the command once
        if self.trajectory_sent:
            return

        msg = JointTrajectory()
        # MUST exactly match the order in your ros2_controllers.yaml
        msg.joint_names = [
            'shoulder_pan_joint1',
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]

        # 2. DEFINE THE WAYPOINT
        point = JointTrajectoryPoint()
        
        # Target Positions: Moving all joints to 0.0 (Horizontal Pose)
        point.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Tell the controller to make this move take exactly 4.0 seconds
        # The controller will automatically generate a smooth velocity curve
        point.time_from_start = Duration(sec=4, nanosec=0)

        msg.points.append(point)
        
        self.get_logger().info('Publishing Trajectory Command: Lowering arm to horizontal...')
        self.publisher_.publish(msg)
        self.trajectory_sent = True

def main(args=None):
    rclpy.init(args=args)
    node = ConventionalController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Conventional Controller...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()