#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import math
import time

class CustomTorqueController(Node):
    def __init__(self):
        super().__init__('custom_torque_controller')
        
        # 1. ACTION SPACE (Publisher)
        # This talks directly to Gazebo's effort controller
        self.torque_pub = self.create_publisher(
            Float64MultiArray, 
            '/forward_effort_controller/commands', 
            10
        )
        
        # 2. OBSERVATION SPACE (Subscriber)
        # This reads the live encoder data from the robot
        self.state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Control Loop Timer (100 Hz = 0.01 seconds)
        self.timer = self.create_timer(0.01, self.control_loop)
        
        # Robot State Variables
        self.current_positions = [0.0] * 7
        self.current_velocities = [0.0] * 7
        self.start_time = time.time()
        
        self.get_logger().info('AI Control Node Initialized. Waiting for Switcher...')

    def joint_state_callback(self, msg):
        # Gazebo alphabetizes joint names. We must map them back to the correct physical order.
        expected_order = [
            "shoulder_pan_joint1", "shoulder_pan_joint", "shoulder_lift_joint", 
            "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        ]
        
        for i, target_name in enumerate(expected_order):
            if target_name in msg.name:
                idx = msg.name.index(target_name)
                self.current_positions[i] = msg.position[idx]
                self.current_velocities[i] = msg.velocity[idx]

    def control_loop(self):
        """
        THIS IS YOUR RESEARCH SANDBOX.
        Replace the logic below with your Neural Network inference, 
        Impedance Control math, or PID gravity compensation.
        """
        
        t = time.time() - self.start_time
        
        # --- EXAMPLE: SINE WAVE EXCITATION DISTURBANCE ---
        # We will inject a gentle oscillating torque into the elbow and shoulder
        # while keeping the wrists limp (0.0)
        
        tau_shoulder = 15.0 * math.sin(2.0 * t)  # 15 Nm sine wave
        tau_elbow = 10.0 * math.cos(2.0 * t)     # 10 Nm cosine wave
        
        # The array MUST match the exact 7-DOF sequence of your YAML file
        action_torques = [
            0.0,            # shoulder_pan_joint1
            0.0,            # shoulder_pan_joint
            tau_shoulder,   # shoulder_lift_joint
            tau_elbow,      # elbow_joint
            0.0,            # wrist_1_joint
            0.0,            # wrist_2_joint
            0.0             # wrist_3_joint
        ]
        
        # Publish the torques to the Gazebo motors
        msg = Float64MultiArray()
        msg.data = action_torques
        self.torque_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CustomTorqueController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down AI Controller...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()