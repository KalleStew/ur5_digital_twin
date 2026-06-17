import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from controller_manager_msgs.srv import SwitchController

class AITorqueController(Node):
    def __init__(self):
        super().__init__('ai_torque_controller')
        
        # 1. Subscriptions: Read Live Data (Position, Velocity, Torque/Effort)
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.sensor_callback,
            10)
            
        # 2. Publishers: Send Raw Torque Commands
        self.torque_pub = self.create_publisher(
            Float64MultiArray, 
            '/forward_effort_controller/commands', 
            10)
            
        # 3. Services: Mode Switching
        self.switch_client = self.create_client(SwitchController, '/controller_manager/switch_controller')
        
        # Internal state
        self.current_positions = []
        self.current_efforts = []
        self.ai_active = False

    def sensor_callback(self, msg):
        """ This runs hundreds of times a second to read the Maestro/Gazebo feedback """
        self.current_positions = msg.position
        self.current_efforts = msg.effort
        
        # Example Fault Detection Logic:
        # If the effort on the shoulder joint exceeds 100 Nm, trigger an emergency response!
        if len(self.current_efforts) > 0 and abs(self.current_efforts[2]) > 100.0:
            self.get_logger().error("FAULT DETECTED: Torque spike on shoulder lift joint!")

    def activate_torque_mode(self):
        """ Disables MoveIt's position controller and activates raw torque mode """
        self.get_logger().info('Waiting for controller manager service...')
        self.switch_client.wait_for_service()
        
        req = SwitchController.Request()
        req.activate_controllers = ['forward_effort_controller']
        req.deactivate_controllers = ['ur_manipulator_controller']
        req.strictness = SwitchController.Request.BEST_EFFORT
        
        future = self.switch_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result().ok:
            self.get_logger().info('SUCCESS: Robot is now in raw Torque Control Mode!')
            self.ai_active = True
        else:
            self.get_logger().error('Failed to switch controllers.')

    def run_ai_control_loop(self):
        """ The main control loop for disturbances and AI trajectory generation """
        if not self.ai_active:
            return
            
        msg = Float64MultiArray()
        
        # --- YOUR AI / CONTROL LOGIC GOES HERE ---
        # For this test, we are just applying a static torque vector to the 7 joints
        # WARNING: Without a programmed PID loop here, gravity will immediately pull the arm down!
        torque_commands = [0.0, 0.0, 50.0, 0.0, 0.0, 0.0, 0.0] 
        # -----------------------------------------
        
        msg.data = torque_commands
        self.torque_pub.publish(msg)
        self.get_logger().debug(f'Published Torques: {torque_commands}')

def main(args=None):
    rclpy.init(args=args)
    node = AITorqueController()
    
    # 1. Switch the robot from Position to Torque mode
    node.activate_torque_mode()
    
    # 2. Run the AI loop at 100 Hz (0.01 seconds)
    timer_period = 0.01  
    node.create_timer(timer_period, node.run_ai_control_loop)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down AI controller...")
    finally:
        # Best practice: Switch back to position control on shutdown to prevent the arm from dropping
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()