#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import SwitchController

class ControllerSwitcher(Node):
    def __init__(self):
        super().__init__('mode_switcher_node')
        self.client = self.create_client(SwitchController, '/controller_manager/switch_controller')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for Controller Manager service...')
            
    def switch_to_ai_mode(self):
        self.get_logger().info('Unplugging MoveIt... Plugging in AI Effort Controller...')
        req = SwitchController.Request()
        
        # Turn ON the raw effort controller
        req.activate_controllers = ['forward_effort_controller']
        # Turn OFF the MoveIt trajectory controller
        req.deactivate_controllers = ['ur_manipulator_controller']
        
        # STRICT means if one fails, the whole operation aborts (Safety First)
        req.strictness = SwitchController.Request.STRICT
        
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result().ok:
            self.get_logger().info('SUCCESS: Robot is now in AI / Raw Torque Mode!')
        else:
            self.get_logger().error('FAILED to switch controllers.')

    def switch_to_moveit_mode(self):
        self.get_logger().info('Unplugging AI... Restoring MoveIt Trajectory Control...')
        req = SwitchController.Request()
        req.activate_controllers = ['ur_manipulator_controller']
        req.deactivate_controllers = ['forward_effort_controller']
        req.strictness = SwitchController.Request.STRICT
        
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result().ok:
            self.get_logger().info('SUCCESS: Robot is now in MoveIt Mode!')
        else:
            self.get_logger().error('FAILED to switch controllers.')

def main(args=None):
    rclpy.init(args=args)
    switcher = ControllerSwitcher()
    
    # Prompt the user for which mode they want
    print("\n--- UR5 Control Mode Switcher ---")
    print("1: AI Research Mode (Raw Effort Control)")
    print("2: MoveIt Reset Mode (Trajectory Control)")
    choice = input("Select mode (1 or 2): ")
    
    if choice == '1':
        switcher.switch_to_ai_mode()
    elif choice == '2':
        switcher.switch_to_moveit_mode()
    else:
        print("Invalid selection.")
        
    switcher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()