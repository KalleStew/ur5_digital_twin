from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("ur57_arm", package_name="my_arm_moveit_config").to_moveit_configs()

    planner_node = Node(
        package="my_arm_controller",
        executable="multi_waypoint_planner",
        output="screen",
        # Opens a new terminal
        prefix="gnome-terminal --", 
        parameters=[
            moveit_config.robot_description_kinematics,
            {'use_sim_time': True}
        ],
    )

    return LaunchDescription([planner_node])