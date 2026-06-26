from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("ur5_arm", package_name="ur5_moveit_config").to_moveit_configs()

    planner_node = Node(
        package="ur5_controller",
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