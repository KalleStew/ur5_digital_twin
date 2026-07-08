import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    description_share = get_package_share_directory('ur5_description')
    moveit_share = get_package_share_directory('ur5_moveit_config')

    start_zenoh_router_arg = DeclareLaunchArgument(
        'start_zenoh_router',
        default_value='true',
        description='Start a local rmw_zenoh router for ROS graph discovery',
    )

    zenoh_router = ExecuteProcess(
        cmd=['ros2', 'run', 'rmw_zenoh_cpp', 'rmw_zenohd'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_zenoh_router')),
    )

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_share, 'launch', 'sim.launch.py')
        )
    )

    # Delay MoveIt startup so /robot_description and /controller_manager are present.
    move_group_launch = TimerAction(
        period=6.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(moveit_share, 'launch', 'move_group.launch.py')
                )
            )
        ],
    )

    # Start RViz after move_group so the semantic model is already available.
    rviz_launch = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(moveit_share, 'launch', 'moveit_rviz.launch.py')
                )
            )
        ],
    )

    return LaunchDescription([
        start_zenoh_router_arg,
        zenoh_router,
        sim_launch,
        move_group_launch,
        rviz_launch,
    ])
