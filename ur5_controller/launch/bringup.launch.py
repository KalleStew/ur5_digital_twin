"""Full bring-up: Gazebo + controllers + MoveIt (ur5_moveit_config), then the Candle pre-flight.

The pre-flight waits for the spawner chain itself, so no timer guesses the bring-up duration.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGS = {
    'gz_headless': 'true',
    'start_rviz': 'false',
    'gz_render_engine': '',
}


def generate_launch_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ur5_moveit_config'), 'launch', 'gazebo_sim.launch.py')),
        launch_arguments={k: LaunchConfiguration(k) for k in ARGS}.items(),
    )
    preflight = Node(
        package='ur5_controller',
        executable='preflight.py',
        arguments=['--wait-for-bringup', '120'],
        output='screen',
    )
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=v) for k, v in ARGS.items()] + [sim, preflight])
