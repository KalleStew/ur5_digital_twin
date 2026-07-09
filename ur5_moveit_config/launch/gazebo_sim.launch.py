import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

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
        ),
        # Force Gazebo interface to use sim time
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Delay MoveIt startup so /robot_description and /controller_manager are present.
    move_group_launch = TimerAction(
        period=6.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(moveit_share, 'launch', 'move_group.launch.py')
                ),
                # CRITICAL: Force MoveGroup to synchronize with Gazebo's /clock
                launch_arguments={'use_sim_time': 'true'}.items()
            )
        ],
    )

    # Bridge the Gazebo Harmonic clock to ROS 2 native /clock topic
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
        name='clock_bridge'
    )    

    # Start RViz after move_group so the semantic model is already available.
    rviz_launch = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(moveit_share, 'launch', 'moveit_rviz.launch.py')
                ),
                # CRITICAL: Force RViz TF tree to synchronize with Gazebo's /clock
                launch_arguments={'use_sim_time': 'true'}.items()
            )
        ],
    )

    return LaunchDescription([
        start_zenoh_router_arg,
        zenoh_router,
        sim_launch,
        move_group_launch,
        rviz_launch,
        clock_bridge
    ])
