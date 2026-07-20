import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter

def generate_launch_description():
    description_share = get_package_share_directory('ur5_description')
    moveit_share = get_package_share_directory('ur5_moveit_config')

    # Ensure all nodes launched from this description use simulation time.
    global_use_sim_time = SetParameter(name='use_sim_time', value=True)

    start_zenoh_router_arg = DeclareLaunchArgument(
        'start_zenoh_router',
        default_value='false',
        description='Start a local rmw_zenoh router for ROS graph discovery',
    )

    rmw_implementation_arg = DeclareLaunchArgument(
        'rmw_implementation',
        default_value='rmw_fastrtps_cpp',
        description='RMW implementation for this launch (e.g., rmw_fastrtps_cpp or rmw_zenoh_cpp)',
    )

    set_rmw = SetEnvironmentVariable(
        name='RMW_IMPLEMENTATION',
        value=LaunchConfiguration('rmw_implementation'),
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
                ),
            )
        ],
    )

    # Bridge the Gazebo Harmonic clock to ROS 2 native /clock topic
    clock_bridge = TimerAction(
        period=1.1,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
                output='screen',
                name='clock_bridge'
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

    # Give the router a short grace period before Gazebo starts publishing clock.
    delayed_sim_launch = TimerAction(
        period=1.0,
        actions=[sim_launch],
    )

    return LaunchDescription([
        global_use_sim_time,
        rmw_implementation_arg,
        set_rmw,
        start_zenoh_router_arg,
        zenoh_router,
        delayed_sim_launch,
        move_group_launch,
        rviz_launch,
        clock_bridge
    ])
