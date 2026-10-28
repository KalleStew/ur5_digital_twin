import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, OpaqueFunction, TimerAction, SetEnvironmentVariable
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
        # Match project invariant / ur5env. FastDDS here + Zenoh in the shell
        # makes the graph invisible to ros2 CLI ("nothing launched").
        default_value='rmw_zenoh_cpp',
        description='RMW for this launch. Default rmw_zenoh_cpp; override only if intentional.',
    )

    gz_headless_arg = DeclareLaunchArgument(
        'gz_headless',
        default_value='false',
        description=(
            'Forwarded to sim.launch.py: run gz server only (-s). '
            'Default false so bare run_sim opens Gazebo GUI; set true for headless.'
        ),
    )
    gz_render_engine_arg = DeclareLaunchArgument(
        'gz_render_engine',
        default_value='',
        description='Forwarded to sim.launch.py / env GZ_SIM_RENDER_ENGINE',
    )
    start_rviz_arg = DeclareLaunchArgument(
        'start_rviz',
        default_value='true',
        description='Launch MoveIt RViz. Default true (matches original GUI bring-up); set false for headless.',
    )
    run_preflight_arg = DeclareLaunchArgument(
        'run_preflight',
        default_value='true',
        description='After controllers start, move the arm to candle_pose and check the settle.',
    )

    def _preflight(context, *args, **kwargs):
        flag = LaunchConfiguration('run_preflight').perform(context).lower()
        if flag not in ('true', '1', 'yes'):
            return []
        try:
            get_package_share_directory('ur5_controller')
        except Exception:
            return [LogInfo(msg='ERROR: ur5_controller not found; Candle pre-flight was not started')]
        return [Node(
            package='ur5_controller',
            executable='interactive_control.py',
            arguments=['--candle', '--wait-for-bringup', '120'],
            output='screen',
        )]

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
        ),
        launch_arguments={
            'gz_headless': LaunchConfiguration('gz_headless'),
            'gz_render_engine': LaunchConfiguration('gz_render_engine'),
        }.items(),
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

    # Bridge the Gazebo clock, and the physics-stamped joint states used by Brain B and telemetry.
    clock_bridge = TimerAction(
        period=1.1,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                           '/ur5/gz/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model'],
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
                ),
                condition=IfCondition(LaunchConfiguration('start_rviz')),
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
        gz_headless_arg,
        gz_render_engine_arg,
        start_rviz_arg,
        run_preflight_arg,
        zenoh_router,
        delayed_sim_launch,
        move_group_launch,
        rviz_launch,
        clock_bridge,
        OpaqueFunction(function=_preflight),
    ])
