from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("ur57_arm", package_name="ur5_moveit_config").to_moveit_configs()

    rmw_implementation_arg = DeclareLaunchArgument(
        'rmw_implementation',
        default_value='rmw_fastrtps_cpp',
        description='RMW implementation for the planner (must match simulator terminals)',
    )

    set_rmw = SetEnvironmentVariable(
        name='RMW_IMPLEMENTATION',
        value=LaunchConfiguration('rmw_implementation'),
    )

    rmw_log = LogInfo(msg=['[planner] RMW_IMPLEMENTATION=', LaunchConfiguration('rmw_implementation')])

    planner_node = Node(
        package="ur5_controller",
        executable="multi_waypoint_planner",
        output="screen",
        emulate_tty=True,
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            moveit_config.trajectory_execution,
            moveit_config.planning_scene_monitor,
            {'use_sim_time': True}
        ],
    )

    return LaunchDescription([
        rmw_implementation_arg,
        set_rmw,
        rmw_log,
        planner_node,
    ])
