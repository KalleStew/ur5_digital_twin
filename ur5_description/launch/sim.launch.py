import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = 'ur5_description'
    pkg_share = get_package_share_directory(pkg_name)
    urdf_file = os.path.join(pkg_share, 'urdf', 'ur5.urdf')
    controller_yaml = os.path.join(
        get_package_share_directory('ur5_moveit_config'),
        'config',
        'ros2_controllers.yaml',
    )

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read().replace('__ROS2_CONTROLLERS_YAML__', controller_yaml)

    # Expose meshes to Gazebo Harmonic resource path
    workspace_share_dir = os.path.join(pkg_share, '..')
    env_gz = AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', workspace_share_dir)

    # 1. Start Robot State Publisher (Provides transforms to the system)
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_desc}]
    )

    # 2. Start Gazebo Harmonic via ros_gz_sim (replaces legacy 'ign gazebo' command)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # 3. Spawn the robot into Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-string', robot_desc, '-name', 'ur5_arm', '-allow_renaming', 'true'],
        output='screen'
    )

    # 4. Spawn the Joint State Broadcaster
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager-timeout', '30'],
        output='screen',
    )

    # 5. Spawn the MoveIt trajectory controller
    load_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ur_manipulator_controller', '--controller-manager-timeout', '30'],
        output='screen',
    )

    # We use event handlers to ensure controllers load ONLY AFTER the robot is fully spawned
    return LaunchDescription([
        rsp_node,
        env_gz,
        gazebo,
        spawn_entity,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_arm_controller],
            )
        ),
    ])