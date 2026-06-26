import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'ur5_description'
    pkg_share = get_package_share_directory(pkg_name)
    urdf_file = os.path.join(pkg_share, 'urdf', 'ur5.urdf')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # 1. Start Robot State Publisher (Provides transforms to the system)
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_desc}]
    )

    # 2. Start Gazebo Sim — 'ign gazebo' retired; use ros_gz_sim launch include
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # 3. Spawn the robot into Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-string', robot_desc, '-name', 'ur5_arm', '-allow_renaming', 'true'],
        output='screen'
    )

    # 4. Load the Joint State Broadcaster (spawner replaces deprecated load_controller)
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '30'],
        output='screen'
    )

    # 5. Load the Trajectory Controller
    load_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ur_manipulator_controller',
                   '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '30'],
        output='screen'
    )

    # We use event handlers to ensure controllers load ONLY AFTER the robot is fully spawned
    return LaunchDescription([
        rsp_node,
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