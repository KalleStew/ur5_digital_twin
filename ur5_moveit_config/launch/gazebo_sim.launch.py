import os
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # ── 1. Build the complete MoveIt configuration object in memory.
    #       This replaces the old multi-file IncludeLaunchDescription chain
    #       (rsp.launch.py + move_group.launch.py + moveit_rviz.launch.py).
    #       robot_description is read once here and passed to every downstream
    #       node, guaranteeing a single source of truth. ─────────────────────
    moveit_config = (
        MoveItConfigsBuilder("ur5_arm", package_name="ur5_moveit_config")
        .robot_description(file_path="urdf/ur5.urdf")
        .robot_description_semantic(file_path="config/ur5.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/ros2_controllers.yaml")
        .to_moveit_configs()
    )

    # ── 2. GZ_SIM_RESOURCE_PATH — lets Gazebo Harmonic resolve mesh URIs
    #       (package://ur5_description/meshes/...) at spawn time. ───────────
    workspace_share_dir = os.path.join(
        get_package_share_directory('ur5_description'), '..'
    )
    env_gz = AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', workspace_share_dir)

    # ── 3. Gazebo Harmonic (ros_gz_sim) ──────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # ── 4. Robot State Publisher — receives robot_description from the
    #       MoveItConfigsBuilder dict (no separate URDF file read). ──────────
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            moveit_config.robot_description,
            {'use_sim_time': True},
        ],
    )

    # ── 5. Spawn robot into Gazebo via the /robot_description topic. ─────────
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/robot_description', '-name', 'ur57_arm'],
        output='screen',
    )

    # ── 6. move_group — MoveIt planning server. ───────────────────────────────
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': True},
        ],
    )

    # ── 7. RViz with MoveIt plugin. ───────────────────────────────────────────
    rviz_config = os.path.join(
        get_package_share_directory('ur5_moveit_config'), 'config', 'moveit.rviz'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        arguments=['-d', rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {'use_sim_time': True},
        ],
    )

    # ── 8. Controller Spawners.
    #       --controller-manager-timeout 30 absorbs VM/WSL2 load lag.
    #       forward_effort_controller starts --inactive so the position
    #       controller holds the arm on boot; Python scripts switch on demand.
    spawn_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
        ],
    )
    spawn_trajectory = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'ur_manipulator_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
        ],
    )
    spawn_effort = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'forward_effort_controller',
            '--controller-manager', '/controller_manager',
            '--inactive',
            '--controller-manager-timeout', '30',
        ],
    )

    return LaunchDescription([
        env_gz,
        gazebo,
        rsp_node,
        spawn_robot,
        move_group_node,
        rviz_node,
        spawn_broadcaster,
        spawn_trajectory,
        spawn_effort,
    ])