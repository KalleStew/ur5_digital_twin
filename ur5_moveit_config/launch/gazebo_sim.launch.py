import os
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # -----------------------------------------------------------------------
    # 1. Build unified MoveIt config — single source of truth for all nodes.
    #    This replaces the old multi-file IncludeLaunchDescription chain and
    #    eliminates the hardcoded absolute path in the URDF gazebo plugin block.
    # -----------------------------------------------------------------------
    moveit_config = (
        MoveItConfigsBuilder("ur5_arm", package_name="ur5_moveit_config")
        .robot_description(file_path="urdf/ur5.urdf")
        .robot_description_semantic(file_path="config/ur5.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/ros2_controllers.yaml")
        .to_moveit_configs()
    )

    # -----------------------------------------------------------------------
    # 2. Gazebo resource path — IGN_GAZEBO_RESOURCE_PATH retired with Ignition.
    #    Only GZ_SIM_RESOURCE_PATH is honoured by Gazebo Harmonic+.
    # -----------------------------------------------------------------------
    workspace_share = os.path.join(
        get_package_share_directory("ur5_description"), "..")
    env_gz = AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", workspace_share)

    # -----------------------------------------------------------------------
    # 3. Gazebo simulator
    # -----------------------------------------------------------------------
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": "-r empty.sdf"}.items(),
    )

    # -----------------------------------------------------------------------
    # 4. Robot State Publisher — description passed in-memory from builder,
    #    no disk path dependency at runtime.
    # -----------------------------------------------------------------------
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": True},
        ],
    )

    # -----------------------------------------------------------------------
    # 5. Spawn robot into Gazebo via /robot_description topic
    # -----------------------------------------------------------------------
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "/robot_description", "-name", "ur5_arm"],
        output="screen",
    )

    # -----------------------------------------------------------------------
    # 6. MoveIt move_group — receives full config dict from builder
    # -----------------------------------------------------------------------
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
        ],
    )

    # -----------------------------------------------------------------------
    # 7. RViz with MoveIt plugin — kinematics/semantic passed in-memory
    # -----------------------------------------------------------------------
    rviz_config = os.path.join(
        get_package_share_directory("ur5_moveit_config"), "config", "moveit.rviz")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True},
        ],
    )

    # -----------------------------------------------------------------------
    # 8. Controller spawners — 30s timeout handles VM/WSL2 load lag.
    #    forward_effort_controller spawned inactive; activate via service call.
    # -----------------------------------------------------------------------
    spawn_broadcaster = Node(
        package="controller_manager", executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30",
        ],
    )
    spawn_trajectory = Node(
        package="controller_manager", executable="spawner",
        arguments=[
            "ur_manipulator_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30",
        ],
    )
    spawn_effort = Node(
        package="controller_manager", executable="spawner",
        arguments=[
            "forward_effort_controller",
            "--controller-manager", "/controller_manager",
            "--inactive",
            "--controller-manager-timeout", "30",
        ],
    )

    return LaunchDescription([
        env_gz,
        gazebo,
        rsp,
        spawn,
        move_group,
        rviz,
        spawn_broadcaster,
        spawn_trajectory,
        spawn_effort,
    ])
