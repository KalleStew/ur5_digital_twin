import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    workspace_share_dir = os.path.join(get_package_share_directory('ur5_description'), '..')
    env_ign = AppendEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', workspace_share_dir)
    env_gz = AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', workspace_share_dir)

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/robot_description', '-name', 'ur57_arm'],
        output='screen',
    )

    moveit_config_dir = get_package_share_directory('ur5_moveit_config')
    
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(moveit_config_dir, 'launch', 'rsp.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items()
    )
    
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(moveit_config_dir, 'launch', 'move_group.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(moveit_config_dir, 'launch', 'moveit_rviz.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Spawners will patiently wait up to 30s for the Mac VM, then fire instantly
    spawn_broadcaster = Node(package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager", "--controller-manager-timeout", "30"])
    spawn_trajectory = Node(package="controller_manager", executable="spawner", arguments=["ur_manipulator_controller", "--controller-manager", "/controller_manager", "--controller-manager-timeout", "30"])
    spawn_effort = Node(package="controller_manager", executable="spawner", arguments=["forward_effort_controller", "--controller-manager", "/controller_manager", "--inactive", "--controller-manager-timeout", "30"])

    return LaunchDescription([
        env_ign, env_gz, gazebo, spawn, rsp, move_group, rviz,
        spawn_broadcaster, spawn_trajectory, spawn_effort
    ])