import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def inject_candle_urdf(urdf_xml: str, values: dict) -> str:
    """Replace __CANDLE_<joint>__ with candle_pose. Same rule as ur5_controller.robot."""
    out = urdf_xml
    for name, value in values.items():
        token = f'__CANDLE_{name}__'
        if token not in out:
            raise KeyError(f'URDF missing {token}')
        out = out.replace(token, f'{float(value):.10g}')
    if '__CANDLE_' in out:
        raise KeyError('URDF has a candle placeholder with no SRDF value')
    return out


def candle_values(srdf_path: str) -> dict:
    import xml.etree.ElementTree as ET
    root = ET.parse(srdf_path).getroot()
    for gs in root.findall('group_state'):
        if gs.get('name') == 'candle_pose':
            return {j.get('name'): float(j.get('value')) for j in gs.findall('joint')}
    raise KeyError(f'group_state candle_pose not in {srdf_path}')


def _build_gz_sim(context, *args, **kwargs):
    """Compose gz_args at runtime: optional -s and --render-engine, then -r world."""
    pkg_share = get_package_share_directory('ur5_description')
    world_file = os.path.join(pkg_share, 'worlds', 'empty.sdf')

    headless = LaunchConfiguration('gz_headless').perform(context).lower() in (
        'true', '1', 'yes',
    )
    render_engine = LaunchConfiguration('gz_render_engine').perform(context).strip()
    if not render_engine:
        render_engine = os.environ.get('GZ_SIM_RENDER_ENGINE', '').strip()

    gz_parts = []
    if headless:
        gz_parts.append('-s')
    gz_parts.append('-r')
    if render_engine:
        gz_parts.extend(['--render-engine', render_engine])
    gz_parts.append(world_file)

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('ros_gz_sim'),
                    'launch',
                    'gz_sim.launch.py',
                )
            ),
            launch_arguments={'gz_args': ' '.join(gz_parts)}.items(),
        )
    ]


def generate_launch_description():
    pkg_name = 'ur5_description'
    pkg_share = get_package_share_directory(pkg_name)
    urdf_file = os.path.join(pkg_share, 'urdf', 'ur5.urdf')
    controller_yaml = os.path.join(
        get_package_share_directory('ur5_moveit_config'),
        'config',
        'ros2_controllers.yaml',
    )

    srdf_file = os.path.join(
        get_package_share_directory('ur5_moveit_config'),
        'config',
        'ur57_arm.srdf',
    )
    with open(urdf_file, 'r') as infp:
        robot_desc = inject_candle_urdf(
            infp.read().replace('__ROS2_CONTROLLERS_YAML__', controller_yaml),
            candle_values(srdf_file),
        )

    workspace_share_dir = os.path.join(pkg_share, '..')
    env_gz = AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', workspace_share_dir)

    # Default false so bare gazebo_sim / run_sim opens a GUI; ur5sim overrides to true.
    gz_headless_arg = DeclareLaunchArgument(
        'gz_headless',
        default_value='false',
        description=(
            'If true, run gz sim server only (-s). Default false for GUI; '
            'set true for headless / ARM64 software-GL bring-up.'
        ),
    )
    gz_render_engine_arg = DeclareLaunchArgument(
        'gz_render_engine',
        default_value='',
        description=(
            'Optional gz render engine (e.g. ogre, ogre2). Empty uses Gazebo default. '
            'Override also via env GZ_SIM_RENDER_ENGINE. Do not bake a platform-specific value.'
        ),
    )

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_desc}],
    )

    gazebo = OpaqueFunction(function=_build_gz_sim)

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-string', robot_desc, '-name', 'ur5_arm', '-allow_renaming', 'true'],
        output='screen',
    )

    # Original event-driven spawner chain (JSB → JTC → FEC inactive).
    # Brief delay after create lets gz_ros2_control finish CM bring-up before
    # configure/activate (GUI path otherwise races: already-loaded + configure fail).
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager-timeout', '60'],
        output='screen',
    )

    load_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ur_manipulator_controller', '--controller-manager-timeout', '60'],
        output='screen',
    )

    load_effort_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'forward_effort_controller',
            '--inactive',
            '--controller-manager-timeout', '60',
        ],
        output='screen',
    )

    delayed_jsb = TimerAction(
        period=2.0,
        actions=[load_joint_state_broadcaster],
    )

    return LaunchDescription([
        gz_headless_arg,
        gz_render_engine_arg,
        rsp_node,
        env_gz,
        gazebo,
        spawn_entity,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[delayed_jsb],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_arm_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_arm_controller,
                on_exit=[load_effort_controller],
            )
        ),
    ])
