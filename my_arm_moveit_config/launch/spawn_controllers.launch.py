# from moveit_configs_utils import MoveItConfigsBuilder
# from moveit_configs_utils.launches import generate_spawn_controllers_launch


# def generate_launch_description():
#     moveit_config = MoveItConfigsBuilder("ur5_arm", package_name="my_arm_moveit_config").to_moveit_configs()
#     return generate_spawn_controllers_launch(moveit_config)

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_spawn_controllers_launch
from launch_ros.actions import Node # We need to import the Node function!

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("ur5_arm", package_name="my_arm_moveit_config").to_moveit_configs()
    
    # 1. Generate the standard controllers MoveIt expects
    ld = generate_spawn_controllers_launch(moveit_config)
    
    # 2. Define our custom torque controller (set to inactive!)
    forward_effort_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_effort_controller", "-c", "/controller_manager", "--inactive"],
    )
    
    # 3. Add our custom node to the launch list
    ld.add_action(forward_effort_controller_spawner)
    
    return ld