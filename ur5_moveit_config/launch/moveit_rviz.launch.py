from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch
from launch_ros.actions import SetParameter

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("ur57_arm", package_name="ur5_moveit_config").to_moveit_configs()
    
    # Generate the base RViz launch description
    ld = generate_moveit_rviz_launch(moveit_config)
    
    # CRITICAL: Force the RViz TF tree to synchronize with Gazebo's /clock
    ld.add_action(SetParameter(name='use_sim_time', value=True))
    
    return ld