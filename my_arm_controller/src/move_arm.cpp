#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>("move_arm_node");

  // 1. Declare our dynamic parameters (with the old coordinate as a default fallback)
  node->declare_parameter<double>("target_x", 0.3);
  node->declare_parameter<double>("target_y", 0.0);
  node->declare_parameter<double>("target_z", 0.5);

  // 2. Read the parameters passed from the terminal
  double tx = node->get_parameter("target_x").as_double();
  double ty = node->get_parameter("target_y").as_double();
  double tz = node->get_parameter("target_z").as_double();

  RCLCPP_INFO(node->get_logger(), "Moving arm to XYZ: [%.2f, %.2f, %.2f]", tx, ty, tz);

  auto const move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "ur_manipulator");

  // 3. Apply the dynamic coordinates to the pose target
  geometry_msgs::msg::Pose target_pose;
  target_pose.orientation.w = 1.0;
  target_pose.position.x = tx;
  target_pose.position.y = ty;
  target_pose.position.z = tz;

  move_group->setPoseTarget(target_pose);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
    move_group->execute(plan);
  } else {
    RCLCPP_ERROR(node->get_logger(), "Planner failed to find a valid path to that coordinate!");
  }

  rclcpp::shutdown();
  return 0;
}