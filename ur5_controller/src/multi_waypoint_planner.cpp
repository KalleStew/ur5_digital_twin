#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <thread>
#include <iostream>
#include <fstream>
#include <string>
#include <cmath> 

class MultiWaypointPlanner : public rclcpp::Node {
public:
  // ...
    MultiWaypointPlanner() : Node("multi_waypoint_planner", rclcpp::NodeOptions().append_parameter_override("use_sim_time", true)) {    click_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/clicked_point", 10,
      std::bind(&MultiWaypointPlanner::point_callback, this, std::placeholders::_1)
    );
    
    display_pub_ = this->create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path", 10);
    
    RCLCPP_INFO(this->get_logger(), "==========================================");
    RCLCPP_INFO(this->get_logger(), " Multi-Waypoint Planner & Exporter Ready! ");
    RCLCPP_INFO(this->get_logger(), "==========================================");
  }

  void setup_move_group() {
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "ur_manipulator");
    
    // Save the starting pose so we can return to it later
    initial_pose_ = move_group_->getCurrentPose().pose;
    waypoints_.push_back(initial_pose_); 
  }

  void run_terminal_loop() {
    std::string command;
    while (rclcpp::ok()) {
      std::cout << "\n[MENU] Enter a command:\n"
                << "  'plan'                   -> Preview path in RViz\n"
                << "  'execute'                -> Move the simulated robot\n"
                << "  'return home'            -> Add the starting position to the end of the path\n"
                << "  'export dense'           -> Save full interpolated path (Joints in Deg)\n"
                << "  'export sparse'          -> Save clicked points (XYZ/RPY in Deg)\n"
                << "  'export waypoint joints' -> Save Joint Angles of ONLY the clicked points (in Deg)\n"
                << "  'clear'                  -> Reset waypoints\n"
                << "Command: ";
                
      std::getline(std::cin, command);

      if (command == "plan") {
        if (waypoints_.size() < 2) {
          RCLCPP_WARN(this->get_logger(), "Need more waypoints. Use 'Publish Point' in RViz.");
          continue;
        }

        // jump_threshold removed in MoveIt 2 / ROS 2 LL; use CartesianInterpolator struct
        moveit::planning_interface::MoveGroupInterface::CartesianInterpolator interp;
        interp.max_step = 0.01; // 1 cm resolution

        double fraction = move_group_->computeCartesianPath(waypoints_, interp, calculated_trajectory_);
        
        if (fraction < 1.0) {
            RCLCPP_WARN(this->get_logger(), "Path Calculation: %.2f%%. The straight line hit the table or joint limits!", fraction * 100.0);
        } else {
            RCLCPP_INFO(this->get_logger(), "Path Calculation: 100%% successful.");
        }

        // Null check to prevent Segfault if MoveIt drops the state
        auto current_state = move_group_->getCurrentState();
        if (!current_state) {
            RCLCPP_ERROR(this->get_logger(), "Failed to fetch robot state! Ensure Gazebo is running.");
            continue;
        }

        // Publish orange preview animation
        moveit_msgs::msg::DisplayTrajectory display_msg;
        moveit::core::robotStateToRobotStateMsg(*current_state, display_msg.trajectory_start);
        display_msg.trajectory.push_back(calculated_trajectory_);
        display_pub_->publish(display_msg);

      } else if (command == "execute") {
        if (calculated_trajectory_.joint_trajectory.points.empty()) {
          RCLCPP_WARN(this->get_logger(), "You must 'plan' successfully before executing.");
          continue;
        }
        RCLCPP_INFO(this->get_logger(), "Executing trajectory...");
        move_group_->execute(calculated_trajectory_);
        
        // Reset waypoints starting from new location
        waypoints_.clear();
        waypoints_.push_back(move_group_->getCurrentPose().pose);
        calculated_trajectory_ = moveit_msgs::msg::RobotTrajectory();

      } else if (command == "return home") {
        // Feature: Finish in starting position
        waypoints_.push_back(initial_pose_);
        RCLCPP_INFO(this->get_logger(), "Appended starting position to the path.");

      } else if (command == "export dense") {
        export_dense_csv();

      } else if (command == "export sparse") {
        export_sparse_csv();

      } else if (command == "export waypoint joints") {
        export_waypoint_joints_csv();

      } else if (command == "clear") {
        waypoints_.clear();
        waypoints_.push_back(move_group_->getCurrentPose().pose);
        calculated_trajectory_ = moveit_msgs::msg::RobotTrajectory();
        RCLCPP_INFO(this->get_logger(), "Waypoints cleared.");
      }
    }
  }

private:
  void point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    geometry_msgs::msg::Pose new_waypoint = waypoints_.back(); // Keep previous orientation
    new_waypoint.position.x = msg->point.x;
    new_waypoint.position.y = msg->point.y;
    new_waypoint.position.z = msg->point.z;

    waypoints_.push_back(new_waypoint);
    RCLCPP_INFO(this->get_logger(), "Added Target %zu -> [X: %.2f, Y: %.2f, Z: %.2f]",
                waypoints_.size() - 1, msg->point.x, msg->point.y, msg->point.z);
  }

  // EXPORT 1: Full Interpolated Path (Hundreds of points)
  void export_dense_csv() {
    if (calculated_trajectory_.joint_trajectory.points.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No trajectory calculated! Type 'plan' first.");
      return;
    }

    std::ofstream file("trajectory_dense_export.csv");
    if (!file.is_open()) return;

    auto joint_traj = calculated_trajectory_.joint_trajectory;
    for (const auto& name : joint_traj.joint_names) {
      file << name << "_[deg],";
    }
    file << "time_from_start_sec\n";

    for (const auto& point : joint_traj.points) {
      for (double rad_pos : point.positions) {
        file << (rad_pos * (180.0 / M_PI)) << ",";
      }
      double time_sec = point.time_from_start.sec + (point.time_from_start.nanosec * 1e-9);
      file << time_sec << "\n";
    }
    file.close();
    RCLCPP_INFO(this->get_logger(), "Saved %zu dense waypoints.", joint_traj.points.size());
  }

  // EXPORT 2: Clicked Cartesian Points (XYZ/RPY)
  void export_sparse_csv() {
    if (waypoints_.size() < 2) return;
    std::ofstream file("trajectory_sparse_export.csv");
    if (!file.is_open()) return;

    file << "Waypoint_Index,X_[m],Y_[m],Z_[m],Roll_[deg],Pitch_[deg],Yaw_[deg]\n";
    for (size_t i = 0; i < waypoints_.size(); ++i) {
      const auto& pos = waypoints_[i].position;
      const auto& q = waypoints_[i].orientation;

      double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
      double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
      double roll = std::atan2(sinr_cosp, cosr_cosp);

      double sinp = std::sqrt(1 + 2 * (q.w * q.y - q.x * q.z));
      double cosp = std::sqrt(1 - 2 * (q.w * q.y - q.x * q.z));
      double pitch = 2 * std::atan2(sinp, cosp) - M_PI / 2;

      double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
      double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
      double yaw = std::atan2(siny_cosp, cosy_cosp);

      file << i << "," << pos.x << "," << pos.y << "," << pos.z << ","
           << roll * (180.0 / M_PI) << "," << pitch * (180.0 / M_PI) << "," << yaw * (180.0 / M_PI) << "\n";
    }
    file.close();
    RCLCPP_INFO(this->get_logger(), "Saved %zu target coordinates.", waypoints_.size());
  }

  // EXPORT 3: Joint Angles of ONLY the Clicked Points using Inverse Kinematics
  void export_waypoint_joints_csv() {
    if (waypoints_.size() < 2) return;
    std::ofstream file("waypoint_joints_export.csv");
    if (!file.is_open()) return;

    file << "Waypoint_Index,J1_[deg],J2_[deg],J3_[deg],J4_[deg],J5_[deg],J6_[deg],J7_[deg]\n";

    auto kinematic_state = move_group_->getCurrentState();
    const auto* joint_model_group = kinematic_state->getJointModelGroup("ur_manipulator");

    for (size_t i = 0; i < waypoints_.size(); ++i) {
      // Calculate IK for this specific waypoint
      bool found_ik = kinematic_state->setFromIK(joint_model_group, waypoints_[i], 0.1);
      if (found_ik) {
        std::vector<double> joint_values;
        kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
        file << i << ",";
        for (double rad : joint_values) {
          file << (rad * (180.0 / M_PI)) << ",";
        }
        file << "\n";
      } else {
        RCLCPP_WARN(this->get_logger(), "Could not find IK solution for waypoint %zu!", i);
        file << i << ",FAILED,FAILED,FAILED,FAILED,FAILED,FAILED,FAILED\n";
      }
    }
    file.close();
    RCLCPP_INFO(this->get_logger(), "Exported IK joint angles for %zu waypoints.", waypoints_.size());
  }

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr click_sub_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_pub_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  
  geometry_msgs::msg::Pose initial_pose_;
  std::vector<geometry_msgs::msg::Pose> waypoints_;
  moveit_msgs::msg::RobotTrajectory calculated_trajectory_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MultiWaypointPlanner>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  node->setup_move_group();
  node->run_terminal_loop();

  rclcpp::shutdown();
  spinner.join();
  return 0;
}