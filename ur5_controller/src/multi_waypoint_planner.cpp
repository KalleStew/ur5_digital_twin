#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <thread>
#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <mutex>
#include <algorithm>

class MultiWaypointPlanner : public rclcpp::Node {
public:
  // ...
    MultiWaypointPlanner() : Node("multi_waypoint_planner", rclcpp::NodeOptions().append_parameter_override("use_sim_time", true)) {
    click_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/clicked_point", 10,
      std::bind(&MultiWaypointPlanner::point_callback, this, std::placeholders::_1)
    );
    
    display_pub_ = this->create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path", 10);
    
    RCLCPP_INFO(this->get_logger(), "==========================================");
    RCLCPP_INFO(this->get_logger(), " Multi-Waypoint Planner & Exporter Ready! ");
    RCLCPP_INFO(this->get_logger(), "==========================================");
  }

  bool setup_move_group() {
    try {
      move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "ur_manipulator");
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize MoveGroupInterface: %s", e.what());
      return false;
    }

    if (!move_group_) {
      RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface is null after initialization.");
      return false;
    }

    // Save the starting pose so we can return to it later.
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      initial_pose_ = move_group_->getCurrentPose().pose;
      waypoints_.clear();
      waypoints_.push_back(initial_pose_);
    }

    return true;
  }

  void run_terminal_loop() {
    std::string command;
    while (rclcpp::ok()) {
      std::cout << "\n[MENU] Enter a command:\n"
                << "  'plan'                   -> Preview path in RViz\n"
                << "  'execute'                -> Move the simulated robot\n"
                << "  'return home'            -> Add the starting position to the end of the path\n"
                << "  'export joints'          -> Save interpolated trajectory in Joint Space CSV (rad)\n"
                << "  'export cartesian'       -> Save clicked waypoints in Cartesian CSV (rad)\n"
                << "  'clear'                  -> Reset waypoints\n"
                << "Command: ";
                
      std::getline(std::cin, command);

      if (command == "plan") {
        std::vector<geometry_msgs::msg::Pose> waypoints_snapshot;
        {
          std::lock_guard<std::mutex> lock(data_mutex_);
          if (waypoints_.size() < 2) {
            RCLCPP_WARN(this->get_logger(), "Need more waypoints. Use 'Publish Point' in RViz.");
            continue;
          }
          waypoints_snapshot = waypoints_;
        }

        if (!move_group_) {
          RCLCPP_ERROR(this->get_logger(), "Move group is not initialized.");
          continue;
        }

        if (waypoints_snapshot.size() < 2) {
          RCLCPP_WARN(this->get_logger(), "Need more waypoints. Use 'Publish Point' in RViz.");
          continue;
        }

        const double eef_step = 0.01; // 1 cm resolution
        
        moveit_msgs::msg::RobotTrajectory local_trajectory;
        double fraction = move_group_->computeCartesianPath(waypoints_snapshot, eef_step, local_trajectory);

        {
          std::lock_guard<std::mutex> lock(data_mutex_);
          calculated_trajectory_ = local_trajectory;
        }

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
        if (!move_group_) {
          RCLCPP_ERROR(this->get_logger(), "Move group is not initialized.");
          continue;
        }

        moveit_msgs::msg::RobotTrajectory trajectory_to_execute;
        {
          std::lock_guard<std::mutex> lock(data_mutex_);
          trajectory_to_execute = calculated_trajectory_;
        }

        if (trajectory_to_execute.joint_trajectory.points.empty()) {
          RCLCPP_WARN(this->get_logger(), "You must 'plan' successfully before executing.");
          continue;
        }
        RCLCPP_INFO(this->get_logger(), "Executing trajectory...");
        move_group_->execute(trajectory_to_execute);
        
        // Reset waypoints starting from new location
        {
          std::lock_guard<std::mutex> lock(data_mutex_);
          waypoints_.clear();
          waypoints_.push_back(move_group_->getCurrentPose().pose);
          calculated_trajectory_ = moveit_msgs::msg::RobotTrajectory();
        }

      } else if (command == "return home") {
        // Feature: Finish in starting position
        {
          std::lock_guard<std::mutex> lock(data_mutex_);
          waypoints_.push_back(initial_pose_);
        }
        RCLCPP_INFO(this->get_logger(), "Appended starting position to the path.");

      } else if (command == "export joints") {
        export_joints_csv();

      } else if (command == "export cartesian") {
        export_cartesian_csv();

      } else if (command == "clear") {
        if (!move_group_) {
          RCLCPP_ERROR(this->get_logger(), "Move group is not initialized.");
          continue;
        }

        {
          std::lock_guard<std::mutex> lock(data_mutex_);
          waypoints_.clear();
          waypoints_.push_back(move_group_->getCurrentPose().pose);
          calculated_trajectory_ = moveit_msgs::msg::RobotTrajectory();
        }
        RCLCPP_INFO(this->get_logger(), "Waypoints cleared.");
      }
    }
  }

private:
  void point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);

    if (waypoints_.empty()) {
      geometry_msgs::msg::Pose seed_pose;
      seed_pose.orientation.w = 1.0;
      waypoints_.push_back(seed_pose);
    }

    geometry_msgs::msg::Pose new_waypoint = waypoints_.back(); // Keep previous orientation
    new_waypoint.position.x = msg->point.x;
    new_waypoint.position.y = msg->point.y;
    new_waypoint.position.z = msg->point.z;

    waypoints_.push_back(new_waypoint);
    RCLCPP_INFO(this->get_logger(), "Added Target %zu -> [X: %.2f, Y: %.2f, Z: %.2f]",
                waypoints_.size() - 1, msg->point.x, msg->point.y, msg->point.z);
  }

  // EXPORT 1: Joint-Space CSV for interactive controller ingestion.
  void export_joints_csv() {
    moveit_msgs::msg::RobotTrajectory traj_snapshot;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      traj_snapshot = calculated_trajectory_;
    }

    if (traj_snapshot.joint_trajectory.points.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No trajectory calculated! Type 'plan' first.");
      return;
    }

    std::ofstream file("trajectory_joints_export.csv");
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open trajectory_joints_export.csv for writing.");
      return;
    }

    auto joint_traj = traj_snapshot.joint_trajectory;
    const size_t joint_count = std::min<size_t>(7, joint_traj.joint_names.size());

    file << "time_sec";
    for (size_t i = 0; i < joint_count; ++i) {
      file << "," << joint_traj.joint_names[i];
    }
    for (size_t i = joint_count; i < 7; ++i) {
      file << ",joint_" << i;
    }
    file << "\n";

    for (const auto& point : joint_traj.points) {
      double time_sec = point.time_from_start.sec + (point.time_from_start.nanosec * 1e-9);
      file << time_sec;

      for (size_t i = 0; i < joint_count && i < point.positions.size(); ++i) {
        file << "," << point.positions[i];
      }
      for (size_t i = std::min(joint_count, point.positions.size()); i < 7; ++i) {
        file << ",0.0";
      }
      file << "\n";
    }
    file.close();
    RCLCPP_INFO(this->get_logger(), "Saved %zu joint-space waypoints to trajectory_joints_export.csv.", joint_traj.points.size());
  }

  // EXPORT 2: Cartesian CSV for interactive controller ingestion.
  void export_cartesian_csv() {
    std::vector<geometry_msgs::msg::Pose> waypoints_snapshot;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      waypoints_snapshot = waypoints_;
    }

    if (waypoints_snapshot.size() < 2) return;

    std::ofstream file("trajectory_cartesian_export.csv");
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open trajectory_cartesian_export.csv for writing.");
      return;
    }

    file << "time_sec,x,y,z,roll,pitch,yaw\n";
    constexpr double time_step_sec = 4.0;
    for (size_t i = 0; i < waypoints_snapshot.size(); ++i) {
      const auto& pos = waypoints_snapshot[i].position;
      const auto& q = waypoints_snapshot[i].orientation;

      double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
      double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
      double roll = std::atan2(sinr_cosp, cosr_cosp);

      double sinp = 2 * (q.w * q.y - q.z * q.x);
      sinp = std::clamp(sinp, -1.0, 1.0);
      double pitch = std::asin(sinp);

      double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
      double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
      double yaw = std::atan2(siny_cosp, cosy_cosp);

      const double time_sec = static_cast<double>(i + 1) * time_step_sec;
      file << time_sec << "," << pos.x << "," << pos.y << "," << pos.z << ","
           << roll << "," << pitch << "," << yaw << "\n";
    }
    file.close();
    RCLCPP_INFO(this->get_logger(), "Saved %zu cartesian waypoints to trajectory_cartesian_export.csv.", waypoints_snapshot.size());
  }

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr click_sub_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_pub_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::mutex data_mutex_;
  
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

  if (!node->setup_move_group()) {
    RCLCPP_FATAL(node->get_logger(), "Planner startup aborted: Move group setup failed.");
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }
  node->run_terminal_loop();

  rclcpp::shutdown();
  spinner.join();
  return 0;
}