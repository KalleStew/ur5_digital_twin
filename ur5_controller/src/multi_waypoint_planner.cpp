#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <std_msgs/msg/string.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <thread>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cmath>
#include <cstdlib>
#include <mutex>
#include <algorithm>
#include <cctype>
#include <regex>
#include <unistd.h>
#include <filesystem>
#include <map>

namespace {

// Same resolution as ur5_controller.trajectory_io.data_dir(): never install/ or the CWD.
std::filesystem::path data_dir() {
  if (const char* env = std::getenv("UR5_DATA_DIR"); env && *env) {
    return std::filesystem::absolute(env);
  }
  try {
    const std::filesystem::path install = std::filesystem::path(
      ament_index_cpp::get_package_prefix("ur5_controller")).parent_path();
    if (install.filename() == "install") {
      return install.parent_path() / "ur5_data";
    }
  } catch (const std::exception&) {
  }
  const char* home = std::getenv("HOME");
  return std::filesystem::path(home ? home : ".") / "ur5_data";
}

bool valid_name(const std::string& name) {
  static const std::regex re("^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$");
  return std::regex_match(name, re) && name.find("..") == std::string::npos;
}

std::vector<std::string> split_words(const std::string& s) {
  std::istringstream in(s);
  std::vector<std::string> out;
  std::string w;
  while (in >> w) {
    out.push_back(w);
  }
  return out;
}

std::string trim_copy(const std::string& input) {
  const auto first = input.find_first_not_of(" \t\n\r\f\v");
  if (first == std::string::npos) {
    return "";
  }

  const auto last = input.find_last_not_of(" \t\n\r\f\v");
  return input.substr(first, last - first + 1);
}

std::string normalize_command(std::string input) {
  input = trim_copy(input);
  std::transform(input.begin(), input.end(), input.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  return input;
}

int levenshtein_distance(const std::string& lhs, const std::string& rhs) {
  const size_t lhs_size = lhs.size();
  const size_t rhs_size = rhs.size();

  std::vector<int> previous(rhs_size + 1);
  std::vector<int> current(rhs_size + 1);

  for (size_t j = 0; j <= rhs_size; ++j) {
    previous[j] = static_cast<int>(j);
  }

  for (size_t i = 0; i < lhs_size; ++i) {
    current[0] = static_cast<int>(i + 1);
    for (size_t j = 0; j < rhs_size; ++j) {
      const int substitution_cost = lhs[i] == rhs[j] ? 0 : 1;
      current[j + 1] = std::min({
        previous[j + 1] + 1,
        current[j] + 1,
        previous[j] + substitution_cost,
      });
    }
    previous.swap(current);
  }

  return previous[rhs_size];
}

std::string resolve_command(const std::string& raw_command) {
  const std::string command = normalize_command(raw_command);

  if (command.empty()) {
    return "";
  }

  static const std::vector<std::string> canonical_commands = {
    "plan",
    "execute",
    "return home",
    "export joints",
    "export cartesian",
    "clear",
  };

  if (std::find(canonical_commands.begin(), canonical_commands.end(), command) != canonical_commands.end()) {
    return command;
  }

  if (command == "return" || command == "home" || command == "rh") {
    return "return home";
  }

  if (command == "ej") {
    return "export joints";
  }
  if (command == "ec") {
    return "export cartesian";
  }
  if (command == "export" || command == "quit") {
    return command;
  }

  if (levenshtein_distance(command, "export") <= 2) {
    return "export";
  }

  std::string best_match;
  int best_distance = 3;
  for (const auto& candidate : canonical_commands) {
    const int distance = levenshtein_distance(command, candidate);
    if (distance < best_distance) {
      best_distance = distance;
      best_match = candidate;
    }
  }

  if (!best_match.empty()) {
    return best_match;
  }

  return command;
}

}  // namespace

class MultiWaypointPlanner : public rclcpp::Node {
public:
  MultiWaypointPlanner() : Node("multi_waypoint_planner", rclcpp::NodeOptions().append_parameter_override("use_sim_time", true)) {
    waypoints_dir_ = data_dir() / "waypoints";
    min_export_dt_ = this->declare_parameter<double>("min_export_dt", 0.5);
    status_pub_ = this->create_publisher<std_msgs::msg::String>("/planner_status", 10);

    click_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/clicked_point", 10,
      std::bind(&MultiWaypointPlanner::point_callback, this, std::placeholders::_1)
    );

    // Separate group so blocking commands don't starve the default group (joint_states for getCurrentState).
    command_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions command_sub_options;
    command_sub_options.callback_group = command_cb_group_;
    command_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/planner_command", 10,
      std::bind(&MultiWaypointPlanner::command_callback, this, std::placeholders::_1),
      command_sub_options
    );
    
    display_pub_ = this->create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path", 10);
    
    RCLCPP_INFO(this->get_logger(), "==========================================");
    RCLCPP_INFO(this->get_logger(), " Multi-Waypoint Planner & Exporter Ready! ");
    RCLCPP_INFO(this->get_logger(), " Waypoints dir: %s", waypoints_dir_.c_str());
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

  // Returns when stdin closes or on 'quit'; the node keeps serving /planner_command until Ctrl-C.
  bool run_terminal_loop() {
    std::string command;
    while (rclcpp::ok()) {
      std::cout << "\n[MENU] Enter a command:\n"
                << "  'point x y z [r p y]'    -> Add a task-space waypoint (world frame, m / rad)\n"
                << "  'plan'                   -> Cartesian path through the waypoints, preview in RViz\n"
                << "  'execute'                -> Move the simulated robot\n"
                << "  'return home'            -> Add the starting position to the end of the path\n"
                << "  'export joints <name>'   -> Save the planned path as $UR5_DATA_DIR/waypoints/<name>.csv\n"
                << "  'export cartesian <name>'-> Save the waypoints as a Cartesian CSV\n"
                << "  'clear'                  -> Reset waypoints to the current pose\n"
                << "  'quit'                   -> Exit\n"
                << "Command: ";
      if (!std::getline(std::cin, command)) {
        RCLCPP_INFO(this->get_logger(), "stdin closed; still serving /planner_command (Ctrl-C to stop).");
        return false;
      }
      if (resolve_command(command) == "quit") {
        return true;
      }
      handle_command(command, "terminal");
    }
    return true;
  }

private:
  void command_callback(const std_msgs::msg::String::SharedPtr msg) {
    handle_command(msg->data, "topic");
  }

  void report(bool ok, const std::string& command, const std::string& detail) {
    if (ok) {
      RCLCPP_INFO(this->get_logger(), "%s: %s", command.c_str(), detail.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "%s: %s", command.c_str(), detail.c_str());
    }
    std_msgs::msg::String msg;
    msg.data = (ok ? "OK " : "ERROR ") + command + ": " + detail;
    status_pub_->publish(msg);
  }

  void handle_command(const std::string& raw_command, const std::string& source) {
    std::lock_guard<std::mutex> command_lock(command_mutex_);

    const auto words = split_words(normalize_command(raw_command));
    if (words.empty()) {
      return;
    }
    if (words[0] == "point") {
      add_point_command(words);
      return;
    }
    std::string resolved_command;
    std::string name;
    bool force = false;
    if (words[0] == "export" && words.size() >= 3 && (words[1] == "joints" || words[1] == "cartesian")) {
      resolved_command = "export " + words[1];
      name = split_words(trim_copy(raw_command))[2];
      force = words.size() >= 4 && words[3] == "force";
    } else {
      resolved_command = resolve_command(raw_command);
    }

    if (resolved_command == "plan") {
      std::vector<geometry_msgs::msg::Pose> waypoints_snapshot;
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        waypoints_snapshot = waypoints_;
      }
      if (waypoints_snapshot.size() < 2) {
        report(false, "plan", "need at least one waypoint after the start pose (point x y z, or RViz Publish Point)");
        return;
      }
      if (!move_group_) {
        report(false, "plan", "move group is not initialized");
        return;
      }

      const double eef_step = 0.01;
      moveit_msgs::msg::RobotTrajectory local_trajectory;
      const double fraction = move_group_->computeCartesianPath(waypoints_snapshot, eef_step, local_trajectory);
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        calculated_trajectory_ = local_trajectory;
        plan_fraction_ = fraction;
      }

      auto current_state = move_group_->getCurrentState();
      if (current_state) {
        moveit_msgs::msg::DisplayTrajectory display_msg;
        moveit::core::robotStateToRobotStateMsg(*current_state, display_msg.trajectory_start);
        display_msg.trajectory.push_back(local_trajectory);
        display_pub_->publish(display_msg);
      }
      if (fraction < 1.0) {
        report(false, "plan", "only " + std::to_string(fraction * 100.0).substr(0, 5) +
               "% of the Cartesian path is feasible (collision, joint limit or IK); move or remove waypoints");
      } else {
        report(true, "plan", std::to_string(local_trajectory.joint_trajectory.points.size()) + " points, 100% feasible");
      }
      return;
    }

    if (resolved_command == "execute") {
      if (!move_group_) {
        report(false, "execute", "move group is not initialized");
        return;
      }
      moveit_msgs::msg::RobotTrajectory trajectory_to_execute;
      double fraction = 0.0;
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        trajectory_to_execute = calculated_trajectory_;
        fraction = plan_fraction_;
      }
      if (trajectory_to_execute.joint_trajectory.points.empty() || fraction < 1.0) {
        report(false, "execute", "no fully feasible plan; run 'plan' first");
        return;
      }
      const auto code = move_group_->execute(trajectory_to_execute);
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        waypoints_.clear();
        waypoints_.push_back(move_group_->getCurrentPose().pose);
        calculated_trajectory_ = moveit_msgs::msg::RobotTrajectory();
        plan_fraction_ = 0.0;
      }
      report(code == moveit::core::MoveItErrorCode::SUCCESS, "execute",
             code == moveit::core::MoveItErrorCode::SUCCESS ? "done" : "execution failed");
      return;
    }

    if (resolved_command == "return home") {
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        waypoints_.push_back(initial_pose_);
      }
      report(true, "return home", "appended the starting pose");
      return;
    }

    if (resolved_command == "export joints" || resolved_command == "export cartesian") {
      if (name.empty()) {
        report(false, resolved_command, "needs a name: '" + resolved_command + " <name> [force]'");
        return;
      }
      if (!valid_name(name)) {
        report(false, resolved_command, "invalid name '" + name + "': letters, digits, _ . - only");
        return;
      }
      if (resolved_command == "export joints") {
        export_joints_csv(name, force);
      } else {
        export_cartesian_csv(name, force);
      }
      return;
    }

    if (resolved_command == "clear") {
      if (!move_group_) {
        report(false, "clear", "move group is not initialized");
        return;
      }
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        waypoints_.clear();
        waypoints_.push_back(move_group_->getCurrentPose().pose);
        calculated_trajectory_ = moveit_msgs::msg::RobotTrajectory();
        plan_fraction_ = 0.0;
      }
      report(true, "clear", "waypoints reset to the current pose");
      return;
    }

    report(false, "command", "unknown " + source + " command '" + raw_command +
           "'; try point, plan, execute, return home, export joints <name>, export cartesian <name>, clear");
  }

  void add_point_command(const std::vector<std::string>& words) {
    if (words.size() != 4 && words.size() != 7) {
      report(false, "point", "usage: point x y z [roll pitch yaw]");
      return;
    }
    std::vector<double> v;
    try {
      for (size_t i = 1; i < words.size(); ++i) {
        size_t used = 0;
        v.push_back(std::stod(words[i], &used));
        if (used != words[i].size() || !std::isfinite(v.back())) {
          throw std::invalid_argument(words[i]);
        }
      }
    } catch (const std::exception&) {
      report(false, "point", "non-numeric value in '" + trim_copy(words.size() > 1 ? words[1] : "") + "...'");
      return;
    }
    size_t n = 0;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      geometry_msgs::msg::Pose pose = waypoints_.empty() ? initial_pose_ : waypoints_.back();
      pose.position.x = v[0];
      pose.position.y = v[1];
      pose.position.z = v[2];
      if (v.size() == 6) {
        tf2::Quaternion q;
        q.setRPY(v[3], v[4], v[5]);
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();
      }
      waypoints_.push_back(pose);
      n = waypoints_.size() - 1;
    }
    report(true, "point", "waypoint " + std::to_string(n) + " added");
  }

  void point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);

    if (waypoints_.empty()) {
      geometry_msgs::msg::Pose seed_pose;
      seed_pose.orientation.w = 1.0;
      waypoints_.push_back(seed_pose);
    }

    geometry_msgs::msg::Pose new_waypoint = waypoints_.back();
    new_waypoint.position.x = msg->point.x;
    new_waypoint.position.y = msg->point.y;
    new_waypoint.position.z = msg->point.z;

    waypoints_.push_back(new_waypoint);
    RCLCPP_INFO(this->get_logger(), "Added Target %zu -> [X: %.2f, Y: %.2f, Z: %.2f]",
                waypoints_.size() - 1, msg->point.x, msg->point.y, msg->point.z);
  }

  bool open_export(const std::string& command, const std::string& name, bool force,
                   std::filesystem::path& out_path, std::ofstream& file) {
    std::error_code ec;
    std::filesystem::create_directories(waypoints_dir_, ec);
    out_path = waypoints_dir_ / (name + ".csv");
    if (std::filesystem::exists(out_path) && !force) {
      report(false, command, out_path.string() + " exists; pick another name or append 'force'");
      return false;
    }
    file.open(out_path.string() + ".part");
    if (!file.is_open()) {
      report(false, command, "cannot write " + out_path.string() + "; set UR5_DATA_DIR to a writable directory");
      return false;
    }
    return true;
  }

  // Joint CSV in controller joint order. Rows are rest-to-rest waypoints for Brain A, so the
  // dense planner output is thinned to one row per min_export_dt seconds plus the final point.
  void export_joints_csv(const std::string& name, bool force) {
    moveit_msgs::msg::RobotTrajectory traj_snapshot;
    double fraction = 0.0;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      traj_snapshot = calculated_trajectory_;
      fraction = plan_fraction_;
    }
    if (traj_snapshot.joint_trajectory.points.empty()) {
      report(false, "export joints", "no plan; run 'plan' first");
      return;
    }
    if (fraction < 1.0) {
      report(false, "export joints", "refused: the last plan covered only part of the path");
      return;
    }

    static const std::vector<std::string> kCanonical = {
      "shoulder_pan_joint1", "shoulder_pan_joint", "shoulder_lift_joint",
      "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
    };
    const auto& joint_traj = traj_snapshot.joint_trajectory;
    std::vector<std::pair<double, std::vector<double>>> rows;
    double last_t = 0.0;
    for (size_t k = 0; k < joint_traj.points.size(); ++k) {
      const auto& point = joint_traj.points[k];
      const double t = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9;
      const bool final_point = k + 1 == joint_traj.points.size();
      if (t <= 0.0 || (!final_point && t - last_t < min_export_dt_)) {
        continue;
      }
      std::map<std::string, double> by_name;
      for (size_t i = 0; i < joint_traj.joint_names.size() && i < point.positions.size(); ++i) {
        by_name[joint_traj.joint_names[i]] = point.positions[i];
      }
      std::vector<double> q;
      for (const auto& jn : kCanonical) {
        const auto it = by_name.find(jn);
        if (it == by_name.end()) {
          report(false, "export joints", "planned trajectory lacks joint " + jn);
          return;
        }
        q.push_back(it->second);
      }
      if (final_point && !rows.empty() && t - rows.back().first < min_export_dt_) {
        rows.back() = {std::max(t, rows.back().first), q};
      } else {
        rows.emplace_back(t, q);
      }
      last_t = t;
    }
    if (rows.empty()) {
      report(false, "export joints", "planned trajectory has no timed points");
      return;
    }

    std::filesystem::path out_path;
    std::ofstream file;
    if (!open_export("export joints", name, force, out_path, file)) {
      return;
    }
    file << "time_sec";
    for (const auto& jn : kCanonical) {
      file << "," << jn;
    }
    file << "\n";
    file.precision(6);
    for (const auto& [t, q] : rows) {
      file << std::fixed << t;
      for (double v : q) {
        file << "," << v;
      }
      file << "\n";
    }
    file.close();
    std::filesystem::rename(out_path.string() + ".part", out_path);
    report(true, "export joints", "wrote " + std::to_string(rows.size()) + " rows to " + out_path.string());
  }

  void export_cartesian_csv(const std::string& name, bool force) {
    std::vector<geometry_msgs::msg::Pose> waypoints_snapshot;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      waypoints_snapshot = waypoints_;
    }
    if (waypoints_snapshot.size() < 2) {
      report(false, "export cartesian", "need at least one waypoint after the start pose");
      return;
    }

    std::filesystem::path out_path;
    std::ofstream file;
    if (!open_export("export cartesian", name, force, out_path, file)) {
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
    std::filesystem::rename(out_path.string() + ".part", out_path);
    report(true, "export cartesian", "wrote " + std::to_string(waypoints_snapshot.size()) +
           " rows to " + out_path.string());
  }

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr click_sub_;
  rclcpp::CallbackGroup::SharedPtr command_cb_group_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::mutex data_mutex_;
  std::mutex command_mutex_;
  std::filesystem::path waypoints_dir_;
  double min_export_dt_ = 0.5;
  double plan_fraction_ = 0.0;

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
  RCLCPP_INFO(node->get_logger(), "Planner ready; commands on /planner_command, replies on /planner_status.");
  if (isatty(fileno(stdin)) && node->run_terminal_loop()) {
    rclcpp::shutdown();
  }
  spinner.join();
  return 0;
}