cmake_minimum_required(VERSION 3.8)
project(my_arm_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Find the required ROS 2 and MoveIt packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)



# 2. Tell CMake to build an executable named "move_arm" from your C++ file
add_executable(move_arm src/move_arm.cpp)
add_executable(trajectory_exporter src/trajectory_exporter.cpp)
add_executable(multi_waypoint_planner src/multi_waypoint_planner.cpp)


# 3. Link the required libraries to your new executable
ament_target_dependencies(move_arm rclcpp moveit_ros_planning_interface geometry_msgs)
ament_target_dependencies(trajectory_exporter rclcpp moveit_msgs)
ament_target_dependencies(multi_waypoint_planner rclcpp moveit_msgs geometry_msgs moveit_ros_planning_interface)

# 4. Install the executable so the ROS 2 system can find it
install(TARGETS
  move_arm
  trajectory_exporter
  multi_waypoint_planner
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()