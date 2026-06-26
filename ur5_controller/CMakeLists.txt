cmake_minimum_required(VERSION 3.8)
project(ur5_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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
# add_executable(move_arm src/move_arm.cpp)
add_executable(multi_waypoint_planner src/multi_waypoint_planner.cpp)


# 3. Link the required libraries to your new executable using modern CMake targets
# ament_target_dependencies is deprecated — use target_link_libraries with explicit targets
target_link_libraries(multi_waypoint_planner
  rclcpp::rclcpp
  ${moveit_ros_planning_interface_TARGETS}
  ${geometry_msgs_TARGETS}
  ${moveit_msgs_TARGETS}
)

# 4. Install the executable so the ROS 2 system can find it
install(TARGETS
  multi_waypoint_planner
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()