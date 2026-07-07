cmake_minimum_required(VERSION 3.22)
project(ur5_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Find the required ROS 2 and MoveIt packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)


# 2. Tell CMake to build an executable named "multi_waypoint_planner" from the C++ source
add_executable(multi_waypoint_planner src/multi_waypoint_planner.cpp)

# 3. Include directories for the target (modern CMake style)
target_include_directories(multi_waypoint_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${moveit_ros_planning_interface_INCLUDE_DIRS}
)

# 4. Modern CMake: target_link_libraries replaces the discouraged ament_target_dependencies macro.
#    Using direct imported targets and package-provided variables ensures correct
#    transitive dependency propagation under Jazzy's CMake 3.22+ toolchain.
target_link_libraries(multi_waypoint_planner
  rclcpp::rclcpp
  ${moveit_ros_planning_interface_LIBRARIES}
  ${geometry_msgs_TARGETS}
  ${moveit_msgs_TARGETS}
)

# Required to let ament resolve ament_index and environment hooks for the above packages
ament_target_dependencies(multi_waypoint_planner
  moveit_ros_planning_interface
  geometry_msgs
  moveit_msgs
)

# 5. Install the executable so the ROS 2 system can find it
install(TARGETS
  multi_waypoint_planner
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()