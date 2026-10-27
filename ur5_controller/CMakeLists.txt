cmake_minimum_required(VERSION 3.22)
project(ur5_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_executable(multi_waypoint_planner src/multi_waypoint_planner.cpp)

target_include_directories(multi_waypoint_planner PUBLIC
  ${moveit_ros_planning_interface_INCLUDE_DIRS}
)

target_link_libraries(multi_waypoint_planner
  rclcpp::rclcpp
  ament_index_cpp::ament_index_cpp
  tf2::tf2
  ${moveit_ros_planning_interface_LIBRARIES}
  ${geometry_msgs_TARGETS}
  ${moveit_msgs_TARGETS}
  ${std_msgs_TARGETS}
)

ament_target_dependencies(multi_waypoint_planner
  moveit_ros_planning_interface
  geometry_msgs
  moveit_msgs
  std_msgs
  tf2
  ament_index_cpp
)

install(TARGETS multi_waypoint_planner
  DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/interactive_control.py
  scripts/planner_cli.py
  scripts/validate_h5.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config waypoints
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(unit_tests test TIMEOUT 120)
endif()

ament_package()
