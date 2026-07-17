cmake_minimum_required(VERSION 3.5)
project(wam_motion_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2 REQUIRED)

add_executable(joint_motion
  src/joint_motion.cpp
)

add_executable(cartesian_motion
  src/cartesian_motion.cpp
)

ament_target_dependencies(
  cartesian_motion
  control_msgs
  geometry_msgs
  rclcpp
  rclcpp_action
  moveit_core
  moveit_ros_planning_interface
  moveit_msgs
  tf2
)

ament_target_dependencies(
  joint_motion
  control_msgs
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  moveit_msgs
)

install(
  TARGETS cartesian_motion joint_motion
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
