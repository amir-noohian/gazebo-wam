cmake_minimum_required(VERSION 3.5)
project(wam_motion_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(joint_motion
  src/joint_motion.cpp
)

ament_target_dependencies(
  joint_motion
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
)

install(
  TARGETS joint_motion
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
