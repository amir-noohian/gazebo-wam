cmake_minimum_required(VERSION 3.5)
project(wam_cartesian_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(urdf REQUIRED)

add_library(
  wam_cartesian_controller
  SHARED
  src/wam_cartesian_controller.cpp
)

target_include_directories(
  wam_cartesian_controller
  PUBLIC
    include
)

ament_target_dependencies(
  wam_cartesian_controller
  controller_interface
  geometry_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  orocos_kdl
  kdl_parser
  urdf
)

pluginlib_export_plugin_description_file(
  controller_interface
  plugin.xml
)

install(
  TARGETS wam_cartesian_controller
  EXPORT export_wam_cartesian_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)

ament_export_dependencies(
  controller_interface
  geometry_msgs
  hardware_interface
  kdl_parser
  orocos_kdl
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
