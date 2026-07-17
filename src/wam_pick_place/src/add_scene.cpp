#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit_msgs/msg/collision_object.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"


int main(int argc, char **argv)
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Create a ROS 2 node for logging
  auto node = rclcpp::Node::make_shared("add_scene");

  // Interface used to modify MoveIt's planning scene
  moveit::planning_interface::PlanningSceneInterface
      planning_scene_interface;

  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

  // =========================================================
  // Table
  // =========================================================

  moveit_msgs::msg::CollisionObject table;

  // Unique object name
  table.id = "table";

  // The table pose is expressed relative to this frame
  table.header.frame_id = "world";

  // Define the table as a rectangular box
  shape_msgs::msg::SolidPrimitive table_primitive;
  table_primitive.type = shape_msgs::msg::SolidPrimitive::BOX;

  // BOX dimensions:
  // dimensions[0] = x length
  // dimensions[1] = y width
  // dimensions[2] = z height
  table_primitive.dimensions.resize(3);
  table_primitive.dimensions[0] = 0.8;
  table_primitive.dimensions[1] = 1.2;
  table_primitive.dimensions[2] = 0.05;

  // Pose of the center of the table
  geometry_msgs::msg::Pose table_pose;
  table_pose.position.x = 0.7;
  table_pose.position.y = 0.0;
  table_pose.position.z = 0.4;
  table_pose.orientation.x = 0.0;
  table_pose.orientation.y = 0.0;
  table_pose.orientation.z = 0.0;
  table_pose.orientation.w = 1.0;

  // Add the table geometry and its corresponding pose
  table.primitives.push_back(table_primitive);
  table.primitive_poses.push_back(table_pose);

  // Tell MoveIt to add or replace this object
  table.operation = moveit_msgs::msg::CollisionObject::ADD;

  collision_objects.push_back(table);

  // =========================================================
  // Cube
  // =========================================================

  moveit_msgs::msg::CollisionObject cube;

  cube.id = "cube";
  cube.header.frame_id = "world";

  shape_msgs::msg::SolidPrimitive cube_primitive;
  cube_primitive.type = shape_msgs::msg::SolidPrimitive::BOX;

  cube_primitive.dimensions.resize(3);
  cube_primitive.dimensions[0] = 0.06;
  cube_primitive.dimensions[1] = 0.06;
  cube_primitive.dimensions[2] = 0.10;

  geometry_msgs::msg::Pose cube_pose;
  cube_pose.position.x = 0.55;
  cube_pose.position.y = 0.0;

  // The table center is at z = 0.4 and its thickness is 0.05 m.
  // Therefore, the table surface is at z = 0.425 m.
  // The cube height is 0.10 m, so its center is 0.05 m above the surface.
  cube_pose.position.z = 0.475;

  cube_pose.orientation.x = 0.0;
  cube_pose.orientation.y = 0.0;
  cube_pose.orientation.z = 0.0;
  cube_pose.orientation.w = 1.0;

  cube.primitives.push_back(cube_primitive);
  cube.primitive_poses.push_back(cube_pose);

  cube.operation = moveit_msgs::msg::CollisionObject::ADD;

  collision_objects.push_back(cube);

  // =========================================================
  // Apply objects to the planning scene
  // =========================================================

  bool success =
      planning_scene_interface.applyCollisionObjects(collision_objects);

  if (!success)
  {
    RCLCPP_ERROR(
        node->get_logger(),
        "Failed to apply collision objects.");

    rclcpp::shutdown();
    return 1;
  }

  const auto object_names =
      planning_scene_interface.getKnownObjectNames();

  RCLCPP_INFO(
      node->get_logger(),
      "MoveIt currently knows %zu collision objects:",
      object_names.size());

  for (const auto &name : object_names)
  {
    RCLCPP_INFO(node->get_logger(), "  - %s", name.c_str());
  }

  rclcpp::shutdown();
  return 0;
}
