#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "moveit/move_group_interface/move_group_interface.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("move_to_pose");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread spinner([&executor]()
  {
    executor.spin();
  });

  moveit::planning_interface::MoveGroupInterface move_group(node, "arm");

  RCLCPP_INFO(
      node->get_logger(),
      "Planning frame: %s",
      move_group.getPlanningFrame().c_str());

  RCLCPP_INFO(
      node->get_logger(),
      "End-effector link: %s",
      move_group.getEndEffectorLink().c_str());

  geometry_msgs::msg::Pose target_pose;

  target_pose.position.x = 0.50;
  target_pose.position.y = 0.00;
  target_pose.position.z = 0.65;

  target_pose.orientation.x = 0.0;
  target_pose.orientation.y = 0.0;
  target_pose.orientation.z = 0.0;
  target_pose.orientation.w = 1.0;

  move_group.setPoseTarget(target_pose);

  move_group.setPlanningTime(10.0);
  move_group.setNumPlanningAttempts(10);

  moveit::planning_interface::MoveGroupInterface::Plan plan;

  const bool planning_success =
      static_cast<bool>(move_group.plan(plan));

  if (!planning_success)
  {
    RCLCPP_ERROR(
        node->get_logger(),
        "MoveIt could not find a valid trajectory to the pose target.");

    move_group.clearPoseTargets();
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(
      node->get_logger(),
      "Planning succeeded. Executing.");

  const bool execution_success =
      static_cast<bool>(move_group.execute(plan));

  if (!execution_success)
  {
    RCLCPP_ERROR(
        node->get_logger(),
        "Trajectory execution failed.");

    move_group.clearPoseTargets();
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(
      node->get_logger(),
      "The end effector reached the target pose.");

  move_group.clearPoseTargets();

  executor.cancel();
  spinner.join();

  rclcpp::shutdown();
  return 0;
}