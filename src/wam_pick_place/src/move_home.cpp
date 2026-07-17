#include <memory>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("move_home");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread spinner([&executor]()
  {
    executor.spin();
  });

  moveit::planning_interface::MoveGroupInterface move_group(node, "arm");

  std::vector<double> home_joint_values = {
    0.0,   // base_yaw
    0.0,   // shoulder_pitch
    0.0,   // shoulder_yaw
    1.0,   // elbow_pitch
    0.0,   // wrist_yaw
    0.0,   // wrist_pitch
    0.0    // palm_yaw
  };

  bool target_set =
      move_group.setJointValueTarget(home_joint_values);

  if (!target_set)
  {
    RCLCPP_ERROR(
        node->get_logger(),
        "The requested joint target is invalid.");

    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;

  bool planning_success =
      static_cast<bool>(move_group.plan(plan));

  if (!planning_success)
  {
    RCLCPP_ERROR(
        node->get_logger(),
        "MoveIt failed to find a valid plan.");

    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(
      node->get_logger(),
      "Planning succeeded. Executing trajectory.");

  bool execution_success =
      static_cast<bool>(move_group.execute(plan));

  if (!execution_success)
  {
    RCLCPP_ERROR(
        node->get_logger(),
        "Trajectory execution failed.");

    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(
      node->get_logger(),
      "The WAM reached the home configuration.");

  executor.cancel();
  spinner.join();

  rclcpp::shutdown();
  return 0;
}