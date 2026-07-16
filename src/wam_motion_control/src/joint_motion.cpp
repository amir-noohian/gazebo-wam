#include <memory>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  const auto node_options =
      rclcpp::NodeOptions()
          .automatically_declare_parameters_from_overrides(true);

  auto node = rclcpp::Node::make_shared(
      "wam_joint_motion",
      node_options);

  /*
   * MoveGroupInterface communicates with move_group through ROS topics,
   * services, and actions. An executor must therefore process callbacks
   * while the main thread performs planning and execution.
   */
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread executor_thread([&executor]() {
    executor.spin();
  });

  /*
   * This name must match the planning-group name in wam.srdf.
   * We will verify it before running.
   */
  const std::string planning_group = "arm";

  moveit::planning_interface::MoveGroupInterface move_group(
      node,
      planning_group);

  RCLCPP_INFO(
      node->get_logger(),
      "Planning frame: %s",
      move_group.getPlanningFrame().c_str());

  RCLCPP_INFO(
      node->get_logger(),
      "End-effector link: %s",
      move_group.getEndEffectorLink().c_str());

  RCLCPP_INFO(
      node->get_logger(),
      "Planning group: %s",
      planning_group.c_str());

  /*
   * Limit the trajectory speed during the first test.
   * 0.2 means 20 percent of the configured maximum.
   */
  move_group.setMaxVelocityScalingFactor(0.2);
  move_group.setMaxAccelerationScalingFactor(0.2);
  move_group.setPlanningTime(5.0);

  /*
   * Seven target joint positions for the 7-DOF WAM.
   * The order must match the active-joint order of the MoveIt group.
   */
  const std::vector<double> joint_target = {
      0.0,
      -0.4,
      0.2,
      1.2,
      0.0,
      0.4,
      0.0
  };

  const bool target_accepted =
      move_group.setJointValueTarget(joint_target);

  if (!target_accepted)
  {
    RCLCPP_ERROR(
        node->get_logger(),
        "The requested joint target violates one or more joint limits.");

    executor.cancel();
    executor_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;

  RCLCPP_INFO(node->get_logger(), "Planning joint-space motion...");

  const auto planning_result = move_group.plan(plan);

  if (planning_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Motion planning failed.");

    executor.cancel();
    executor_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Planning succeeded.");
  RCLCPP_INFO(node->get_logger(), "Executing trajectory...");

  const auto execution_result = move_group.execute(plan);

  if (execution_result.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_INFO(node->get_logger(), "Trajectory execution succeeded.");
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Trajectory execution failed.");
  }

  executor.cancel();
  executor_thread.join();

  rclcpp::shutdown();
  return 0;
}
