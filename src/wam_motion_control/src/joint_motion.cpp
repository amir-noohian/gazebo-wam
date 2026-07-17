#include <chrono>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

namespace
{

template<typename ParameterT>
ParameterT declare_if_missing(
  const rclcpp::Node::SharedPtr & node,
  const std::string & name,
  const ParameterT & default_value)
{
  if (!node->has_parameter(name)) {
    return node->declare_parameter<ParameterT>(name, default_value);
  }

  return node->get_parameter(name).get_value<ParameterT>();
}

bool valid_scaling_factor(double value)
{
  return value > 0.0 && value <= 1.0;
}

int run_motion(const rclcpp::Node::SharedPtr & node)
{
  const std::string planning_group =
    declare_if_missing<std::string>(node, "planning_group", "arm");
  const std::string controller_action = declare_if_missing<std::string>(
    node,
    "controller_action",
    "/arm_controller/follow_joint_trajectory");
  const double velocity_scaling =
    declare_if_missing<double>(node, "velocity_scaling", 0.2);
  const double acceleration_scaling =
    declare_if_missing<double>(node, "acceleration_scaling", 0.2);
  const double planning_time =
    declare_if_missing<double>(node, "planning_time", 5.0);
  const double server_timeout =
    declare_if_missing<double>(node, "server_timeout", 10.0);
  const double state_timeout =
    declare_if_missing<double>(node, "state_timeout", 10.0);
  const double controller_timeout =
    declare_if_missing<double>(node, "controller_timeout", 10.0);
  const bool plan_only =
    declare_if_missing<bool>(node, "plan_only", false);

  const std::map<std::string, double> default_targets = {
    {"wam/base_yaw_joint", 0.0},
    {"wam/shoulder_pitch_joint", -0.4},
    {"wam/shoulder_yaw_joint", 0.2},
    {"wam/elbow_pitch_joint", 1.2},
    {"wam/wrist_yaw_joint", 0.0},
    {"wam/wrist_pitch_joint", 0.4},
    {"wam/palm_yaw_joint", 0.0},
  };

  std::map<std::string, double> joint_targets;
  for (const auto & target : default_targets) {
    joint_targets.emplace(
      target.first,
      declare_if_missing<double>(
        node,
        "joint_targets." + target.first,
        target.second));
  }

  if (!valid_scaling_factor(velocity_scaling) ||
    !valid_scaling_factor(acceleration_scaling))
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Velocity and acceleration scaling factors must be in (0, 1].");
    return 1;
  }

  if (planning_time <= 0.0 || server_timeout <= 0.0 ||
    state_timeout <= 0.0 || controller_timeout <= 0.0)
  {
    RCLCPP_ERROR(node->get_logger(), "All timeout values must be positive.");
    return 1;
  }

  moveit::planning_interface::MoveGroupInterface move_group(
    node,
    planning_group,
    std::shared_ptr<tf2_ros::Buffer>(),
    rclcpp::Duration::from_seconds(server_timeout));

  RCLCPP_INFO(node->get_logger(), "Planning frame: %s", move_group.getPlanningFrame().c_str());
  RCLCPP_INFO(node->get_logger(), "End-effector link: %s", move_group.getEndEffectorLink().c_str());
  RCLCPP_INFO(node->get_logger(), "Planning group: %s", planning_group.c_str());

  const std::vector<std::string> group_joints = move_group.getActiveJoints();
  const std::set<std::string> expected_joints(group_joints.begin(), group_joints.end());

  for (const auto & target : joint_targets) {
    if (expected_joints.count(target.first) == 0) {
      RCLCPP_ERROR(
        node->get_logger(),
        "Joint target '%s' is not part of planning group '%s'.",
        target.first.c_str(),
        planning_group.c_str());
      return 1;
    }
  }

  for (const auto & joint : expected_joints) {
    if (joint_targets.count(joint) == 0) {
      RCLCPP_ERROR(
        node->get_logger(),
        "Planning-group joint '%s' has no configured target.",
        joint.c_str());
      return 1;
    }
  }

  if (!move_group.startStateMonitor(state_timeout) ||
    !move_group.getCurrentState(state_timeout))
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "No complete current robot state was received within %.1f seconds.",
      state_timeout);
    return 1;
  }

  move_group.setStartStateToCurrentState();
  move_group.setMaxVelocityScalingFactor(velocity_scaling);
  move_group.setMaxAccelerationScalingFactor(acceleration_scaling);
  move_group.setPlanningTime(planning_time);

  if (!move_group.setJointValueTarget(joint_targets)) {
    RCLCPP_ERROR(
      node->get_logger(),
      "The requested joint target violates a joint limit or is invalid.");
    return 1;
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  RCLCPP_INFO(node->get_logger(), "Planning joint-space motion...");

  const auto planning_result = move_group.plan(plan);
  if (planning_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Motion planning failed with MoveIt error code %d.",
      planning_result.val);
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Planning succeeded.");
  if (plan_only) {
    RCLCPP_INFO(node->get_logger(), "Plan-only mode enabled; trajectory will not be executed.");
    return 0;
  }

  auto controller_client =
    rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
    node,
    controller_action);

  if (!controller_client->wait_for_action_server(
      std::chrono::duration<double>(controller_timeout)))
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Controller action '%s' was not available within %.1f seconds.",
      controller_action.c_str(),
      controller_timeout);
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Executing trajectory...");
  const auto execution_result = move_group.execute(plan);

  if (execution_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Trajectory execution failed with MoveIt error code %d.",
      execution_result.val);
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Trajectory execution succeeded.");
  return 0;
}

}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  const auto node_options =
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("wam_joint_motion", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread executor_thread([&executor]() {executor.spin();});

  int result = 1;
  try {
    result = run_motion(node);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(node->get_logger(), "Joint-motion client failed: %s", error.what());
  }

  executor.cancel();
  executor_thread.join();
  rclcpp::shutdown();
  return result;
}
