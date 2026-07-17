#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
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

geometry_msgs::msg::Quaternion quaternion_from_rpy(
  double roll,
  double pitch,
  double yaw)
{
  tf2::Quaternion quaternion;
  quaternion.setRPY(roll, pitch, yaw);
  quaternion.normalize();

  geometry_msgs::msg::Quaternion result;
  result.x = quaternion.x();
  result.y = quaternion.y();
  result.z = quaternion.z();
  result.w = quaternion.w();
  return result;
}

geometry_msgs::msg::Quaternion apply_local_rpy_offset(
  const geometry_msgs::msg::Quaternion & current,
  double roll,
  double pitch,
  double yaw)
{
  tf2::Quaternion current_rotation(current.x, current.y, current.z, current.w);
  tf2::Quaternion offset;
  offset.setRPY(roll, pitch, yaw);

  tf2::Quaternion target = current_rotation * offset;
  target.normalize();

  geometry_msgs::msg::Quaternion result;
  result.x = target.x();
  result.y = target.y();
  result.z = target.z();
  result.w = target.w();
  return result;
}

int run_motion(const rclcpp::Node::SharedPtr & node)
{
  const std::string planning_group =
    declare_if_missing<std::string>(node, "planning_group", "arm");
  const std::string end_effector_link = declare_if_missing<std::string>(
    node, "end_effector_link", "wam/wrist_palm_stump_link");
  const std::string pose_reference_frame = declare_if_missing<std::string>(
    node, "pose_reference_frame", "wam/base_link");
  const std::string controller_action = declare_if_missing<std::string>(
    node,
    "controller_action",
    "/arm_controller/follow_joint_trajectory");

  const bool relative = declare_if_missing<bool>(node, "relative", true);
  const bool linear_path = declare_if_missing<bool>(node, "linear_path", true);
  const bool avoid_collisions =
    declare_if_missing<bool>(node, "avoid_collisions", true);
  const bool plan_only = declare_if_missing<bool>(node, "plan_only", false);

  const double target_x =
    declare_if_missing<double>(node, "target_position.x", 0.0);
  const double target_y =
    declare_if_missing<double>(node, "target_position.y", 0.0);
  const double target_z =
    declare_if_missing<double>(node, "target_position.z", -0.05);
  const double target_roll =
    declare_if_missing<double>(node, "target_orientation_rpy.roll", 0.0);
  const double target_pitch =
    declare_if_missing<double>(node, "target_orientation_rpy.pitch", 0.0);
  const double target_yaw =
    declare_if_missing<double>(node, "target_orientation_rpy.yaw", 0.0);

  const double position_tolerance =
    declare_if_missing<double>(node, "position_tolerance", 0.005);
  const double orientation_tolerance =
    declare_if_missing<double>(node, "orientation_tolerance", 0.01);
  const double velocity_scaling =
    declare_if_missing<double>(node, "velocity_scaling", 0.2);
  const double acceleration_scaling =
    declare_if_missing<double>(node, "acceleration_scaling", 0.2);
  const double planning_time =
    declare_if_missing<double>(node, "planning_time", 5.0);
  const double eef_step =
    declare_if_missing<double>(node, "eef_step", 0.005);
  const double jump_threshold =
    declare_if_missing<double>(node, "jump_threshold", 1.0);
  const double minimum_path_fraction =
    declare_if_missing<double>(node, "minimum_path_fraction", 0.99);
  const double server_timeout =
    declare_if_missing<double>(node, "server_timeout", 10.0);
  const double state_timeout =
    declare_if_missing<double>(node, "state_timeout", 10.0);
  const double controller_timeout =
    declare_if_missing<double>(node, "controller_timeout", 10.0);

  if (!valid_scaling_factor(velocity_scaling) ||
    !valid_scaling_factor(acceleration_scaling))
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Velocity and acceleration scaling factors must be in (0, 1].");
    return 1;
  }

  if (position_tolerance <= 0.0 || orientation_tolerance <= 0.0 ||
    planning_time <= 0.0 || eef_step <= 0.0 || jump_threshold < 0.0 ||
    minimum_path_fraction <= 0.0 || minimum_path_fraction > 1.0 ||
    server_timeout <= 0.0 || state_timeout <= 0.0 || controller_timeout <= 0.0)
  {
    RCLCPP_ERROR(node->get_logger(), "One or more Cartesian-motion parameters are invalid.");
    return 1;
  }

  moveit::planning_interface::MoveGroupInterface move_group(
    node,
    planning_group,
    std::shared_ptr<tf2_ros::Buffer>(),
    rclcpp::Duration::from_seconds(server_timeout));

  if (!move_group.startStateMonitor(state_timeout)) {
    RCLCPP_ERROR(
      node->get_logger(),
      "No complete current robot state was received within %.1f seconds.",
      state_timeout);
    return 1;
  }

  const moveit::core::RobotStatePtr current_state =
    move_group.getCurrentState(state_timeout);
  if (!current_state) {
    RCLCPP_ERROR(node->get_logger(), "Failed to read the current robot state.");
    return 1;
  }

  move_group.setPoseReferenceFrame(pose_reference_frame);
  const geometry_msgs::msg::PoseStamped current_pose =
    move_group.getCurrentPose(end_effector_link);
  geometry_msgs::msg::PoseStamped target_pose;

  if (relative) {
    target_pose = current_pose;
    target_pose.pose.position.x += target_x;
    target_pose.pose.position.y += target_y;
    target_pose.pose.position.z += target_z;
    target_pose.pose.orientation = apply_local_rpy_offset(
      current_pose.pose.orientation,
      target_roll,
      target_pitch,
      target_yaw);
  } else {
    target_pose.header.frame_id = pose_reference_frame;
    target_pose.header.stamp = node->now();
    target_pose.pose.position.x = target_x;
    target_pose.pose.position.y = target_y;
    target_pose.pose.position.z = target_z;
    target_pose.pose.orientation =
      quaternion_from_rpy(target_roll, target_pitch, target_yaw);
  }

  if (target_pose.header.frame_id.empty()) {
    target_pose.header.frame_id = pose_reference_frame;
  }

  move_group.setPoseReferenceFrame(target_pose.header.frame_id);
  move_group.setStartStateToCurrentState();
  move_group.setGoalPositionTolerance(position_tolerance);
  move_group.setGoalOrientationTolerance(orientation_tolerance);
  move_group.setMaxVelocityScalingFactor(velocity_scaling);
  move_group.setMaxAccelerationScalingFactor(acceleration_scaling);
  move_group.setPlanningTime(planning_time);

  RCLCPP_INFO(node->get_logger(), "Planning group: %s", planning_group.c_str());
  RCLCPP_INFO(node->get_logger(), "End-effector link: %s", end_effector_link.c_str());
  RCLCPP_INFO(node->get_logger(), "Pose reference frame: %s", target_pose.header.frame_id.c_str());
  RCLCPP_INFO(
    node->get_logger(),
    "Target position: [%.4f, %.4f, %.4f]",
    target_pose.pose.position.x,
    target_pose.pose.position.y,
    target_pose.pose.position.z);

  moveit_msgs::msg::RobotTrajectory cartesian_trajectory;
  moveit::planning_interface::MoveGroupInterface::Plan pose_plan;

  if (linear_path) {
    std::vector<geometry_msgs::msg::Pose> waypoints = {target_pose.pose};
    moveit_msgs::msg::MoveItErrorCodes cartesian_error;
    const double path_fraction = move_group.computeCartesianPath(
      waypoints,
      eef_step,
      jump_threshold,
      cartesian_trajectory,
      avoid_collisions,
      &cartesian_error);

    RCLCPP_INFO(
      node->get_logger(),
      "Cartesian path completion: %.1f%%",
      100.0 * path_fraction);

    if (path_fraction < minimum_path_fraction) {
      if (cartesian_error.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
        RCLCPP_ERROR(
          node->get_logger(),
          "Cartesian path was incomplete although the request was valid; "
          "the target may be unreachable or too close to a singularity.");
      } else {
        RCLCPP_ERROR(
          node->get_logger(),
          "Cartesian path was incomplete (MoveIt error code %d).",
          cartesian_error.val);
      }
      return 1;
    }

    robot_trajectory::RobotTrajectory timed_trajectory(
      move_group.getRobotModel(),
      planning_group);
    timed_trajectory.setRobotTrajectoryMsg(*current_state, cartesian_trajectory);

    trajectory_processing::IterativeParabolicTimeParameterization time_parameterization;
    if (!time_parameterization.computeTimeStamps(
        timed_trajectory,
        velocity_scaling,
        acceleration_scaling))
    {
      RCLCPP_ERROR(node->get_logger(), "Failed to time-parameterize the Cartesian path.");
      return 1;
    }

    timed_trajectory.getRobotTrajectoryMsg(cartesian_trajectory);
  } else {
    if (!move_group.setPoseTarget(target_pose, end_effector_link)) {
      RCLCPP_ERROR(node->get_logger(), "The Cartesian pose target is invalid.");
      return 1;
    }

    const auto planning_result = move_group.plan(pose_plan);
    move_group.clearPoseTargets();

    if (planning_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      RCLCPP_ERROR(
        node->get_logger(),
        "Pose-goal planning failed with MoveIt error code %d.",
        planning_result.val);
      return 1;
    }
  }

  RCLCPP_INFO(node->get_logger(), "Cartesian planning succeeded.");
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

  RCLCPP_INFO(node->get_logger(), "Executing Cartesian motion...");
  const auto execution_result = linear_path ?
    move_group.execute(cartesian_trajectory) : move_group.execute(pose_plan);

  if (execution_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Cartesian execution failed with MoveIt error code %d.",
      execution_result.val);
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Cartesian motion succeeded.");
  return 0;
}

}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  const auto node_options =
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("wam_cartesian_motion", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread executor_thread([&executor]() {executor.spin();});

  int result = 1;
  try {
    result = run_motion(node);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(node->get_logger(), "Cartesian-motion client failed: %s", error.what());
  }

  executor.cancel();
  executor_thread.join();
  rclcpp::shutdown();
  return result;
}
