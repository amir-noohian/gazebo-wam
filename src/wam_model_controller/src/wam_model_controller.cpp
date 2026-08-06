#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>

#include "wam_model_controller/wam_model_controller.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/parameter_client.hpp"

namespace wam_model_controller
{

controller_interface::return_type
WamModelController::init(const std::string & controller_name)
{
  const auto result =
    controller_interface::ControllerInterface::init(controller_name);

  if (result != controller_interface::return_type::OK)
  {
    return result;
  }

  joint_names_ = {
    "wam/base_yaw_joint",
    "wam/shoulder_pitch_joint",
    "wam/shoulder_yaw_joint",
    "wam/elbow_pitch_joint",
    "wam/wrist_yaw_joint",
    "wam/wrist_pitch_joint",
    "wam/palm_yaw_joint"
  };

  auto_declare<std::vector<double>>(
    "q_des",
    std::vector<double>(joint_names_.size(), 0.0));

  auto_declare<std::vector<double>>(
    "kp",
    {50.0, 80.0, 50.0, 40.0, 10.0, 10.0, 0.2});

  auto_declare<std::vector<double>>(
    "kd",
    {5.0, 8.0, 5.0, 4.0, 1.0, 1.0, 0.01});

  auto_declare<std::vector<double>>(
    "torque_limits",
    {20.0, 20.0, 20.0, 20.0, 10.0, 10.0, 5.0});

  auto_declare<bool>(
    "hold_current_position",
    true);

  auto_declare<double>("trajectory_velocity", 0.3);
  auto_declare<double>("trajectory_acceleration", 0.3);

  auto_declare<std::string>(
    "root_link",
    "wam/base_link");

  auto_declare<std::string>(
    "tip_link",
    "wam/wrist_palm_link");

  RCLCPP_INFO(
    get_node()->get_logger(),
    "WamModelController initialized with %zu joints.",
    joint_names_.size());

  return controller_interface::return_type::OK;
}

controller_interface::InterfaceConfiguration
WamModelController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration configuration;

  configuration.type =
    controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & joint_name : joint_names_)
  {
    configuration.names.push_back(
      joint_name + "/" + hardware_interface::HW_IF_EFFORT);
  }

  return configuration;
}

controller_interface::InterfaceConfiguration
WamModelController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration configuration;

  configuration.type =
    controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & joint_name : joint_names_)
  {
    configuration.names.push_back(
      joint_name + "/" + hardware_interface::HW_IF_POSITION);

    configuration.names.push_back(
      joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
  }

  return configuration;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
WamModelController::on_configure(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  const std::size_t number_of_joints = joint_names_.size();

  q_.assign(number_of_joints, 0.0);
  dq_.assign(number_of_joints, 0.0);

  q_des_ =
    get_node()->get_parameter("q_des").as_double_array();

  kp_ =
    get_node()->get_parameter("kp").as_double_array();

  kd_ =
    get_node()->get_parameter("kd").as_double_array();

  torque_limits_ =
    get_node()->get_parameter("torque_limits").as_double_array();

  hold_current_position_ =
    get_node()->get_parameter("hold_current_position").as_bool();

  trajectory_velocity_ =
    get_node()->get_parameter("trajectory_velocity").as_double();

  trajectory_acceleration_ =
    get_node()->get_parameter("trajectory_acceleration").as_double();

  root_link_ =
    get_node()->get_parameter("root_link").as_string();

  tip_link_ =
    get_node()->get_parameter("tip_link").as_string();

  if (
    q_des_.size() != number_of_joints ||
    kp_.size() != number_of_joints ||
    kd_.size() != number_of_joints ||
    torque_limits_.size() != number_of_joints)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "q_des, kp, kd, and torque_limits must each contain %zu values.",
      number_of_joints);

    return
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::ERROR;
  }

  if (trajectory_velocity_ <= 0.0 || trajectory_acceleration_ <= 0.0)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "trajectory_velocity and trajectory_acceleration must be positive.");
    return
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::ERROR;
  }

  /*
   * The controller node is already managed by the controller manager's
   * executor. Therefore, do not create SyncParametersClient using get_node().
   *
   * We create a separate temporary node to retrieve robot_description from
   * robot_state_publisher.
   */
  auto parameter_client_node =
    std::make_shared<rclcpp::Node>(
      "wam_model_controller_parameter_client");

  auto parameter_client =
    std::make_shared<rclcpp::SyncParametersClient>(
      parameter_client_node,
      "/robot_state_publisher");

  if (!parameter_client->wait_for_service(std::chrono::seconds(5)))
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Parameter service for /robot_state_publisher is not available.");

    return
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::ERROR;
  }

  const auto robot_description_parameters =
    parameter_client->get_parameters(
      {"robot_description"});

  if (
    robot_description_parameters.empty() ||
    robot_description_parameters[0].get_type() !=
    rclcpp::ParameterType::PARAMETER_STRING)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Could not retrieve robot_description from /robot_state_publisher.");

    return
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::ERROR;
  }

  const std::string robot_description =
    robot_description_parameters[0].as_string();

  if (robot_description.empty())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "robot_description retrieved from /robot_state_publisher is empty.");

    return
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::ERROR;
  }

  kdl_initialized_ = false;
  kdl_dynamics_solver_.reset();

  if (!kdl_parser::treeFromString(robot_description, kdl_tree_))
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Failed to create the KDL tree from robot_description.");

    return
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::ERROR;
  }

  if (!kdl_tree_.getChain(root_link_, tip_link_, kdl_chain_))
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Failed to create KDL chain from '%s' to '%s'.",
      root_link_.c_str(),
      tip_link_.c_str());

    return
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::ERROR;
  }

  const std::size_t kdl_joint_count =
    kdl_chain_.getNrOfJoints();

  if (kdl_joint_count != number_of_joints)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "KDL chain contains %zu joints, but controller expects %zu.",
      kdl_joint_count,
      number_of_joints);

    return
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::ERROR;
  }

  kdl_q_.resize(kdl_joint_count);
  kdl_gravity_.resize(kdl_joint_count);

  /*
   * Gravity is expressed in the KDL chain root frame.
   *
   * Since wam/base_link is aligned with the Gazebo world frame in your URDF,
   * gravity acts along negative Z.
   */
  const KDL::Vector gravity_vector(
    0.0,
    0.0,
    -9.81);

  kdl_dynamics_solver_ =
    std::make_unique<KDL::ChainDynParam>(
      kdl_chain_,
      gravity_vector);

  kdl_initialized_ = true;

  auto completion_publisher =
    get_node()->create_publisher<std_msgs::msg::Bool>(
      "~/trajectory_complete", rclcpp::SystemDefaultsQoS());
  completion_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::Bool>>(
      completion_publisher);

  RCLCPP_INFO(
    get_node()->get_logger(),
    "KDL initialized: %s -> %s, %zu joints, %u segments.",
    root_link_.c_str(),
    tip_link_.c_str(),
    kdl_joint_count,
    kdl_chain_.getNrOfSegments());

  RCLCPP_INFO(
    get_node()->get_logger(),
    "WamModelController configured for %zu joints.",
    number_of_joints);

  return
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
    CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
WamModelController::on_activate(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  const std::size_t expected_state_interfaces =
    2 * joint_names_.size();

  if (state_interfaces_.size() != expected_state_interfaces)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Expected %zu state interfaces, but received %zu.",
      expected_state_interfaces,
      state_interfaces_.size());

    return
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::ERROR;
  }

  if (command_interfaces_.size() != joint_names_.size())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Expected %zu effort command interfaces, but received %zu.",
      joint_names_.size(),
      command_interfaces_.size());

    return
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::ERROR;
  }

  if (!kdl_initialized_ || !kdl_dynamics_solver_)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Cannot activate controller because KDL is not initialized.");

    return
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::ERROR;
  }

  trajectory_start_.resize(joint_names_.size());
  trajectory_target_ = q_des_;
  trajectory_direction_.assign(joint_names_.size(), 0.0);
  dq_des_.assign(joint_names_.size(), 0.0);
  trajectory_path_length_ = 0.0;

  for (std::size_t i = 0; i < joint_names_.size(); ++i)
  {
    q_[i] =
      state_interfaces_[2 * i].get_value();

    dq_[i] =
      state_interfaces_[2 * i + 1].get_value();

    trajectory_start_[i] = q_[i];
    const double displacement = trajectory_target_[i] - trajectory_start_[i];
    trajectory_path_length_ += displacement * displacement;

    command_interfaces_[i].set_value(0.0);
  }

  trajectory_path_length_ = std::sqrt(trajectory_path_length_);
  trajectory_active_ = false;
  completion_pending_ = false;

  if (hold_current_position_ || trajectory_path_length_ < 1e-9)
  {
    q_des_ = trajectory_start_;
  }
  else
  {
    for (std::size_t i = 0; i < joint_names_.size(); ++i)
    {
      trajectory_direction_[i] =
        (trajectory_target_[i] - trajectory_start_[i]) / trajectory_path_length_;
    }

    // If the path is too short to reach the requested velocity, the profile
    // is triangular and its peak velocity is sqrt(acceleration * length).
    const double distance_to_accelerate_and_decelerate =
      trajectory_velocity_ * trajectory_velocity_ / trajectory_acceleration_;

    if (trajectory_path_length_ <= distance_to_accelerate_and_decelerate)
    {
      trajectory_peak_velocity_ =
        std::sqrt(trajectory_acceleration_ * trajectory_path_length_);
      trajectory_accel_time_ =
        trajectory_peak_velocity_ / trajectory_acceleration_;
      trajectory_cruise_time_ = 0.0;
    }
    else
    {
      trajectory_peak_velocity_ = trajectory_velocity_;
      trajectory_accel_time_ =
        trajectory_peak_velocity_ / trajectory_acceleration_;
      const double acceleration_distance =
        trajectory_peak_velocity_ * trajectory_peak_velocity_ /
        trajectory_acceleration_;
      trajectory_cruise_time_ =
        (trajectory_path_length_ - acceleration_distance) /
        trajectory_peak_velocity_;
    }

    trajectory_duration_ =
      2.0 * trajectory_accel_time_ + trajectory_cruise_time_;
    trajectory_start_time_ = get_node()->now();
    trajectory_active_ = true;
    q_des_ = trajectory_start_;

    RCLCPP_INFO(
      get_node()->get_logger(),
      "Started joint trajectory: length=%.3f rad, peak velocity=%.3f rad/s, "
      "acceleration=%.3f rad/s^2, duration=%.3f s.",
      trajectory_path_length_, trajectory_peak_velocity_,
      trajectory_acceleration_, trajectory_duration_);
  }

  if (hold_current_position_)
  {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "Captured the current joint configuration as q_des.");
  }
  else if (!trajectory_active_)
  {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "Using q_des from ros2_controllers.yaml.");
  }

  return
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
    CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
WamModelController::on_deactivate(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  for (std::size_t i = 0; i < joint_names_.size(); ++i)
  {
    q_[i] =
      state_interfaces_[2 * i].get_value();

    dq_[i] =
      state_interfaces_[2 * i + 1].get_value();

    q_des_[i] = q_[i];
  }

  for (auto & command_interface : command_interfaces_)
  {
    command_interface.set_value(0.0);
  }

  RCLCPP_INFO(
    get_node()->get_logger(),
    "WamModelController deactivated.");

  return
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
    CallbackReturn::SUCCESS;
}

controller_interface::return_type
WamModelController::update()
{
  if (!kdl_initialized_ || !kdl_dynamics_solver_)
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      2000,
      "KDL dynamics solver is not initialized.");

    return controller_interface::return_type::ERROR;
  }

  // Evaluate the scalar trapezoidal profile. Its output is path distance and
  // path velocity; the unit path direction maps both values to every joint.
  if (trajectory_active_)
  {
    const double elapsed =
      std::max(0.0, (get_node()->now() - trajectory_start_time_).seconds());
    double path_position = 0.0;
    double path_velocity = 0.0;

    if (elapsed < trajectory_accel_time_)
    {
      path_velocity = trajectory_acceleration_ * elapsed;
      path_position = 0.5 * trajectory_acceleration_ * elapsed * elapsed;
    }
    else if (elapsed < trajectory_accel_time_ + trajectory_cruise_time_)
    {
      const double acceleration_distance =
        0.5 * trajectory_acceleration_ *
        trajectory_accel_time_ * trajectory_accel_time_;
      const double cruise_elapsed = elapsed - trajectory_accel_time_;
      path_velocity = trajectory_peak_velocity_;
      path_position =
        acceleration_distance + trajectory_peak_velocity_ * cruise_elapsed;
    }
    else if (elapsed < trajectory_duration_)
    {
      const double remaining = trajectory_duration_ - elapsed;
      path_velocity = trajectory_acceleration_ * remaining;
      path_position =
        trajectory_path_length_ -
        0.5 * trajectory_acceleration_ * remaining * remaining;
    }
    else
    {
      path_position = trajectory_path_length_;
      path_velocity = 0.0;
      trajectory_active_ = false;
      completion_pending_ = true;
      RCLCPP_INFO(get_node()->get_logger(), "Joint trajectory completed.");
    }

    for (std::size_t i = 0; i < joint_names_.size(); ++i)
    {
      q_des_[i] =
        trajectory_start_[i] + trajectory_direction_[i] * path_position;
      dq_des_[i] = trajectory_direction_[i] * path_velocity;
    }
  }

  if (completion_pending_ && completion_publisher_ && completion_publisher_->trylock())
  {
    completion_publisher_->msg_.data = true;
    completion_publisher_->unlockAndPublish();
    completion_pending_ = false;
  }

  /*
   * First read every joint state and copy the joint positions into the KDL
   * joint array. Gravity must be calculated using the complete configuration.
   */
  for (std::size_t i = 0; i < joint_names_.size(); ++i)
  {
    q_[i] =
      state_interfaces_[2 * i].get_value();

    dq_[i] =
      state_interfaces_[2 * i + 1].get_value();

    if (!std::isfinite(q_[i]) || !std::isfinite(dq_[i]))
    {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(),
        *get_node()->get_clock(),
        2000,
        "Invalid state for joint %s: q=%f, dq=%f.",
        joint_names_[i].c_str(),
        q_[i],
        dq_[i]);

      for (auto & command_interface : command_interfaces_)
      {
        command_interface.set_value(0.0);
      }

      return controller_interface::return_type::ERROR;
    }

    kdl_q_(i) = q_[i];
  }

  /*
   * Calculate the gravity torque vector:
   *
   *     g(q)
   */
  const int gravity_result =
    kdl_dynamics_solver_->JntToGravity(
      kdl_q_,
      kdl_gravity_);

  if (gravity_result < 0)
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      2000,
      "KDL failed to calculate gravity. Error code: %d.",
      gravity_result);

    for (auto & command_interface : command_interfaces_)
    {
      command_interface.set_value(0.0);
    }

    return controller_interface::return_type::ERROR;
  }

  /*
   * Gravity compensation plus joint-space PD:
   *
   * tau = g(q) + Kp(q_des - q) + Kd(dq_des - dq)
   */
  for (std::size_t i = 0; i < joint_names_.size(); ++i)
  {
    const double position_error =
      q_des_[i] - q_[i];

    const double gravity_torque =
      kdl_gravity_(i);

    const double pd_torque =
      kp_[i] * position_error +
      kd_[i] * (dq_des_[i] - dq_[i]);

    const double raw_torque =
      gravity_torque + pd_torque;

    const double limited_torque =
      std::max(
        -torque_limits_[i],
        std::min(
          raw_torque,
          torque_limits_[i]));

    if (!std::isfinite(limited_torque))
    {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(),
        *get_node()->get_clock(),
        2000,
        "Invalid torque calculated for joint %s.",
        joint_names_[i].c_str());

      command_interfaces_[i].set_value(0.0);
      continue;
    }

    command_interfaces_[i].set_value(limited_torque);
  }

  std::ostringstream log_stream;

  log_stream << std::fixed << std::setprecision(3);
  log_stream << "\nWAM controller state:\n";

  for (std::size_t i = 0; i < joint_names_.size(); ++i)
  {
    const double position_error =
      q_des_[i] - q_[i];

    const double pd_torque =
      kp_[i] * position_error +
      kd_[i] * (dq_des_[i] - dq_[i]);

    const double commanded_torque =
      command_interfaces_[i].get_value();

    log_stream
      << "[" << i << "] "
      << joint_names_[i]
      << " | q: " << q_[i]
      << " | q_des: " << q_des_[i]
      << " | dq: " << dq_[i]
      << " | error: " << position_error
      << " | gravity: " << kdl_gravity_(i)
      << " | PD: " << pd_torque
      << " | command: " << commanded_torque
      << "\n";
  }

  RCLCPP_INFO_THROTTLE(
    get_node()->get_logger(),
    *get_node()->get_clock(),
    2000,
    "%s",
    log_stream.str().c_str());

  return controller_interface::return_type::OK;
}

}  // namespace wam_model_controller

PLUGINLIB_EXPORT_CLASS(
  wam_model_controller::WamModelController,
  controller_interface::ControllerInterface)
