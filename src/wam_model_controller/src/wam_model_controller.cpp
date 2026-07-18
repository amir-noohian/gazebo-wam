#include <algorithm>
#include "wam_model_controller/wam_model_controller.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

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
    {50.0, 80.0, 50.0, 40.0, 10.0, 10.0, 5.0});

  auto_declare<std::vector<double>>(
    "kd",
    {5.0, 8.0, 5.0, 4.0, 1.0, 1.0, 0.5});

  auto_declare<std::vector<double>>(
    "torque_limits",
    {20.0, 20.0, 20.0, 20.0, 10.0, 10.0, 5.0});

  auto_declare<bool>(
    "hold_current_position",
    true);

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

  for (std::size_t i = 0; i < joint_names_.size(); ++i)
  {
    q_[i] = state_interfaces_[2 * i].get_value();
    dq_[i] = state_interfaces_[2 * i + 1].get_value();

    if (hold_current_position_)
    {
      q_des_[i] = q_[i];
    }

    command_interfaces_[i].set_value(0.0);
  }

  if (hold_current_position_)
  {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "Captured the current joint configuration as q_des.");
  }
  else
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
  q_[i] = state_interfaces_[2 * i].get_value();
  dq_[i] = state_interfaces_[2 * i + 1].get_value();

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
  for (std::size_t i = 0; i < joint_names_.size(); ++i)
  {
    q_[i] = state_interfaces_[2 * i].get_value();
    dq_[i] = state_interfaces_[2 * i + 1].get_value();

    const double position_error =
      q_des_[i] - q_[i];

    const double raw_torque =
      kp_[i] * position_error -
      kd_[i] * dq_[i];

    const double torque =
    std::max(
        -torque_limits_[i],
        std::min(raw_torque, torque_limits_[i]));

    command_interfaces_[i].set_value(torque);
  }

  RCLCPP_INFO_THROTTLE(
    get_node()->get_logger(),
    *get_node()->get_clock(),
    2000,
    "Joint 1: q=%.3f, q_des=%.3f, dq=%.3f, tau=%.3f",
    q_[0],
    q_des_[0],
    dq_[0],
    command_interfaces_[0].get_value());

  return controller_interface::return_type::OK;
}

}   // namespace wam_model_controller

PLUGINLIB_EXPORT_CLASS(
  wam_model_controller::WamModelController,
  controller_interface::ControllerInterface)