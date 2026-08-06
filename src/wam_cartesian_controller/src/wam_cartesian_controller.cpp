#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>
#include <functional>

#include "wam_cartesian_controller/wam_cartesian_controller.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/parameter_client.hpp"

namespace wam_cartesian_controller
{

controller_interface::return_type
WamCartesianController::init(const std::string & controller_name)
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
    "cartesian_kp",
    {30.0, 30.0, 30.0});

  auto_declare<std::vector<double>>(
    "cartesian_kd",
    {10.0, 10.0, 10.0});

  auto_declare<std::vector<double>>(
    "torque_limits",
    {20.0, 20.0, 20.0, 20.0, 10.0, 10.0, 5.0});

  auto_declare<bool>(
    "hold_current_position",
    true);

  auto_declare<double>("trajectory_velocity", 0.05);
  auto_declare<double>("trajectory_acceleration", 0.05);
  auto_declare<std::string>("nullspace_mode", "fixed");
  auto_declare<int>("nullspace_joint", 6);
  auto_declare<double>("nullspace_target", -0.7853981633974483);
  auto_declare<double>("nullspace_sine_amplitude", 0.2);
  auto_declare<double>("nullspace_sine_frequency", 0.1);
  auto_declare<double>("nullspace_kp", 5.0);
  auto_declare<double>("nullspace_kd", 1.0);
  auto_declare<double>("nullspace_damping", 0.01);
  auto_declare<double>("nullspace_max_torque", 0.2);

  auto_declare<std::string>(
    "root_link",
    "wam/base_link");

  auto_declare<std::string>(
    "tip_link",
    "wam/wrist_palm_link");

  RCLCPP_INFO(
    get_node()->get_logger(),
    "WamCartesianController initialized with %zu joints.",
    joint_names_.size());

  return controller_interface::return_type::OK;
}

controller_interface::InterfaceConfiguration
WamCartesianController::command_interface_configuration() const
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
WamCartesianController::state_interface_configuration() const
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
WamCartesianController::on_configure(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  const std::size_t number_of_joints = joint_names_.size();

  q_.assign(number_of_joints, 0.0);
  dq_.assign(number_of_joints, 0.0);

  const auto cartesian_kp =
    get_node()->get_parameter("cartesian_kp").as_double_array();

  const auto cartesian_kd =
    get_node()->get_parameter("cartesian_kd").as_double_array();

  torque_limits_ =
    get_node()->get_parameter("torque_limits").as_double_array();

  hold_current_position_ =
    get_node()->get_parameter("hold_current_position").as_bool();

  trajectory_velocity_ =
    get_node()->get_parameter("trajectory_velocity").as_double();

  trajectory_acceleration_ =
    get_node()->get_parameter("trajectory_acceleration").as_double();

  nullspace_mode_ =
    get_node()->get_parameter("nullspace_mode").as_string();
  const auto nullspace_joint =
    get_node()->get_parameter("nullspace_joint").as_int();
  nullspace_target_ =
    get_node()->get_parameter("nullspace_target").as_double();
  nullspace_sine_amplitude_ =
    get_node()->get_parameter("nullspace_sine_amplitude").as_double();
  nullspace_sine_frequency_ =
    get_node()->get_parameter("nullspace_sine_frequency").as_double();
  nullspace_kp_ =
    get_node()->get_parameter("nullspace_kp").as_double();
  nullspace_kd_ =
    get_node()->get_parameter("nullspace_kd").as_double();
  nullspace_damping_ =
    get_node()->get_parameter("nullspace_damping").as_double();
  nullspace_max_torque_ =
    get_node()->get_parameter("nullspace_max_torque").as_double();

  root_link_ =
    get_node()->get_parameter("root_link").as_string();

  tip_link_ =
    get_node()->get_parameter("tip_link").as_string();

  if (
    cartesian_kp.size() != 3 ||
    cartesian_kd.size() != 3 ||
    torque_limits_.size() != number_of_joints)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "cartesian_kp and cartesian_kd must contain 3 values; "
      "torque_limits must contain %zu values.",
      number_of_joints);

    return
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::ERROR;
  }

  for (Eigen::Index axis = 0; axis < 3; ++axis)
  {
    if (!std::isfinite(cartesian_kp[axis]) || cartesian_kp[axis] < 0.0 ||
      !std::isfinite(cartesian_kd[axis]) || cartesian_kd[axis] < 0.0)
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Cartesian gains must be finite and non-negative.");
      return
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::ERROR;
    }

    cartesian_kp_(axis) = cartesian_kp[axis];
    cartesian_kd_(axis) = cartesian_kd[axis];
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

  if ((nullspace_mode_ != "fixed" && nullspace_mode_ != "sine") ||
    nullspace_joint < 1 || nullspace_joint > static_cast<int64_t>(number_of_joints) ||
    !std::isfinite(nullspace_target_) ||
    !std::isfinite(nullspace_sine_amplitude_) || nullspace_sine_amplitude_ < 0.0 ||
    !std::isfinite(nullspace_sine_frequency_) || nullspace_sine_frequency_ <= 0.0 ||
    nullspace_kp_ < 0.0 ||
    nullspace_kd_ < 0.0 || nullspace_damping_ <= 0.0 ||
    nullspace_max_torque_ <= 0.0)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "One or more null-space parameters are invalid.");
    return
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::ERROR;
  }
  nullspace_joint_index_ = static_cast<std::size_t>(nullspace_joint - 1);

  /*
   * The controller node is already managed by the controller manager's
   * executor. Therefore, do not create SyncParametersClient using get_node().
   *
   * We create a separate temporary node to retrieve robot_description from
   * robot_state_publisher.
   */
  auto parameter_client_node =
    std::make_shared<rclcpp::Node>(
      "wam_cartesian_controller_parameter_client");

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

  desired_position_.setZero();
  nullspace_torque_.setZero();

  desired_linear_velocity_.setZero();
  trajectory_start_position_.setZero();
  trajectory_target_position_.setZero();

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

  kdl_fk_solver_ =
    std::make_unique<KDL::ChainFkSolverPos_recursive>(
      kdl_chain_);
  
  kdl_jacobian_solver_ =
    std::make_unique<KDL::ChainJntToJacSolver>(kdl_chain_);

  kdl_jacobian_.resize(kdl_chain_.getNrOfJoints());

  kdl_initialized_ = true;

  target_subscription_ =
    get_node()->create_subscription<geometry_msgs::msg::PointStamped>(
      "~/target_position",
      rclcpp::SystemDefaultsQoS(),
      std::bind(
        &WamCartesianController::target_callback,
        this,
        std::placeholders::_1));

  RCLCPP_INFO(
    get_node()->get_logger(),
    "KDL initialized: %s -> %s, %zu joints, %u segments.",
    root_link_.c_str(),
    tip_link_.c_str(),
    kdl_joint_count,
    kdl_chain_.getNrOfSegments());
  
  RCLCPP_INFO(
    get_node()->get_logger(),
    "KDL forward-kinematics solver created for chain with %u joints.",
    kdl_chain_.getNrOfJoints());
  
  RCLCPP_INFO(
    get_node()->get_logger(),
    "KDL solvers created for chain with %u joints.",
    kdl_chain_.getNrOfJoints());

  RCLCPP_INFO(
    get_node()->get_logger(),
    "WamCartesianController configured for %zu joints.",
    number_of_joints);

  return
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
    CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
WamCartesianController::on_activate(
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

  if (
    !kdl_initialized_ ||
    !kdl_dynamics_solver_ ||
    !kdl_fk_solver_ ||
    !kdl_jacobian_solver_)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Cannot activate controller because KDL is not initialized.");

    return
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::ERROR;
  }

  position_state_indices_.resize(joint_names_.size());
  velocity_state_indices_.resize(joint_names_.size());

  // Print the actual joint and interface names.
  for (std::size_t interface_index = 0;
      interface_index < state_interfaces_.size();
      ++interface_index)
  {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "state_interfaces_[%zu] | joint=%s | interface=%s",
      interface_index,
      state_interfaces_[interface_index].get_name().c_str(),
      state_interfaces_[interface_index].get_interface_name().c_str());
  }

  for (std::size_t joint = 0; joint < joint_names_.size(); ++joint)
  {
    bool position_found = false;
    bool velocity_found = false;

    for (std::size_t interface_index = 0;
        interface_index < state_interfaces_.size();
        ++interface_index)
    {
      const std::string joint_name =
        state_interfaces_[interface_index].get_name();

      const std::string interface_name =
        state_interfaces_[interface_index].get_interface_name();

      if (
        joint_name == joint_names_[joint] &&
        interface_name == hardware_interface::HW_IF_POSITION)
      {
        position_state_indices_[joint] = interface_index;
        position_found = true;
      }

      if (
        joint_name == joint_names_[joint] &&
        interface_name == hardware_interface::HW_IF_VELOCITY)
      {
        velocity_state_indices_[joint] = interface_index;
        velocity_found = true;
      }
    }

    if (!position_found || !velocity_found)
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Could not find position and velocity interfaces for joint %s.",
        joint_names_[joint].c_str());

      return
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::ERROR;
    }

    RCLCPP_INFO(
      get_node()->get_logger(),
      "%s | position index=%zu | velocity index=%zu",
      joint_names_[joint].c_str(),
      position_state_indices_[joint],
      velocity_state_indices_[joint]);
  }

  // Read the initial state using the discovered indices.
  for (std::size_t i = 0; i < joint_names_.size(); ++i)
  {
    q_[i] =
      state_interfaces_[position_state_indices_[i]].get_value();

    dq_[i] =
      state_interfaces_[velocity_state_indices_[i]].get_value();

    if (!std::isfinite(q_[i]) || !std::isfinite(dq_[i]))
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Invalid initial state for joint %s: q=%f, dq=%f.",
        joint_names_[i].c_str(),
        q_[i],
        dq_[i]);

      return
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::ERROR;
    }

    kdl_q_(i) = q_[i];

    // command_interfaces_[i].set_value(0.0);
  }

  nullspace_sine_center_ = q_[nullspace_joint_index_];
  nullspace_start_time_ = get_node()->now();

  const int gravity_result =
    kdl_dynamics_solver_->JntToGravity(
      kdl_q_,
      kdl_gravity_);

  if (gravity_result < 0)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Failed to calculate initial gravity torque.");

    return
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::ERROR;
  }

  for (std::size_t i = 0; i < joint_names_.size(); ++i)
  {
    const double initial_gravity_torque =
      std::max(
        -torque_limits_[i],
        std::min(
          kdl_gravity_(i),
          torque_limits_[i]));

    command_interfaces_[i].set_value(
      initial_gravity_torque);
  }




  if (hold_current_position_)
  {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "The Cartesian target will be initialized from the current end-effector position.");
  }
  else
  {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "Using the configured Cartesian target.");
  }

  trajectory_initialized_ = false;
  trajectory_active_ = false;
  desired_linear_velocity_.setZero();
  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    target_pending_ = false;
  }

  return
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
    CallbackReturn::SUCCESS;
}

void WamCartesianController::target_callback(
  const geometry_msgs::msg::PointStamped::SharedPtr message)
{
  if (!message->header.frame_id.empty() && message->header.frame_id != root_link_)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Rejected Cartesian target in frame '%s'; expected '%s'.",
      message->header.frame_id.c_str(), root_link_.c_str());
    return;
  }

  const Eigen::Vector3d target(
    message->point.x,
    message->point.y,
    message->point.z);

  if (!target.allFinite())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Rejected non-finite Cartesian target.");
    return;
  }

  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    pending_target_position_ = target;
    target_pending_ = true;
  }

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Received Cartesian target [%.4f, %.4f, %.4f] in %s.",
    target(0), target(1), target(2), root_link_.c_str());
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
WamCartesianController::on_deactivate(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  if (
    position_state_indices_.size() == joint_names_.size() &&
    velocity_state_indices_.size() == joint_names_.size())
  {
    for (std::size_t i = 0; i < joint_names_.size(); ++i)
    {
      q_[i] =
        state_interfaces_[position_state_indices_[i]].get_value();

      dq_[i] =
        state_interfaces_[velocity_state_indices_[i]].get_value();

    }
  }

  for (auto & command_interface : command_interfaces_)
  {
    command_interface.set_value(0.0);
  }

  RCLCPP_INFO(
    get_node()->get_logger(),
    "WamCartesianController deactivated.");

  return
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
    CallbackReturn::SUCCESS;
}

controller_interface::return_type
WamCartesianController::update()
{
  // ---------------------------------------------------------
  // 1. Verify initialization
  // ---------------------------------------------------------
  if (
    !kdl_initialized_ ||
    !kdl_dynamics_solver_ ||
    !kdl_fk_solver_ ||
    !kdl_jacobian_solver_)
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      2000,
      "One or more KDL solvers are not initialized.");

    return controller_interface::return_type::ERROR;
  }

  if (
    position_state_indices_.size() != joint_names_.size() ||
    velocity_state_indices_.size() != joint_names_.size())
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      2000,
      "State-interface indices have not been initialized.");

    for (auto & command_interface : command_interfaces_)
    {
      command_interface.set_value(0.0);
    }

    return controller_interface::return_type::ERROR;
  }

  // ---------------------------------------------------------
  // 2. Read joint positions and velocities once
  // ---------------------------------------------------------
  for (std::size_t i = 0; i < joint_names_.size(); ++i)
  {
    q_[i] =
      state_interfaces_[position_state_indices_[i]].get_value();

    dq_[i] =
      state_interfaces_[velocity_state_indices_[i]].get_value();

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

  // ---------------------------------------------------------
  // 3. Forward kinematics
  // ---------------------------------------------------------
  const int fk_result =
    kdl_fk_solver_->JntToCart(
      kdl_q_,
      end_effector_pose_);

  if (fk_result < 0)
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      2000,
      "Forward kinematics failed with error code %d.",
      fk_result);

    for (auto & command_interface : command_interfaces_)
    {
      command_interface.set_value(0.0);
    }

    return controller_interface::return_type::ERROR;
  }

  current_position_ <<
    end_effector_pose_.p.x(),
    end_effector_pose_.p.y(),
    end_effector_pose_.p.z();

  // On activation, hold the measured end-effector position. Motion begins
  // only after an external target is received.
  if (!trajectory_initialized_)
  {
    trajectory_start_position_ = current_position_;
    trajectory_target_position_ = current_position_;
    desired_position_ = current_position_;
    desired_linear_velocity_.setZero();
    trajectory_initialized_ = true;
    trajectory_active_ = false;

    RCLCPP_INFO(
      get_node()->get_logger(),
      "Holding activation position [%.4f, %.4f, %.4f]; waiting for a target.",
      current_position_(0), current_position_(1), current_position_(2));
  }

  // Copy a newly received target without blocking the real-time update loop.
  Eigen::Vector3d new_target;
  bool have_new_target = false;
  if (target_mutex_.try_lock())
  {
    if (target_pending_)
    {
      new_target = pending_target_position_;
      target_pending_ = false;
      have_new_target = true;
    }
    target_mutex_.unlock();
  }

  if (have_new_target)
  {
    // Replanning starts from the measured position. This is intentional for
    // manual, point-to-point commands sent after the previous move completes.
    trajectory_start_position_ = current_position_;
    trajectory_target_position_ = new_target;
    trajectory_path_length_ =
      (trajectory_target_position_ - trajectory_start_position_).norm();

    if (trajectory_path_length_ < 1e-9)
    {
      desired_position_ = trajectory_target_position_;
      desired_linear_velocity_.setZero();
      trajectory_active_ = false;
      RCLCPP_INFO(get_node()->get_logger(), "Cartesian target is already reached.");
    }
    else
    {
      const double accel_decel_distance =
        trajectory_velocity_ * trajectory_velocity_ / trajectory_acceleration_;

      if (trajectory_path_length_ <= accel_decel_distance)
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
        trajectory_cruise_time_ =
          (trajectory_path_length_ - accel_decel_distance) /
          trajectory_peak_velocity_;
      }

      trajectory_duration_ =
        2.0 * trajectory_accel_time_ + trajectory_cruise_time_;
      trajectory_start_time_ = get_node()->now();
      trajectory_active_ = true;
      desired_position_ = trajectory_start_position_;
      desired_linear_velocity_.setZero();

      RCLCPP_INFO(
        get_node()->get_logger(),
        "Started Cartesian trajectory: length=%.4f m, peak velocity=%.4f m/s, "
        "acceleration=%.4f m/s^2, duration=%.3f s.",
        trajectory_path_length_, trajectory_peak_velocity_,
        trajectory_acceleration_, trajectory_duration_);
    }
  }

  // ---------------------------------------------------------
  // 4. Jacobian
  // ---------------------------------------------------------
  const int jacobian_result =
    kdl_jacobian_solver_->JntToJac(
      kdl_q_,
      kdl_jacobian_);

  if (jacobian_result < 0)
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      2000,
      "Jacobian calculation failed with error code %d.",
      jacobian_result);

    for (auto & command_interface : command_interfaces_)
    {
      command_interface.set_value(0.0);
    }

    return controller_interface::return_type::ERROR;
  }

  for (std::size_t row = 0; row < 6; ++row)
  {
    for (std::size_t joint = 0; joint < joint_names_.size(); ++joint)
    {
      jacobian_eigen_(row, joint) =
        kdl_jacobian_(row, joint);
    }
  }

  // ---------------------------------------------------------
  // 5. Cartesian velocity: x_dot = J(q) q_dot
  // ---------------------------------------------------------
  cartesian_velocity_.fill(0.0);

  for (std::size_t row = 0; row < 6; ++row)
  {
    for (std::size_t joint = 0; joint < joint_names_.size(); ++joint)
    {
      cartesian_velocity_[row] +=
        kdl_jacobian_(row, joint) * dq_[joint];
    }
  }

  linear_velocity_ <<
    cartesian_velocity_[0],
    cartesian_velocity_[1],
    cartesian_velocity_[2];



  // ---------------------------------------------------------
  // 6. Trapezoidal Cartesian trajectory
  // ---------------------------------------------------------
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
      path_velocity = trajectory_peak_velocity_;
      path_position = acceleration_distance +
        trajectory_peak_velocity_ * (elapsed - trajectory_accel_time_);
    }
    else if (elapsed < trajectory_duration_)
    {
      const double remaining = trajectory_duration_ - elapsed;
      path_velocity = trajectory_acceleration_ * remaining;
      path_position = trajectory_path_length_ -
        0.5 * trajectory_acceleration_ * remaining * remaining;
    }
    else
    {
      trajectory_active_ = false;
      path_position = trajectory_path_length_;
      path_velocity = 0.0;
      RCLCPP_INFO(get_node()->get_logger(), "Cartesian trajectory completed.");
    }

    const Eigen::Vector3d path_direction =
      (trajectory_target_position_ - trajectory_start_position_) /
      trajectory_path_length_;
    desired_position_ =
      trajectory_start_position_ + path_direction * path_position;
    desired_linear_velocity_ = path_direction * path_velocity;
  }

  position_error_ =
    desired_position_
    - current_position_;

  const Eigen::Vector3d velocity_error =
    desired_linear_velocity_
    - linear_velocity_;

  // ---------------------------------------------------------
  // 6. Cartesian PD force
  // F = Kp(x_des - x) - Kd*x_dot
  // ---------------------------------------------------------
  cartesian_force_ =
    cartesian_kp_.cwiseProduct(position_error_) +
    cartesian_kd_.cwiseProduct(velocity_error);

  constexpr double max_cartesian_force = 5.0;

  for (int axis = 0; axis < 3; ++axis)
  {
    cartesian_force_(axis) =
      std::max(
        -max_cartesian_force,
        std::min(
          cartesian_force_(axis),
          max_cartesian_force));
  }

  // ---------------------------------------------------------
  // 7. Position-only Cartesian wrench
  // W = [Fx, Fy, Fz, 0, 0, 0]^T
  // ---------------------------------------------------------
  cartesian_wrench_.setZero();
  cartesian_wrench_.head<3>() = cartesian_force_;

  // ---------------------------------------------------------
  // 8. Cartesian joint torque: tau_cart = J^T W
  // ---------------------------------------------------------
  cartesian_torque_ =
    jacobian_eigen_.transpose() * cartesian_wrench_;

  if (!cartesian_torque_.allFinite())
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      2000,
      "Cartesian torque contains invalid values.");

    for (auto & command_interface : command_interfaces_)
    {
      command_interface.set_value(0.0);
    }

    return controller_interface::return_type::ERROR;
  }

  // ---------------------------------------------------------
  // 9. Damped Moore-Penrose null-space posture control
  // ---------------------------------------------------------
  const Eigen::Matrix<double, 3, 7> translational_jacobian =
    jacobian_eigen_.topRows<3>();
  Eigen::Matrix3d damped_jacobian_product =
    translational_jacobian * translational_jacobian.transpose();
  damped_jacobian_product.diagonal().array() +=
    nullspace_damping_ * nullspace_damping_;

  const Eigen::LDLT<Eigen::Matrix3d> jacobian_decomposition(
    damped_jacobian_product);
  if (jacobian_decomposition.info() != Eigen::Success)
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      2000,
      "Failed to factor the damped Jacobian product.");
    for (auto & command_interface : command_interfaces_)
    {
      command_interface.set_value(0.0);
    }
    return controller_interface::return_type::ERROR;
  }

  const Eigen::Matrix<double, 7, 3> damped_pseudoinverse =
    translational_jacobian.transpose() *
    jacobian_decomposition.solve(Eigen::Matrix3d::Identity());
  const Eigen::Matrix<double, 7, 7> torque_nullspace_projector =
    Eigen::Matrix<double, 7, 7>::Identity() -
    translational_jacobian.transpose() * damped_pseudoinverse.transpose();

  Eigen::Matrix<double, 7, 1> raw_posture_torque =
    Eigen::Matrix<double, 7, 1>::Zero();
  double posture_target = nullspace_target_;
  double posture_velocity_target = 0.0;
  if (nullspace_mode_ == "sine")
  {
    constexpr double two_pi = 6.28318530717958647692;
    const double elapsed =
      std::max(0.0, (get_node()->now() - nullspace_start_time_).seconds());
    const double angular_frequency = two_pi * nullspace_sine_frequency_;
    posture_target = nullspace_sine_center_ +
      nullspace_sine_amplitude_ * std::sin(angular_frequency * elapsed);
    posture_velocity_target = nullspace_sine_amplitude_ * angular_frequency *
      std::cos(angular_frequency * elapsed);
  }

  const double posture_error = posture_target - q_[nullspace_joint_index_];
  const double posture_torque =
    nullspace_kp_ * posture_error +
    nullspace_kd_ * (posture_velocity_target - dq_[nullspace_joint_index_]);
  raw_posture_torque(nullspace_joint_index_) =
    std::max(
      -nullspace_max_torque_,
      std::min(posture_torque, nullspace_max_torque_));
  nullspace_torque_ = torque_nullspace_projector * raw_posture_torque;

  // Scale the complete projected vector so the secondary task cannot exceed
  // its configured torque-vector magnitude.
  const double projected_torque_norm = nullspace_torque_.norm();
  if (projected_torque_norm > nullspace_max_torque_)
  {
    nullspace_torque_ *= nullspace_max_torque_ / projected_torque_norm;
  }

  if (!nullspace_torque_.allFinite())
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      2000,
      "Null-space torque contains invalid values.");
    for (auto & command_interface : command_interfaces_)
    {
      command_interface.set_value(0.0);
    }
    return controller_interface::return_type::ERROR;
  }

  // ---------------------------------------------------------
  // 10. Gravity compensation
  // ---------------------------------------------------------
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

  // ---------------------------------------------------------
  // 11. Apply the complete Cartesian controller
  // tau = g(q) + J^T W + N^T*tau_posture
  // ---------------------------------------------------------
  for (std::size_t i = 0; i < joint_names_.size(); ++i)
  {
    const double commanded_torque =
      kdl_gravity_(i) + cartesian_torque_(i) + nullspace_torque_(i);

    const double limited_torque =
      std::max(
        -torque_limits_[i],
        std::min(
          commanded_torque,
          torque_limits_[i]));

    if (!std::isfinite(limited_torque))
    {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(),
        *get_node()->get_clock(),
        2000,
        "Invalid torque command for joint %s.",
        joint_names_[i].c_str());

      command_interfaces_[i].set_value(0.0);
      continue;
    }

    command_interfaces_[i].set_value(limited_torque);
  }

  // ---------------------------------------------------------
  // 11. Debug output
  // ---------------------------------------------------------
  RCLCPP_INFO_THROTTLE(
    get_node()->get_logger(),
    *get_node()->get_clock(),
    1000,
    "Cartesian position | desired=[%.4f, %.4f, %.4f], "
    "current=[%.4f, %.4f, %.4f], "
    "error=[%.4f, %.4f, %.4f]",
    desired_position_(0),
    desired_position_(1),
    desired_position_(2),
    current_position_(0),
    current_position_(1),
    current_position_(2),
    position_error_(0),
    position_error_(1),
    position_error_(2));

  RCLCPP_INFO_THROTTLE(
    get_node()->get_logger(),
    *get_node()->get_clock(),
    1000,
    "EE velocity | linear=[%.4f, %.4f, %.4f] m/s, "
    "angular=[%.4f, %.4f, %.4f] rad/s",
    cartesian_velocity_[0],
    cartesian_velocity_[1],
    cartesian_velocity_[2],
    cartesian_velocity_[3],
    cartesian_velocity_[4],
    cartesian_velocity_[5]);

  RCLCPP_INFO_THROTTLE(
    get_node()->get_logger(),
    *get_node()->get_clock(),
    1000,
    "Cartesian force: [%.3f, %.3f, %.3f] N",
    cartesian_force_(0),
    cartesian_force_(1),
    cartesian_force_(2));

  RCLCPP_INFO_THROTTLE(
    get_node()->get_logger(),
    *get_node()->get_clock(),
    1000,
    "Cartesian torque: "
    "[%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f] Nm",
    cartesian_torque_(0),
    cartesian_torque_(1),
    cartesian_torque_(2),
    cartesian_torque_(3),
    cartesian_torque_(4),
    cartesian_torque_(5),
    cartesian_torque_(6));

  RCLCPP_INFO_THROTTLE(
    get_node()->get_logger(),
    *get_node()->get_clock(),
    1000,
    "Null space | joint_%zu=%.4f rad, target=%.4f rad, error=%.4f rad, "
    "raw torque=%.3f Nm, projected joint torque=%.3f Nm",
    nullspace_joint_index_ + 1,
    q_[nullspace_joint_index_],
    posture_target,
    posture_error,
    raw_posture_torque(nullspace_joint_index_),
    nullspace_torque_(nullspace_joint_index_));

  std::ostringstream log_stream;
  log_stream << std::fixed << std::setprecision(6);
  log_stream << "\nWAM Cartesian controller state:\n";

  for (std::size_t i = 0; i < joint_names_.size(); ++i)
  {
    const double applied_command =
      command_interfaces_[i].get_value();

    log_stream
      << "[" << i << "] "
      << joint_names_[i]
      << " | q: " << q_[i]
      << " | dq: " << dq_[i]
      << " | gravity: " << kdl_gravity_(i)
      << " | Cartesian: " << cartesian_torque_(i)
      << " | command: " << applied_command
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

}  // namespace wam_cartesian_controller

PLUGINLIB_EXPORT_CLASS(
  wam_cartesian_controller::WamCartesianController,
  controller_interface::ControllerInterface)
