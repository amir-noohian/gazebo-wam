#ifndef WAM_MODEL_CONTROLLER__WAM_MODEL_CONTROLLER_HPP_
#define WAM_MODEL_CONTROLLER__WAM_MODEL_CONTROLLER_HPP_


#include <memory>
#include <string>
#include <vector>

#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_publisher.h"
#include "std_msgs/msg/bool.hpp"

namespace wam_model_controller
{

class WamModelController
  : public controller_interface::ControllerInterface
{
public:
  WamModelController() = default;

  controller_interface::return_type init(
    const std::string & controller_name) override;

  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update() override;

private:
  std::vector<std::string> joint_names_;

  std::vector<double> q_;
  std::vector<double> dq_;
  std::vector<double> q_des_;
  std::vector<double> trajectory_start_;
  std::vector<double> trajectory_target_;
  std::vector<double> trajectory_direction_;
  std::vector<double> dq_des_;

  std::vector<double> kp_;
  std::vector<double> kd_;

  std::vector<double> torque_limits_;

  bool hold_current_position_;

  // Straight joint-space path with a trapezoidal scalar velocity profile.
  double trajectory_velocity_{0.3};
  double trajectory_acceleration_{0.3};
  double trajectory_path_length_{0.0};
  double trajectory_peak_velocity_{0.0};
  double trajectory_accel_time_{0.0};
  double trajectory_cruise_time_{0.0};
  double trajectory_duration_{0.0};
  rclcpp::Time trajectory_start_time_;
  bool trajectory_active_{false};
  bool completion_pending_{false};
  std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Bool>>
    completion_publisher_;

  // KDL robot model
  KDL::Tree kdl_tree_;
  KDL::Chain kdl_chain_;

  std::unique_ptr<KDL::ChainDynParam> kdl_dynamics_solver_;

  KDL::JntArray kdl_q_;
  KDL::JntArray kdl_gravity_;

  // KDL chain endpoints
  std::string root_link_;
  std::string tip_link_;

  // True after the KDL model has been created successfully
  bool kdl_initialized_{false};

};

}  // namespace wam_model_controller

#endif  // WAM_MODEL_CONTROLLER__WAM_MODEL_CONTROLLER_HPP_
