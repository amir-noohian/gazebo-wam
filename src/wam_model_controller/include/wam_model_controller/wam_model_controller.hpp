#ifndef WAM_MODEL_CONTROLLER__WAM_MODEL_CONTROLLER_HPP_
#define WAM_MODEL_CONTROLLER__WAM_MODEL_CONTROLLER_HPP_

#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include <string>
#include <vector>

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

  std::vector<double> kp_;
  std::vector<double> kd_;

  std::vector<double> torque_limits_;

  bool hold_current_position_;
};

}  // namespace wam_model_controller

#endif  // WAM_MODEL_CONTROLLER__WAM_MODEL_CONTROLLER_HPP_