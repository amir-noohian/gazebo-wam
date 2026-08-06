#ifndef WAM_CARTESIAN_CONTROLLER__WAM_CARTESIAN_CONTROLLER_HPP_
#define WAM_CARTESIAN_CONTROLLER__WAM_CARTESIAN_CONTROLLER_HPP_


#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <array>
#include <Eigen/Dense>

#include <geometry_msgs/msg/point_stamped.hpp>

#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>
#include <kdl/tree.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jacobian.hpp>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace wam_cartesian_controller
{

class WamCartesianController
  : public controller_interface::ControllerInterface
{
public:
  WamCartesianController() = default;

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
  void target_callback(const geometry_msgs::msg::PointStamped::SharedPtr message);

  std::vector<std::string> joint_names_;

  std::vector<double> q_;
  std::vector<double> dq_;
  std::vector<double> q_des_;

  std::vector<double> kp_;
  std::vector<double> kd_;

  std::vector<double> torque_limits_;

  bool hold_current_position_;

  // KDL robot model
  KDL::Tree kdl_tree_;
  KDL::Chain kdl_chain_;

  std::unique_ptr<KDL::ChainDynParam> kdl_dynamics_solver_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> kdl_fk_solver_;
  std::unique_ptr<KDL::ChainJntToJacSolver> kdl_jacobian_solver_;

  KDL::JntArray kdl_q_;
  KDL::JntArray kdl_gravity_;
  KDL::JntSpaceInertiaMatrix kdl_mass_matrix_;

  KDL::Frame end_effector_pose_;
  KDL::Jacobian kdl_jacobian_;

  Eigen::Vector3d desired_position_;
  Eigen::Vector3d current_position_;
  Eigen::Vector3d position_error_;

  Eigen::Vector3d cartesian_kp_;
  Eigen::Vector3d cartesian_kd_;
  Eigen::Vector3d linear_velocity_;
  Eigen::Vector3d cartesian_force_;

  Eigen::Matrix<double, 6, 1> cartesian_wrench_;
  Eigen::Matrix<double, 6, 7> jacobian_eigen_;
  Eigen::Matrix<double, 7, 1> cartesian_torque_;
  Eigen::Matrix<double, 7, 1> nullspace_torque_;
  Eigen::Matrix<double, 7, 7> mass_matrix_eigen_;

  std::vector<std::size_t> position_state_indices_;
  std::vector<std::size_t> velocity_state_indices_;

  std::array<double, 6> cartesian_velocity_;

  // KDL chain endpoints
  std::string root_link_;
  std::string tip_link_;

  // True after the KDL model has been created successfully
  bool kdl_initialized_{false};

  Eigen::Vector3d trajectory_start_position_;
  Eigen::Vector3d trajectory_target_position_;
  Eigen::Vector3d desired_linear_velocity_;

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_subscription_;
  std::mutex target_mutex_;
  Eigen::Vector3d pending_target_position_;
  bool target_pending_{false};

  // Secondary posture task for WAM joint 6 (zero-based index 5).
  static constexpr std::size_t nullspace_joint_index_ = 5;
  double nullspace_target_{-0.7853981633974483};
  double nullspace_kp_{5.0};
  double nullspace_kd_{1.0};
  double nullspace_damping_{0.01};
  double nullspace_max_torque_{0.2};

  rclcpp::Time trajectory_start_time_;

  double trajectory_velocity_{0.05};
  double trajectory_acceleration_{0.05};
  double trajectory_path_length_{0.0};
  double trajectory_peak_velocity_{0.0};
  double trajectory_accel_time_{0.0};
  double trajectory_cruise_time_{0.0};
  double trajectory_duration_{0.0};

  bool trajectory_initialized_{false};
  bool trajectory_active_{false};

};

}  // namespace wam_cartesian_controller

#endif  // WAM_CARTESIAN_CONTROLLER__WAM_CARTESIAN_CONTROLLER_HPP_
