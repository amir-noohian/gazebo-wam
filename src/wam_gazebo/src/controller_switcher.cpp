#include <chrono>
#include <memory>

#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

class ControllerSwitcher : public rclcpp::Node
{
public:
  ControllerSwitcher()
  : Node("wam_controller_switcher")
  {
    switch_client_ = create_client<controller_manager_msgs::srv::SwitchController>(
      "/controller_manager/switch_controller");

    completion_subscription_ = create_subscription<std_msgs::msg::Bool>(
      "/wam_model_controller/trajectory_complete",
      rclcpp::SystemDefaultsQoS(),
      [this](const std_msgs::msg::Bool::SharedPtr message)
      {
        if (message->data) {
          request_switch();
        }
      });

    RCLCPP_INFO(
      get_logger(),
      "Waiting for the joint trajectory completion event.");
  }

private:
  void request_switch()
  {
    if (switch_requested_) {
      return;
    }

    if (!switch_client_->service_is_ready()) {
      RCLCPP_ERROR(
        get_logger(),
        "Controller manager switch service is not ready; automatic switch aborted.");
      return;
    }

    switch_requested_ = true;
    auto request =
      std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    request->start_controllers = {"wam_cartesian_controller"};
    request->stop_controllers = {"wam_model_controller"};
    request->strictness =
      controller_manager_msgs::srv::SwitchController::Request::STRICT;
    request->start_asap = true;
    request->timeout.sec = 2;
    request->timeout.nanosec = 0;

    RCLCPP_INFO(
      get_logger(),
      "Joint trajectory completed; switching to Cartesian control.");

    switch_client_->async_send_request(
      request,
      [this](rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future)
      {
        if (future.get()->ok) {
          RCLCPP_INFO(
            get_logger(),
            "Automatic controller switch succeeded; Cartesian targets are now accepted.");
        } else {
          RCLCPP_ERROR(get_logger(), "Automatic controller switch failed.");
          switch_requested_ = false;
        }
      });
  }

  bool switch_requested_{false};
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_client_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr completion_subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerSwitcher>());
  rclcpp::shutdown();
  return 0;
}
