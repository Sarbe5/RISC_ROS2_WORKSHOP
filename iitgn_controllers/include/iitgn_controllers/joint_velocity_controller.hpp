#pragma once

#include <string>
#include <array>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace iitgn_controllers {

class JointVelocityController : public controller_interface::ControllerInterface {
 public:
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  void velocity_command_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

  std::string arm_id_ = "fr3";
  static constexpr size_t num_joints = 7;
  std::array<double, num_joints> desired_velocities_{};
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr velocity_sub_;
};

}  // namespace iitgn_controllers
