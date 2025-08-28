#include <iitgn_controllers/joint_velocity_controller.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <string>

namespace iitgn_controllers {

controller_interface::InterfaceConfiguration
JointVelocityController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

controller_interface::InterfaceConfiguration
JointVelocityController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

controller_interface::return_type
JointVelocityController::update(const rclcpp::Time&, const rclcpp::Duration&) {
  for (size_t i = 0; i < num_joints; ++i) {
    command_interfaces_[i].set_value(desired_velocities_[i]);
  }
  return controller_interface::return_type::OK;
}

CallbackReturn JointVelocityController::on_init() {
  try {
    auto node = get_node();
    velocity_sub_ = node->create_subscription<sensor_msgs::msg::JointState>(
      "joint_velocity_command", 10,
      std::bind(&JointVelocityController::velocity_command_callback, this, std::placeholders::_1));
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception in on_init: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointVelocityController::on_configure(const rclcpp_lifecycle::State&) {
  // Could add param parsing or other setup here
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointVelocityController::on_activate(const rclcpp_lifecycle::State&) {
  return CallbackReturn::SUCCESS;
}

void JointVelocityController::velocity_command_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  const double alpha = 0.1;  // Smoothing factor (0 < alpha <= 1)

  for (size_t i = 0; i < std::min(msg->velocity.size(), desired_velocities_.size()); ++i) {
    // Apply exponential smoothing
    desired_velocities_[i] = alpha * msg->velocity[i] + (1 - alpha) * desired_velocities_[i];
  }
}

}  // namespace iitgn_controllers

// Plugin registration
PLUGINLIB_EXPORT_CLASS(iitgn_controllers::JointVelocityController, controller_interface::ControllerInterface)
