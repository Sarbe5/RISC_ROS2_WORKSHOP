
#include <iitgn_controllers/impedance_controller.hpp>
#include <iitgn_controllers/robot_utils.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

namespace iitgn_controllers {

controller_interface::InterfaceConfiguration
ImpedanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // Request effort interfaces for all joints
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
ImpedanceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  
  // Request position and velocity interfaces for all joints
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

controller_interface::return_type ImpedanceController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& period) {
  
  // 1. Update current joint states
  updateJointStates();
  
  // 2. Update time
  elapsed_time_ += period.seconds();
  
  // 3. Calculate desired trajectory based on impedance mode
  calculateDesiredTrajectory();
  
  // 4. Apply velocity filtering
  applyVelocityFilter();
  
  // 5. Calculate impedance control torques
  Vector7d tau_d = calculateImpedanceTorques();
  
  // 6. Apply safety limits
  for (int i = 0; i < num_joints; ++i) {
    // Clamp torques to maximum limits
    tau_d(i) = std::clamp(tau_d(i), -max_torques[i], max_torques[i]);
    
    // Send commands to robot
    command_interfaces_[i].set_value(tau_d(i));
  }
  
  return controller_interface::return_type::OK;
}

CallbackReturn ImpedanceController::on_init() {
  try {
    // Declare parameters
    auto_declare<std::string>("arm_id", "fr3");
    auto_declare<std::vector<double>>("k_gains", {});
    auto_declare<std::vector<double>>("d_gains", {});
    auto_declare<std::vector<double>>("m_gains", {});
    auto_declare<std::vector<double>>("target_position", {});
    auto_declare<std::string>("impedance_mode", "position_impedance");
    auto_declare<double>("motion_amplitude", 0.5);
    auto_declare<double>("motion_frequency", 0.2);
    auto_declare<double>("compliance_factor", 1.0);
    auto_declare<std::string>("command_topic", "/impedance_command");
    auto_declare<double>("trajectory_duration", 5.0);
    
    RCLCPP_INFO(get_node()->get_logger(), "IITGN Impedance Controller initialized");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during init: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn ImpedanceController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  
  try {
    // Get parameters
    arm_id_ = get_node()->get_parameter("arm_id").as_string();
    auto k_gains = get_node()->get_parameter("k_gains").as_double_array();
    auto d_gains = get_node()->get_parameter("d_gains").as_double_array();
    auto m_gains = get_node()->get_parameter("m_gains").as_double_array();
    target_position_ = get_node()->get_parameter("target_position").as_double_array();
    std::string mode_str = get_node()->get_parameter("impedance_mode").as_string();
    motion_amplitude_ = get_node()->get_parameter("motion_amplitude").as_double();
    motion_frequency_ = get_node()->get_parameter("motion_frequency").as_double();
    compliance_factor_ = get_node()->get_parameter("compliance_factor").as_double();
    command_topic_ = get_node()->get_parameter("command_topic").as_string();
    trajectory_duration_ = get_node()->get_parameter("trajectory_duration").as_double();
    
    
    // Validate gains
    if (k_gains.size() != num_joints) {
      RCLCPP_ERROR(get_node()->get_logger(), "k_gains must have %d elements", num_joints);
      return CallbackReturn::ERROR;
    }
    if (d_gains.size() != num_joints) {
      RCLCPP_ERROR(get_node()->get_logger(), "d_gains must have %d elements", num_joints);
      return CallbackReturn::ERROR;
    }
    if (m_gains.size() != num_joints) {
      RCLCPP_ERROR(get_node()->get_logger(), "m_gains must have %d elements", num_joints);
      return CallbackReturn::ERROR;
    }
    
    // Set gains
    for (int i = 0; i < num_joints; ++i) {
      k_gains_(i) = k_gains[i];
      d_gains_(i) = d_gains[i];
      m_gains_(i) = m_gains[i];
    }
    
    // Set impedance mode
    if (mode_str == "position_impedance") {
      impedance_mode_ = POSITION_IMPEDANCE;
    } else if (mode_str == "trajectory_impedance") {
      impedance_mode_ = TRAJECTORY_IMPEDANCE;
    } else if (mode_str == "sinusoidal_impedance") {
      impedance_mode_ = SINUSOIDAL_IMPEDANCE;
    } else if (mode_str == "compliant_motion") {
      impedance_mode_ = COMPLIANT_MOTION;
    } else {
      RCLCPP_WARN(get_node()->get_logger(), "Unknown impedance mode, using position_impedance");
      impedance_mode_ = POSITION_IMPEDANCE;
    }
    
    // Initialize state variables
    q_.setZero();
    dq_.setZero();
    dq_filtered_.setZero();
    q_desired_.setZero();
    dq_desired_.setZero();
    ddq_desired_.setZero();
    q_target_.setZero();
    
    // Initialize command flags
    new_command_received_ = false;
    external_command_active_ = false;
    direct_tracking_mode_ = false;  // Default to smooth trajectory mode
    
    // Create subscriber for external commands
    command_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
        command_topic_, 10, 
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            this->commandCallback(msg);
        });
    
    // Alternative: Subscribe to joint position commands
    joint_command_subscriber_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
        command_topic_ + "_joint", 10,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
            this->jointCommandCallback(msg);
        });
    
    // Create subscriber for impedance parameter updates
    impedance_config_subscriber_ = get_node()->create_subscription<std_msgs::msg::String>(
        command_topic_ + "_config", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            this->impedanceConfigCallback(msg);
        });
    
    // Get robot description
    auto parameters_client = 
        std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "robot_state_publisher");
    parameters_client->wait_for_service();
    
    auto future = parameters_client->get_parameters({"robot_description"});
    auto result = future.get();
    if (!result.empty()) {
      robot_description_ = result[0].value_to_string();
    } else {
      RCLCPP_WARN(get_node()->get_logger(), "Could not get robot_description");
    }
    
    RCLCPP_INFO(get_node()->get_logger(), "IITGN Impedance Controller configured successfully");
    RCLCPP_INFO(get_node()->get_logger(), "Impedance mode: %s", mode_str.c_str());
    RCLCPP_INFO(get_node()->get_logger(), "Command topic: %s", command_topic_.c_str());
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during configuration: %s", e.what());
    return CallbackReturn::ERROR;
  }
  
  return CallbackReturn::SUCCESS;
}

CallbackReturn ImpedanceController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  
  // Read initial joint states
  updateJointStates();
  
  // Store initial position
  initial_q_ = q_;
  
  // Set target position
  if (target_position_.size() == num_joints) {
    for (int i = 0; i < num_joints; ++i) {
      q_target_(i) = target_position_[i];
    }
  } else {
    q_target_ = initial_q_;  // Default to current position
  }
  
  // Reset time and filters
  elapsed_time_ = 0.0;
  dq_filtered_.setZero();
  dq_desired_.setZero();
  ddq_desired_.setZero();

  external_trajectory_time_ = 0.0;
  trajectory_start_dq_.setZero();
  
  // Reset command flags
  new_command_received_ = false;
  external_command_active_ = false;
  
  RCLCPP_INFO(get_node()->get_logger(), "IITGN Impedance Controller activated");
  return CallbackReturn::SUCCESS;
}

CallbackReturn ImpedanceController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(get_node()->get_logger(), "IITGN Impedance Controller deactivated");
  return CallbackReturn::SUCCESS;
}

void ImpedanceController::updateJointStates() {
  for (int i = 0; i < num_joints; ++i) {
    const auto& position_interface = state_interfaces_.at(2 * i);
    const auto& velocity_interface = state_interfaces_.at(2 * i + 1);
    
    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");
    
    q_(i) = position_interface.get_value();
    dq_(i) = velocity_interface.get_value();
  }
}

void ImpedanceController::calculateDesiredTrajectory() {
  // Check if we have a new external command
  if (new_command_received_) {
    processExternalCommand();
    new_command_received_ = false;
  }
  
  // If external command is active
  if (external_command_active_) {
    
    // DIRECT TRACKING MODE - for sinusoidal and real-time profiles
    if (direct_tracking_mode_) {
      // Directly use the commanded values without smoothing
      q_desired_ = external_q_target_;
      dq_desired_ = external_dq_target_;
      ddq_desired_ = external_ddq_target_;
      
      // Optional: Add a small low-pass filter to reduce noise
      double alpha = 0.1;  // Adjust for desired filtering (0 = no filter, 1 = no smoothing)
      static Vector7d q_desired_filtered = q_desired_;
      q_desired_filtered = (1 - alpha) * q_desired_filtered + alpha * q_desired_;
      q_desired_ = q_desired_filtered;
      
      return;
    }
    
    // SMOOTH TRAJECTORY MODE - for point-to-point movements
    else {
      // Generate smooth trajectory to external target
      double t_normalized = std::min(elapsed_time_ / trajectory_duration_, 1.0);
      double s = 3 * t_normalized * t_normalized - 2 * t_normalized * t_normalized * t_normalized;
      double ds_dt = (6 * t_normalized - 6 * t_normalized * t_normalized) / trajectory_duration_;
      double d2s_dt2 = (6 - 12 * t_normalized) / (trajectory_duration_ * trajectory_duration_);
      
      Vector7d position_diff = external_q_target_ - trajectory_start_q_;
      // Vector7d velocity_diff = external_dq_target_ - trajectory_start_dq_;
      
      // Smooth position trajectory
      q_desired_ = trajectory_start_q_ + s * position_diff;
      dq_desired_ = trajectory_start_dq_ + ds_dt * position_diff;
      ddq_desired_ = d2s_dt2 * position_diff;

      // Update trajectory time
      external_trajectory_time_ += elapsed_time_step_;
      
      // Check if we've reached the target (within tolerance)
      Vector7d position_error = external_q_target_ - q_;
      if (position_error.norm() < 0.01 && t_normalized >= 1.0) {
        external_command_active_ = false;
        external_trajectory_time_ = 0.0;
        RCLCPP_INFO(get_node()->get_logger(), "External command target reached");
      }
      return;
    }
  }
  
  // Original trajectory calculation
  switch (impedance_mode_) {
    case POSITION_IMPEDANCE:
      // Hold target position with impedance
      q_desired_ = q_target_;
      dq_desired_.setZero();
      ddq_desired_.setZero();
      break;
      
    case TRAJECTORY_IMPEDANCE:
      // Smooth trajectory to target position with impedance
      {
        double t_normalized = std::min(elapsed_time_ / 5.0, 1.0);  // 5 second trajectory
        double s = 3 * t_normalized * t_normalized - 2 * t_normalized * t_normalized * t_normalized;
        double ds_dt = (6 * t_normalized - 6 * t_normalized * t_normalized) / 5.0;
        double d2s_dt2 = (6 - 12 * t_normalized) / 25.0;
        
        Vector7d position_diff = q_target_ - initial_q_;
        q_desired_ = initial_q_ + s * position_diff;
        dq_desired_ = ds_dt * position_diff;
        ddq_desired_ = d2s_dt2 * position_diff;
      }
      break;
      
    case SINUSOIDAL_IMPEDANCE:
      // Sinusoidal motion with impedance control
      {
        double omega = 2 * M_PI * motion_frequency_;
        double phase = omega * elapsed_time_;
        double delta_angle = motion_amplitude_ * (1 - std::cos(phase));
        double delta_velocity = motion_amplitude_ * omega * std::sin(phase);
        double delta_acceleration = motion_amplitude_ * omega * omega * std::cos(phase);
        
        q_desired_ = initial_q_;
        dq_desired_.setZero();
        ddq_desired_.setZero();
        
        // Apply to joints 4 and 5
        q_desired_(3) += delta_angle;
        q_desired_(4) += delta_angle;
        dq_desired_(3) += delta_velocity;
        dq_desired_(4) += delta_velocity;
        ddq_desired_(3) += delta_acceleration;
        ddq_desired_(4) += delta_acceleration;
      }
      break;
      
    case COMPLIANT_MOTION:
      // Compliant motion - reduces stiffness for external interactions
      {
        Vector7d compliance_offset = compliance_factor_ * 0.1 * (dq_filtered_);
        q_desired_ = initial_q_ + compliance_offset;
        dq_desired_ = compliance_factor_ * dq_filtered_;
        ddq_desired_.setZero();
      }
      break;
  }
  
  // Validate joint limits
  if (!validateJointLimits(q_desired_)) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                         "Desired position exceeds joint limits");
  }
}

void ImpedanceController::commandCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  RCLCPP_INFO(get_node()->get_logger(), "Received pose command");
  
  // Store the commanded pose
  commanded_pose_ = *msg;
  
  // TODO: Convert pose to joint space using inverse kinematics
  // For now, we'll just set a flag that a new command was received
  new_command_received_ = true;
  
  // You would typically use a kinematics solver here:
  // bool success = kinematics_solver_->getPositionIK(msg->pose, q_, external_q_target_);
  // if (success) {
  //   external_command_active_ = true;
  //   external_dq_target_.setZero();
  //   external_ddq_target_.setZero();
  // }
}

void ImpedanceController::jointCommandCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  RCLCPP_INFO(get_node()->get_logger(), "Received joint command");
  
  if (msg->position.size() != num_joints) {
    RCLCPP_WARN(get_node()->get_logger(), "Joint command size mismatch: expected %d, got %zu", 
                num_joints, msg->position.size());
    return;
  }
  
  // Store the joint command
  for (int i = 0; i < num_joints; ++i) {
    external_q_target_(i) = msg->position[i];
  }
  
  // Set velocities if provided
  if (msg->velocity.size() == num_joints) {
    for (int i = 0; i < num_joints; ++i) {
      external_dq_target_(i) = msg->velocity[i];
    }
  } else {
    external_dq_target_.setZero();
  }
  
  // Set accelerations if provided
  if (msg->effort.size() == num_joints) {  // Using effort field for accelerations
    for (int i = 0; i < num_joints; ++i) {
      external_ddq_target_(i) = msg->effort[i];
    }
  } else {
    external_ddq_target_.setZero();
  }
  
  // Check if this is a direct tracking command (look for a specific frame_id or name)
  if (!msg->name.empty() && msg->name[0] == "direct_track") {
    direct_tracking_mode_ = true;
    RCLCPP_INFO(get_node()->get_logger(), "Direct tracking mode enabled");
  } else {
    direct_tracking_mode_ = false;
  }
  
  // Validate joint limits
  if (validateJointLimits(external_q_target_)) {
    external_command_active_ = true;
    new_command_received_ = true;
    RCLCPP_INFO(get_node()->get_logger(), "Joint command accepted");
  } else {
    RCLCPP_WARN(get_node()->get_logger(), "Joint command rejected: exceeds limits");
  }
}

void ImpedanceController::impedanceConfigCallback(const std_msgs::msg::String::SharedPtr msg) {
  RCLCPP_INFO(get_node()->get_logger(), "Received impedance config: %s", msg->data.c_str());
  
  std::string command = msg->data;
  
  if (command == "stop") {
    external_command_active_ = false;
    direct_tracking_mode_ = false;
    q_target_ = q_;
    RCLCPP_INFO(get_node()->get_logger(), "Emergency stop activated");
  }
  else if (command == "home") {
    external_q_target_ = initial_q_;
    external_dq_target_.setZero();
    external_ddq_target_.setZero();
    external_command_active_ = true;
    direct_tracking_mode_ = false;  // Use smooth trajectory for homing
    new_command_received_ = true;
    RCLCPP_INFO(get_node()->get_logger(), "Homing command activated");
  }
  else if (command == "direct_tracking_on") {
    direct_tracking_mode_ = true;
    RCLCPP_INFO(get_node()->get_logger(), "Direct tracking mode enabled");
  }
  else if (command == "direct_tracking_off") {
    direct_tracking_mode_ = false;
    RCLCPP_INFO(get_node()->get_logger(), "Direct tracking mode disabled");
  }
  else if (command.find("trajectory_duration:") == 0) {
    try {
      double new_duration = std::stod(command.substr(19));
      trajectory_duration_ = std::max(0.1, new_duration);  // Minimum 0.1 seconds
      RCLCPP_INFO(get_node()->get_logger(), "Trajectory duration changed to: %.2f seconds", trajectory_duration_);
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_node()->get_logger(), "Invalid trajectory duration: %s", e.what());
    }
  }
  
  // Parse simple commands
  else if (command == "stop") {
    external_command_active_ = false;
    q_target_ = q_;  // Hold current position
    RCLCPP_INFO(get_node()->get_logger(), "Emergency stop activated");
  }
  else if (command == "home") {
    external_q_target_ = initial_q_;
    external_dq_target_.setZero();
    external_ddq_target_.setZero();
    external_command_active_ = true;
    new_command_received_ = true;
    RCLCPP_INFO(get_node()->get_logger(), "Homing command activated");
  }
  else if (command.find("mode:") == 0) {
    std::string mode_str = command.substr(5);
    if (mode_str == "position_impedance") {
      impedance_mode_ = POSITION_IMPEDANCE;
    } else if (mode_str == "trajectory_impedance") {
      impedance_mode_ = TRAJECTORY_IMPEDANCE;
    } else if (mode_str == "sinusoidal_impedance") {
      impedance_mode_ = SINUSOIDAL_IMPEDANCE;
    } else if (mode_str == "compliant_motion") {
      impedance_mode_ = COMPLIANT_MOTION;
    }
    RCLCPP_INFO(get_node()->get_logger(), "Impedance mode changed to: %s", mode_str.c_str());
  }
  else if (command.find("compliance:") == 0) {
    try {
      double new_compliance = std::stod(command.substr(11));
      compliance_factor_ = std::clamp(new_compliance, 0.0, 2.0);
      RCLCPP_INFO(get_node()->get_logger(), "Compliance factor changed to: %.2f", compliance_factor_);
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_node()->get_logger(), "Invalid compliance value: %s", e.what());
    }
  }
}

void ImpedanceController::processExternalCommand() {
  // This function processes the external command
  // You can add custom logic here for how to handle external commands
  
  // For example, you might want to create a smooth trajectory to the target
  if (external_command_active_) {
    // Reset trajectory timer for smooth motion
    elapsed_time_ = 0.0;
    
    // Store current position as starting point for trajectory
    trajectory_start_q_ = q_;
    
    RCLCPP_INFO(get_node()->get_logger(), "Processing external command - generating trajectory");
  }
}

Vector7d ImpedanceController::calculateImpedanceTorques() {
  // Position error
  Vector7d position_error = q_desired_ - q_;
  
  // Velocity error
  Vector7d velocity_error = dq_desired_ - dq_filtered_;
  
  // Impedance control law: M*ddq_desired + D*(dq_desired - dq) + K*(q_desired - q)
  Vector7d tau_d = 
                   d_gains_.cwiseProduct(velocity_error) +
                   k_gains_.cwiseProduct(position_error);
  
  return tau_d;
}

void ImpedanceController::applyVelocityFilter(double alpha) {
  dq_filtered_ = (1 - alpha) * dq_filtered_ + alpha * dq_;
}

bool ImpedanceController::validateJointLimits(const Vector7d& q) const {
  for (int i = 0; i < num_joints; ++i) {
    if (q(i) < joint_limits_lower[i] || q(i) > joint_limits_upper[i]) {
      return false;
    }
  }
  return true;
}

}  // namespace iitgn_controllers

// Register the controller as a plugin
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(iitgn_controllers::ImpedanceController,
                       controller_interface::ControllerInterface)





                       