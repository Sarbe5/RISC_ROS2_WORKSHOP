// #pragma once

// #include <array>
// #include <memory>
// #include <string>
// #include <vector>

// #include <controller_interface/controller_interface.hpp>
// #include <rclcpp/rclcpp.hpp>
// #include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
// #include <rclcpp_lifecycle/state.hpp>
// #include <realtime_tools/realtime_buffer.h>

// #include <Eigen/Eigen>

// namespace iitgn_controllers {

// using Vector7d = Eigen::Matrix<double, 7, 1>;
// using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// class ImpedanceController : public controller_interface::ControllerInterface {
// public:
//   // Required interface configuration methods
//   controller_interface::InterfaceConfiguration command_interface_configuration() const override;
//   controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  
//   // Main control loop
//   controller_interface::return_type update(const rclcpp::Time& time,
//                                          const rclcpp::Duration& period) override;

//   // Lifecycle methods
//   CallbackReturn on_init() override;
//   CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
//   CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
//   CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

// private:
//   // Constants
//   static constexpr int num_joints = 7;
  
//   // Robot parameters
//   std::string arm_id_;
//   std::string robot_description_;
  
//   // Impedance control gains
//   Vector7d k_gains_;     // Stiffness gains
//   Vector7d d_gains_;     // Damping gains
//   Vector7d m_gains_;     // Inertia gains (for impedance control)
  
//   // State variables
//   Vector7d q_;           // Current joint positions
//   Vector7d dq_;          // Current joint velocities
//   Vector7d dq_filtered_; // Filtered joint velocities
//   Vector7d initial_q_;   // Initial joint positions
  
//   // Impedance control variables
//   Vector7d q_desired_;   // Desired joint positions
//   Vector7d dq_desired_;  // Desired joint velocities
//   Vector7d ddq_desired_; // Desired joint accelerations
//   Vector7d q_target_;    // Target joint positions
  
//   // Time tracking
//   double elapsed_time_;
  
//   // Control parameters
//   enum ImpedanceMode {
//     POSITION_IMPEDANCE,
//     TRAJECTORY_IMPEDANCE,
//     SINUSOIDAL_IMPEDANCE,
//     COMPLIANT_MOTION
//   };
  
//   ImpedanceMode impedance_mode_;
//   std::vector<double> target_position_;
//   double motion_amplitude_;
//   double motion_frequency_;
//   double compliance_factor_;
  
//   // Helper methods
//   void updateJointStates();
//   void calculateDesiredTrajectory();
//   Vector7d calculateImpedanceTorques();
//   void applyVelocityFilter(double alpha = 0.99);
//   bool validateJointLimits(const Vector7d& q) const;
  
//   // Safety limits (Franka FR3/Panda)
//   static constexpr std::array<double, 7> joint_limits_lower = {
//     -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973
//   };
//   static constexpr std::array<double, 7> joint_limits_upper = {
//     2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973
//   };
//   static constexpr std::array<double, 7> max_torques = {
//     87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0
//   };
// };

// }  // namespace iitgn_controllers



#pragma once

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <eigen3/Eigen/Dense>

namespace iitgn_controllers {

using Vector7d = Eigen::Matrix<double, 7, 1>;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

enum ImpedanceMode {
  POSITION_IMPEDANCE,
  TRAJECTORY_IMPEDANCE,
  SINUSOIDAL_IMPEDANCE,
  COMPLIANT_MOTION
};

class ImpedanceController : public controller_interface::ControllerInterface {
public:
  ImpedanceController() = default;
  
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  
  controller_interface::return_type update(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;
  
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

private:
  // Robot parameters
  static constexpr int num_joints = 7;
  std::string arm_id_;
  std::string robot_description_;
  
  // Control gains
  Vector7d k_gains_;
  Vector7d d_gains_;
  Vector7d m_gains_;
  
  // State variables
  Vector7d q_;                    // Current joint positions
  Vector7d dq_;                   // Current joint velocities
  Vector7d dq_filtered_;          // Filtered joint velocities
  Vector7d q_desired_;            // Desired joint positions
  Vector7d dq_desired_;           // Desired joint velocities
  Vector7d ddq_desired_;          // Desired joint accelerations
  Vector7d q_target_;             // Target joint positions
  Vector7d initial_q_;            // Initial joint positions
  
  
  // External command variables
  Vector7d external_q_target_;    // External command target positions
  Vector7d external_dq_target_;   // External command target velocities
  Vector7d external_ddq_target_;  // External command target accelerations
  Vector7d trajectory_start_q_;   // Starting position for trajectory
  Vector7d trajectory_start_dq_;   // Starting velocity for trajectory
  
  // Command state
  bool new_command_received_;     // Flag for new external command
  bool external_command_active_;  // Flag for active external command
  bool direct_tracking_mode_;     // Flag for direct tracking mode
  geometry_msgs::msg::PoseStamped commanded_pose_;  // Latest commanded pose
  
  // Impedance parameters
  ImpedanceMode impedance_mode_;
  double motion_amplitude_;
  double motion_frequency_;
  double compliance_factor_;
  double elapsed_time_;
  double trajectory_duration_;
  double external_trajectory_time_;
  double elapsed_time_step_;
  

  // Parameters from config
  std::vector<double> target_position_;
  std::string command_topic_;
  
  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr command_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_command_subscriber_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr impedance_config_subscriber_;
  
  // Safety limits (you'll need to define these based on your robot)
  static constexpr std::array<double, num_joints> max_torques = {87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0};
  static constexpr std::array<double, num_joints> joint_limits_lower = {-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973};
  static constexpr std::array<double, num_joints> joint_limits_upper = {2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973};
  
  // Methods
  void updateJointStates();
  void calculateDesiredTrajectory();
  Vector7d calculateImpedanceTorques();
  void applyVelocityFilter(double alpha = 0.1);
  bool validateJointLimits(const Vector7d& q) const;
  
  // Callback methods
  void commandCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void jointCommandCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void impedanceConfigCallback(const std_msgs::msg::String::SharedPtr msg);
  void processExternalCommand();
};

}  // namespace iitgn_controllers