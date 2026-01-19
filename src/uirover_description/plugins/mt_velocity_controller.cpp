#include "mt_velocity_controller.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"

namespace mt_velocity_controller
{

MTVelocityController::MTVelocityController()
: controller_interface::ControllerInterface(),
  previous_linear_velocity_(0.0),
  previous_angular_velocity_(0.0)
{
}

controller_interface::CallbackReturn MTVelocityController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("left_wheel_names", std::vector<std::string>());
    auto_declare<std::vector<std::string>>("right_wheel_names", std::vector<std::string>());
    
    auto_declare<double>("wheel_separation", 0.0);
    auto_declare<double>("wheel_radius", 0.0);
    auto_declare<double>("wheel_separation_multiplier", 1.0);
    auto_declare<double>("left_wheel_radius_multiplier", 1.0);
    auto_declare<double>("right_wheel_radius_multiplier", 1.0);
    
    auto_declare<double>("cmd_vel_timeout", 0.5);
    auto_declare<std::string>("command_topic", "cmd_vel");
    auto_declare<bool>("use_stamped_vel", false);
    auto_declare<bool>("open_loop", false);
    
    // Linear limits
    auto_declare<bool>("linear.x.has_velocity_limits", false);
    auto_declare<double>("linear.x.max_velocity", 0.0);
    auto_declare<double>("linear.x.min_velocity", 0.0);
    auto_declare<bool>("linear.x.has_acceleration_limits", false);
    auto_declare<double>("linear.x.max_acceleration", 0.0);
    auto_declare<double>("linear.x.min_acceleration", 0.0);
    auto_declare<bool>("linear.x.has_jerk_limits", false);
    auto_declare<double>("linear.x.max_jerk", 0.0);
    
    // Angular limits
    auto_declare<bool>("angular.z.has_velocity_limits", false);
    auto_declare<double>("angular.z.max_velocity", 0.0);
    auto_declare<double>("angular.z.min_velocity", 0.0);
    auto_declare<bool>("angular.z.has_acceleration_limits", false);
    auto_declare<double>("angular.z.max_acceleration", 0.0);
    auto_declare<double>("angular.z.min_acceleration", 0.0);
    auto_declare<bool>("angular.z.has_jerk_limits", false);
    auto_declare<double>("angular.z.max_jerk", 0.0);
    
    auto_declare<bool>("enable_wheel_slip_compensation", false);
    auto_declare<double>("slip_factor", 0.0);
    auto_declare<double>("publish_rate", 50.0);
    auto_declare<bool>("enable_safety_stop", true);
    auto_declare<double>("safety_stop_timeout", 1.0);
    auto_declare<bool>("publish_wheel_velocities", true);
    auto_declare<bool>("verbose", false);
  }
  catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MTVelocityController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!load_parameters()) {
    return controller_interface::CallbackReturn::ERROR;
  }

  // Compute wheel parameters with multipliers
  wheel_separation_computed_ = wheel_separation_ * wheel_separation_multiplier_;
  left_wheel_radius_computed_ = wheel_radius_ * left_wheel_radius_multiplier_;
  right_wheel_radius_computed_ = wheel_radius_ * right_wheel_radius_multiplier_;

  // Setup subscribers
  if (use_stamped_vel_) {
    twist_stamped_sub_ = get_node()->create_subscription<geometry_msgs::msg::TwistStamped>(
      command_topic_, rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<geometry_msgs::msg::TwistStamped> msg) {
        twist_stamped_callback(msg);
      });
  } else {
    twist_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
      command_topic_, rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<geometry_msgs::msg::Twist> msg) {
        twist_callback(msg);
      });
  }

  // Setup wheel velocities publisher
  if (publish_wheel_velocities_) {
    wheel_velocities_publisher_ = 
      std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(
        get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
          "~/wheel_velocities", rclcpp::SystemDefaultsQoS()));
  }

  // Initialize command buffer
  received_velocity_msg_ptr_.writeFromNonRT(std::shared_ptr<geometry_msgs::msg::Twist>());

  RCLCPP_INFO(get_node()->get_logger(), "MT Velocity Controller configured successfully");
  
  if (verbose_) {
    RCLCPP_INFO(get_node()->get_logger(), "  Wheel separation: %.3f m", wheel_separation_computed_);
    RCLCPP_INFO(get_node()->get_logger(), "  Left wheel radius: %.3f m", left_wheel_radius_computed_);
    RCLCPP_INFO(get_node()->get_logger(), "  Right wheel radius: %.3f m", right_wheel_radius_computed_);
    RCLCPP_INFO(get_node()->get_logger(), "  Command topic: %s", command_topic_.c_str());
    RCLCPP_INFO(get_node()->get_logger(), "  Open loop: %s", open_loop_ ? "true" : "false");
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MTVelocityController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Verify we have the expected number of command interfaces
  const size_t expected_interfaces = left_wheel_names_.size() + right_wheel_names_.size();
  if (command_interfaces_.size() != expected_interfaces) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Expected %zu command interfaces, got %zu",
      expected_interfaces, command_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Reset command and time
  reset_command();
  last_command_time_ = std::chrono::steady_clock::now();
  previous_linear_velocity_ = 0.0;
  previous_angular_velocity_ = 0.0;

  RCLCPP_INFO(get_node()->get_logger(), "MT Velocity Controller activated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MTVelocityController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  reset_command();
  RCLCPP_INFO(get_node()->get_logger(), "MT Velocity Controller deactivated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration 
MTVelocityController::command_interface_configuration() const
{
  std::vector<std::string> conf_names;
  
  for (const auto & joint_name : left_wheel_names_) {
    conf_names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  for (const auto & joint_name : right_wheel_names_) {
    conf_names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  
  return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::InterfaceConfiguration 
MTVelocityController::state_interface_configuration() const
{
  if (open_loop_) {
    return {controller_interface::interface_configuration_type::NONE};
  }
  
  std::vector<std::string> conf_names;
  
  for (const auto & joint_name : left_wheel_names_) {
    conf_names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  for (const auto & joint_name : right_wheel_names_) {
    conf_names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  
  return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::return_type MTVelocityController::update(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & period)
{
  auto current_command = received_velocity_msg_ptr_.readFromRT();
  
  double linear_vel = 0.0;
  double angular_vel = 0.0;
  
  // Check for command timeout
  if (current_command && *current_command) {
    if (is_command_timeout()) {
      if (verbose_) {
        RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 1000,
          "Command timeout - stopping robot");
      }
      reset_command();
    } else {
      linear_vel = (*current_command)->linear.x;
      angular_vel = (*current_command)->angular.z;
    }
  } else if (enable_safety_stop_) {
    // No command received yet or nullptr
    linear_vel = 0.0;
    angular_vel = 0.0;
  }
  
  // Apply velocity limits and smoothing
  const double dt = period.seconds();
  linear_vel = limit_velocity(linear_vel, previous_linear_velocity_, linear_limits_, dt);
  angular_vel = limit_velocity(angular_vel, previous_angular_velocity_, angular_limits_, dt);
  
  previous_linear_velocity_ = linear_vel;
  previous_angular_velocity_ = angular_vel;
  
  // Compute individual wheel velocities
  double left_vel, right_vel;
  compute_wheel_velocities(linear_vel, angular_vel, left_vel, right_vel);
  
  // Apply slip compensation if enabled
  if (enable_wheel_slip_compensation_ && !open_loop_) {
    // Read actual velocities and adjust (simplified slip compensation)
    // This is a placeholder for more sophisticated slip detection
    left_vel *= (1.0 + slip_factor_);
    right_vel *= (1.0 + slip_factor_);
  }
  
  // Command all wheel joints
  const size_t left_wheel_count = left_wheel_names_.size();
  const size_t right_wheel_count = right_wheel_names_.size();
  
  for (size_t i = 0; i < left_wheel_count; ++i) {
    command_interfaces_[i].set_value(left_vel);
  }
  
  for (size_t i = 0; i < right_wheel_count; ++i) {
    command_interfaces_[left_wheel_count + i].set_value(right_vel);
  }
  
  // Publish wheel velocities for diagnostics
  if (publish_wheel_velocities_ && wheel_velocities_publisher_ && 
      wheel_velocities_publisher_->trylock()) {
    wheel_velocities_publisher_->msg_.data.resize(4);
    wheel_velocities_publisher_->msg_.data[0] = left_vel;  // fl
    wheel_velocities_publisher_->msg_.data[1] = right_vel; // fr
    wheel_velocities_publisher_->msg_.data[2] = left_vel;  // bl
    wheel_velocities_publisher_->msg_.data[3] = right_vel; // br
    wheel_velocities_publisher_->unlockAndPublish();
  }
  
  return controller_interface::return_type::OK;
}

void MTVelocityController::twist_callback(
  const std::shared_ptr<geometry_msgs::msg::Twist> msg)
{
  received_velocity_msg_ptr_.writeFromNonRT(msg);
  last_command_time_ = std::chrono::steady_clock::now();
}

void MTVelocityController::twist_stamped_callback(
  const std::shared_ptr<geometry_msgs::msg::TwistStamped> msg)
{
  auto twist_msg = std::make_shared<geometry_msgs::msg::Twist>(msg->twist);
  received_velocity_msg_ptr_.writeFromNonRT(twist_msg);
  last_command_time_ = std::chrono::steady_clock::now();
}

bool MTVelocityController::load_parameters()
{
  left_wheel_names_ = get_node()->get_parameter("left_wheel_names").as_string_array();
  right_wheel_names_ = get_node()->get_parameter("right_wheel_names").as_string_array();
  
  if (left_wheel_names_.empty() || right_wheel_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Wheel names cannot be empty");
    return false;
  }
  
  wheel_separation_ = get_node()->get_parameter("wheel_separation").as_double();
  wheel_radius_ = get_node()->get_parameter("wheel_radius").as_double();
  
  if (wheel_separation_ <= 0.0 || wheel_radius_ <= 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(), 
                 "Wheel separation and radius must be positive values");
    return false;
  }
  
  wheel_separation_multiplier_ = get_node()->get_parameter("wheel_separation_multiplier").as_double();
  left_wheel_radius_multiplier_ = get_node()->get_parameter("left_wheel_radius_multiplier").as_double();
  right_wheel_radius_multiplier_ = get_node()->get_parameter("right_wheel_radius_multiplier").as_double();
  
  cmd_vel_timeout_ = get_node()->get_parameter("cmd_vel_timeout").as_double();
  command_topic_ = get_node()->get_parameter("command_topic").as_string();
  use_stamped_vel_ = get_node()->get_parameter("use_stamped_vel").as_bool();
  open_loop_ = get_node()->get_parameter("open_loop").as_bool();
  
  // Load velocity limits
  linear_limits_.has_velocity_limits = get_node()->get_parameter("linear.x.has_velocity_limits").as_bool();
  linear_limits_.max_velocity = get_node()->get_parameter("linear.x.max_velocity").as_double();
  linear_limits_.min_velocity = get_node()->get_parameter("linear.x.min_velocity").as_double();
  linear_limits_.has_acceleration_limits = get_node()->get_parameter("linear.x.has_acceleration_limits").as_bool();
  linear_limits_.max_acceleration = get_node()->get_parameter("linear.x.max_acceleration").as_double();
  linear_limits_.min_acceleration = get_node()->get_parameter("linear.x.min_acceleration").as_double();
  linear_limits_.has_jerk_limits = get_node()->get_parameter("linear.x.has_jerk_limits").as_bool();
  linear_limits_.max_jerk = get_node()->get_parameter("linear.x.max_jerk").as_double();
  
  angular_limits_.has_velocity_limits = get_node()->get_parameter("angular.z.has_velocity_limits").as_bool();
  angular_limits_.max_velocity = get_node()->get_parameter("angular.z.max_velocity").as_double();
  angular_limits_.min_velocity = get_node()->get_parameter("angular.z.min_velocity").as_double();
  angular_limits_.has_acceleration_limits = get_node()->get_parameter("angular.z.has_acceleration_limits").as_bool();
  angular_limits_.max_acceleration = get_node()->get_parameter("angular.z.max_acceleration").as_double();
  angular_limits_.min_acceleration = get_node()->get_parameter("angular.z.min_acceleration").as_double();
  angular_limits_.has_jerk_limits = get_node()->get_parameter("angular.z.has_jerk_limits").as_bool();
  angular_limits_.max_jerk = get_node()->get_parameter("angular.z.max_jerk").as_double();
  
  enable_wheel_slip_compensation_ = get_node()->get_parameter("enable_wheel_slip_compensation").as_bool();
  slip_factor_ = get_node()->get_parameter("slip_factor").as_double();
  publish_rate_ = get_node()->get_parameter("publish_rate").as_double();
  enable_safety_stop_ = get_node()->get_parameter("enable_safety_stop").as_bool();
  safety_stop_timeout_ = get_node()->get_parameter("safety_stop_timeout").as_double();
  publish_wheel_velocities_ = get_node()->get_parameter("publish_wheel_velocities").as_bool();
  verbose_ = get_node()->get_parameter("verbose").as_bool();
  
  return true;
}

void MTVelocityController::reset_command()
{
  // Set all wheel velocities to zero
  for (auto & interface : command_interfaces_) {
    interface.set_value(0.0);
  }
}

bool MTVelocityController::is_command_timeout() const
{
  if (cmd_vel_timeout_ <= 0.0) {
    return false;  // Timeout disabled
  }
  
  auto now = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(
    now - last_command_time_).count();
  
  return elapsed > cmd_vel_timeout_;
}

double MTVelocityController::limit_velocity(
  double velocity, double previous_velocity,
  const VelocityLimits& limits, double dt)
{
  double limited_velocity = velocity;
  
  // Apply velocity limits
  if (limits.has_velocity_limits) {
    limited_velocity = std::clamp(limited_velocity, limits.min_velocity, limits.max_velocity);
  }
  
  // Apply acceleration limits
  if (limits.has_acceleration_limits && dt > 0.0) {
    const double delta_v = limited_velocity - previous_velocity;
    const double max_delta_v = limits.max_acceleration * dt;
    const double min_delta_v = limits.min_acceleration * dt;
    
    const double clamped_delta_v = std::clamp(delta_v, min_delta_v, max_delta_v);
    limited_velocity = previous_velocity + clamped_delta_v;
  }
  
  // Note: Jerk limiting would require tracking acceleration history
  // Simplified implementation doesn't include jerk limiting
  
  return limited_velocity;
}

void MTVelocityController::compute_wheel_velocities(
  double linear_vel, double angular_vel,
  double& left_vel, double& right_vel)
{
  // Differential drive kinematics
  // v_left = (2 * v - omega * L) / (2 * r)
  // v_right = (2 * v + omega * L) / (2 * r)
  // where v = linear velocity, omega = angular velocity, L = wheel separation, r = wheel radius
  
  const double left_linear = linear_vel - (angular_vel * wheel_separation_computed_ / 2.0);
  const double right_linear = linear_vel + (angular_vel * wheel_separation_computed_ / 2.0);
  
  // Convert linear wheel velocities to angular velocities (rad/s)
  left_vel = left_linear / left_wheel_radius_computed_;
  right_vel = right_linear / right_wheel_radius_computed_;
}

}  // namespace mt_velocity_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  mt_velocity_controller::MTVelocityController,
  controller_interface::ControllerInterface)
