#ifndef MT_VELOCITY_CONTROLLER__MT_VELOCITY_CONTROLLER_HPP_
#define MT_VELOCITY_CONTROLLER__MT_VELOCITY_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"

namespace mt_velocity_controller
{

class MTVelocityController : public controller_interface::ControllerInterface
{
public:
  MTVelocityController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

protected:
  // Parameters
  std::vector<std::string> left_wheel_names_;
  std::vector<std::string> right_wheel_names_;
  
  double wheel_separation_;
  double wheel_radius_;
  double wheel_separation_multiplier_;
  double left_wheel_radius_multiplier_;
  double right_wheel_radius_multiplier_;
  
  double cmd_vel_timeout_;
  std::string command_topic_;
  bool use_stamped_vel_;
  bool open_loop_;
  
  // Velocity limits
  struct VelocityLimits {
    bool has_velocity_limits;
    double max_velocity;
    double min_velocity;
    bool has_acceleration_limits;
    double max_acceleration;
    double min_acceleration;
    bool has_jerk_limits;
    double max_jerk;
  };
  
  VelocityLimits linear_limits_;
  VelocityLimits angular_limits_;
  
  bool enable_wheel_slip_compensation_;
  double slip_factor_;
  double publish_rate_;
  bool enable_safety_stop_;
  double safety_stop_timeout_;
  bool publish_wheel_velocities_;
  bool verbose_;

  // Runtime variables
  std::chrono::steady_clock::time_point last_command_time_;
  double previous_linear_velocity_;
  double previous_angular_velocity_;
  
  // Computed wheel separation and radius with multipliers
  double wheel_separation_computed_;
  double left_wheel_radius_computed_;
  double right_wheel_radius_computed_;

  // Subscribers and publishers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_stamped_sub_;
  
  std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>> 
    wheel_velocities_publisher_;
  
  realtime_tools::RealtimeBuffer<std::shared_ptr<geometry_msgs::msg::Twist>> received_velocity_msg_ptr_;

  // Helper methods
  void twist_callback(const std::shared_ptr<geometry_msgs::msg::Twist> msg);
  void twist_stamped_callback(const std::shared_ptr<geometry_msgs::msg::TwistStamped> msg);
  
  bool load_parameters();
  void reset_command();
  bool is_command_timeout() const;
  
  double limit_velocity(double velocity, double previous_velocity, 
                       const VelocityLimits& limits, double dt);
  
  void compute_wheel_velocities(double linear_vel, double angular_vel,
                                double& left_vel, double& right_vel);
};

}  // namespace mt_velocity_controller

#endif  // MT_VELOCITY_CONTROLLER__MT_VELOCITY_CONTROLLER_HPP_
