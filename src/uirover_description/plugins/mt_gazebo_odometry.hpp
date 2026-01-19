#ifndef MT_GAZEBO_ODOMETRY__MT_GAZEBO_ODOMETRY_HPP_
#define MT_GAZEBO_ODOMETRY__MT_GAZEBO_ODOMETRY_HPP_

#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "controller_interface/controller_interface.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "gazebo_msgs/msg/model_states.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace mt_gazebo_odometry
{

class MTGazeboOdometry : public controller_interface::ControllerInterface
{
public:
  MTGazeboOdometry();

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
  std::string odom_frame_id_;
  std::string base_frame_id_;
  std::string robot_model_name_;
  double publish_rate_;
  bool use_imu_orientation_;
  std::string imu_topic_;
  std::string model_states_topic_;
  bool publish_tf_;
  bool enable_pose_covariance_;
  bool enable_twist_covariance_;
  bool verbose_;
  
  // Covariance parameters
  std::vector<double> position_covariance_;
  std::vector<double> orientation_covariance_;
  std::vector<double> linear_velocity_covariance_;
  std::vector<double> angular_velocity_covariance_;

  // Runtime variables
  std::chrono::steady_clock::time_point last_publish_time_;
  double previous_x_;
  double previous_y_;
  double previous_z_;
  double previous_qx_;
  double previous_qy_;
  double previous_qz_;
  double previous_qw_;
  rclcpp::Time previous_time_;
  bool first_update_;

  // Publishers and subscribers
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr model_states_sub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>> odom_publisher_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  
  realtime_tools::RealtimeBuffer<std::shared_ptr<sensor_msgs::msg::Imu>> received_imu_msg_ptr_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<gazebo_msgs::msg::ModelStates>> received_model_states_ptr_;
  int robot_model_index_;

  // Helper methods
  void imu_callback(const std::shared_ptr<sensor_msgs::msg::Imu> msg);
  void model_states_callback(const std::shared_ptr<gazebo_msgs::msg::ModelStates> msg);
  bool load_parameters();
  int find_model_index(const std::vector<std::string>& names, const std::string& model_name);
  void compute_velocities(
    double current_x, double current_y, double current_z,
    double current_qx, double current_qy, double current_qz, double current_qw,
    double dt,
    double& linear_x, double& linear_y, double& linear_z,
    double& angular_x, double& angular_y, double& angular_z);
  void set_odometry_covariance(nav_msgs::msg::Odometry& odom);
};

}  // namespace mt_gazebo_odometry

#endif  // MT_GAZEBO_ODOMETRY__MT_GAZEBO_ODOMETRY_HPP_
