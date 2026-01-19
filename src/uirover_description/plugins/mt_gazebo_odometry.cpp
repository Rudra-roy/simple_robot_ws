#include "mt_gazebo_odometry.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "gazebo_msgs/msg/model_states.hpp"

namespace mt_gazebo_odometry
{

MTGazeboOdometry::MTGazeboOdometry()
: controller_interface::ControllerInterface(),
  previous_x_(0.0),
  previous_y_(0.0),
  previous_z_(0.0),
  previous_qx_(0.0),
  previous_qy_(0.0),
  previous_qz_(0.0),
  previous_qw_(1.0),
  first_update_(true),
  robot_model_index_(-1)
{
}

controller_interface::CallbackReturn MTGazeboOdometry::on_init()
{
  try {
    auto_declare<std::string>("odom_frame_id", "odom");
    auto_declare<std::string>("base_frame_id", "base_link");
    auto_declare<std::string>("robot_model_name", "drubotara_rover");
    auto_declare<double>("publish_rate", 50.0);
    auto_declare<bool>("use_imu_orientation", true);
    auto_declare<std::string>("imu_topic", "/imu");
    auto_declare<std::string>("model_states_topic", "/model_states");
    auto_declare<bool>("publish_tf", true);
    auto_declare<bool>("enable_pose_covariance", true);
    auto_declare<bool>("enable_twist_covariance", true);
    auto_declare<bool>("verbose", false);
    
    // Covariance parameters (diagonal values)
    auto_declare<std::vector<double>>("position_covariance", std::vector<double>{0.001, 0.001, 0.001});
    auto_declare<std::vector<double>>("orientation_covariance", std::vector<double>{0.001, 0.001, 0.001});
    auto_declare<std::vector<double>>("linear_velocity_covariance", std::vector<double>{0.001, 0.001, 0.001});
    auto_declare<std::vector<double>>("angular_velocity_covariance", std::vector<double>{0.001, 0.001, 0.001});
  }
  catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MTGazeboOdometry::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!load_parameters()) {
    return controller_interface::CallbackReturn::ERROR;
  }

  // Setup IMU subscriber
  if (use_imu_orientation_) {
    imu_sub_ = get_node()->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<sensor_msgs::msg::Imu> msg) {
        imu_callback(msg);
      });
  }

  // Setup model states subscriber
  model_states_sub_ = get_node()->create_subscription<gazebo_msgs::msg::ModelStates>(
    model_states_topic_, rclcpp::SystemDefaultsQoS(),
    [this](const std::shared_ptr<gazebo_msgs::msg::ModelStates> msg) {
      model_states_callback(msg);
    });

  // Setup odometry publisher
  odom_publisher_ = 
    std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(
      get_node()->create_publisher<nav_msgs::msg::Odometry>(
        "/odom", rclcpp::SystemDefaultsQoS()));

  // Setup TF broadcaster
  if (publish_tf_) {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(get_node());
  }

  // Initialize IMU buffer
  received_imu_msg_ptr_.writeFromNonRT(std::shared_ptr<sensor_msgs::msg::Imu>());
  received_model_states_ptr_.writeFromNonRT(std::shared_ptr<gazebo_msgs::msg::ModelStates>());
  robot_model_index_ = -1;

  RCLCPP_INFO(get_node()->get_logger(), "MT Gazebo Odometry configured successfully");
  
  if (verbose_) {
    RCLCPP_INFO(get_node()->get_logger(), "  Robot model name: %s", robot_model_name_.c_str());
    RCLCPP_INFO(get_node()->get_logger(), "  Odom frame: %s", odom_frame_id_.c_str());
    RCLCPP_INFO(get_node()->get_logger(), "  Base frame: %s", base_frame_id_.c_str());
    RCLCPP_INFO(get_node()->get_logger(), "  Publish rate: %.1f Hz", publish_rate_);
    RCLCPP_INFO(get_node()->get_logger(), "  Use IMU orientation: %s", use_imu_orientation_ ? "true" : "false");
    RCLCPP_INFO(get_node()->get_logger(), "  Publish TF: %s", publish_tf_ ? "true" : "false");
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MTGazeboOdometry::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Reset state
  last_publish_time_ = std::chrono::steady_clock::now();
  first_update_ = true;
  previous_time_ = get_node()->now();
  robot_model_index_ = -1;

  RCLCPP_INFO(get_node()->get_logger(), "MT Gazebo Odometry activated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MTGazeboOdometry::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "MT Gazebo Odometry deactivated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration 
MTGazeboOdometry::command_interface_configuration() const
{
  // No command interfaces needed - this is a read-only controller
  return {controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration 
MTGazeboOdometry::state_interface_configuration() const
{
  // No state interfaces needed - we get data from topics
  return {controller_interface::interface_configuration_type::NONE};
}

controller_interface::return_type MTGazeboOdometry::update(
  const rclcpp::Time & time,
  const rclcpp::Duration & period)
{
  // Check if it's time to publish
  auto now = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(
    now - last_publish_time_).count();
  
  const double publish_period = 1.0 / publish_rate_;
  if (elapsed < publish_period) {
    return controller_interface::return_type::OK;
  }
  
  last_publish_time_ = now;

  // Read current pose from Gazebo model states
  auto current_model_states = received_model_states_ptr_.readFromRT();
  if (!current_model_states || !(*current_model_states)) {
    if (verbose_) {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "No model states received yet");
    }
    return controller_interface::return_type::OK;
  }

  // Find robot model index if not found yet
  if (robot_model_index_ < 0) {
    robot_model_index_ = find_model_index((*current_model_states)->name, robot_model_name_);
    if (robot_model_index_ < 0) {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Model '%s' not found in Gazebo model states", robot_model_name_.c_str());
      return controller_interface::return_type::OK;
    }
    RCLCPP_INFO(get_node()->get_logger(), "Found robot model '%s' at index %d", 
                robot_model_name_.c_str(), robot_model_index_);
  }

  const auto& pose = (*current_model_states)->pose[robot_model_index_];
  const double current_x = pose.position.x;
  const double current_y = pose.position.y;
  const double current_z = pose.position.z;
  const double current_qx = pose.orientation.x;
  const double current_qy = pose.orientation.y;
  const double current_qz = pose.orientation.z;
  const double current_qw = pose.orientation.w;

  // Compute velocities
  double linear_x = 0.0, linear_y = 0.0, linear_z = 0.0;
  double angular_x = 0.0, angular_y = 0.0, angular_z = 0.0;
  
  const double dt = period.seconds();
  if (!first_update_ && dt > 0.0) {
    compute_velocities(
      current_x, current_y, current_z,
      current_qx, current_qy, current_qz, current_qw,
      dt,
      linear_x, linear_y, linear_z,
      angular_x, angular_y, angular_z);
  }

  // Update previous values
  previous_x_ = current_x;
  previous_y_ = current_y;
  previous_z_ = current_z;
  previous_qx_ = current_qx;
  previous_qy_ = current_qy;
  previous_qz_ = current_qz;
  previous_qw_ = current_qw;
  previous_time_ = time;
  first_update_ = false;

  // Prepare odometry message
  if (odom_publisher_ && odom_publisher_->trylock()) {
    auto & odom_msg = odom_publisher_->msg_;
    odom_msg.header.stamp = time;
    odom_msg.header.frame_id = odom_frame_id_;
    odom_msg.child_frame_id = base_frame_id_;

    // Set position from Gazebo
    odom_msg.pose.pose.position.x = current_x;
    odom_msg.pose.pose.position.y = current_y;
    odom_msg.pose.pose.position.z = current_z;

    // Set orientation - use IMU if available and enabled
    if (use_imu_orientation_) {
      auto current_imu = received_imu_msg_ptr_.readFromRT();
      if (current_imu && *current_imu) {
        odom_msg.pose.pose.orientation = (*current_imu)->orientation;
      } else {
        // Fallback to Gazebo orientation
        odom_msg.pose.pose.orientation.x = current_qx;
        odom_msg.pose.pose.orientation.y = current_qy;
        odom_msg.pose.pose.orientation.z = current_qz;
        odom_msg.pose.pose.orientation.w = current_qw;
      }
    } else {
      // Use Gazebo orientation
      odom_msg.pose.pose.orientation.x = current_qx;
      odom_msg.pose.pose.orientation.y = current_qy;
      odom_msg.pose.pose.orientation.z = current_qz;
      odom_msg.pose.pose.orientation.w = current_qw;
    }

    // Set velocities
    odom_msg.twist.twist.linear.x = linear_x;
    odom_msg.twist.twist.linear.y = linear_y;
    odom_msg.twist.twist.linear.z = linear_z;
    odom_msg.twist.twist.angular.x = angular_x;
    odom_msg.twist.twist.angular.y = angular_y;
    odom_msg.twist.twist.angular.z = angular_z;

    // Set covariances
    set_odometry_covariance(odom_msg);

    odom_publisher_->unlockAndPublish();
  }

  // Publish TF transform
  if (publish_tf_ && tf_broadcaster_) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = time;
    transform.header.frame_id = odom_frame_id_;
    transform.child_frame_id = base_frame_id_;

    transform.transform.translation.x = current_x;
    transform.transform.translation.y = current_y;
    transform.transform.translation.z = current_z;

    // Use the same orientation as in odometry message
    if (use_imu_orientation_) {
      auto current_imu = received_imu_msg_ptr_.readFromRT();
      if (current_imu && *current_imu) {
        transform.transform.rotation = (*current_imu)->orientation;
      } else {
        transform.transform.rotation.x = current_qx;
        transform.transform.rotation.y = current_qy;
        transform.transform.rotation.z = current_qz;
        transform.transform.rotation.w = current_qw;
      }
    } else {
      transform.transform.rotation.x = current_qx;
      transform.transform.rotation.y = current_qy;
      transform.transform.rotation.z = current_qz;
      transform.transform.rotation.w = current_qw;
    }

    tf_broadcaster_->sendTransform(transform);
  }

  return controller_interface::return_type::OK;
}

void MTGazeboOdometry::imu_callback(
  const std::shared_ptr<sensor_msgs::msg::Imu> msg)
{
  received_imu_msg_ptr_.writeFromNonRT(msg);
}

void MTGazeboOdometry::model_states_callback(
  const std::shared_ptr<gazebo_msgs::msg::ModelStates> msg)
{
  received_model_states_ptr_.writeFromNonRT(msg);
}

bool MTGazeboOdometry::load_parameters()
{
  odom_frame_id_ = get_node()->get_parameter("odom_frame_id").as_string();
  base_frame_id_ = get_node()->get_parameter("base_frame_id").as_string();
  robot_model_name_ = get_node()->get_parameter("robot_model_name").as_string();
  publish_rate_ = get_node()->get_parameter("publish_rate").as_double();
  use_imu_orientation_ = get_node()->get_parameter("use_imu_orientation").as_bool();
  imu_topic_ = get_node()->get_parameter("imu_topic").as_string();
  model_states_topic_ = get_node()->get_parameter("model_states_topic").as_string();
  publish_tf_ = get_node()->get_parameter("publish_tf").as_bool();
  enable_pose_covariance_ = get_node()->get_parameter("enable_pose_covariance").as_bool();
  enable_twist_covariance_ = get_node()->get_parameter("enable_twist_covariance").as_bool();
  verbose_ = get_node()->get_parameter("verbose").as_bool();
  
  position_covariance_ = get_node()->get_parameter("position_covariance").as_double_array();
  orientation_covariance_ = get_node()->get_parameter("orientation_covariance").as_double_array();
  linear_velocity_covariance_ = get_node()->get_parameter("linear_velocity_covariance").as_double_array();
  angular_velocity_covariance_ = get_node()->get_parameter("angular_velocity_covariance").as_double_array();
  
  if (publish_rate_ <= 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(), "Publish rate must be positive");
    return false;
  }
  
  return true;
}

void MTGazeboOdometry::compute_velocities(
  double current_x, double current_y, double current_z,
  double current_qx, double current_qy, double current_qz, double current_qw,
  double dt,
  double& linear_x, double& linear_y, double& linear_z,
  double& angular_x, double& angular_y, double& angular_z)
{
  // Compute linear velocities
  linear_x = (current_x - previous_x_) / dt;
  linear_y = (current_y - previous_y_) / dt;
  linear_z = (current_z - previous_z_) / dt;

  // Compute angular velocities from quaternion difference
  // q_dot = 0.5 * omega * q
  // omega = 2 * q_dot * q^-1
  
  // Compute quaternion derivative
  double dqx = (current_qx - previous_qx_) / dt;
  double dqy = (current_qy - previous_qy_) / dt;
  double dqz = (current_qz - previous_qz_) / dt;
  double dqw = (current_qw - previous_qw_) / dt;

  // Conjugate of current quaternion (for inverse since it's unit quaternion)
  double qx_conj = -current_qx;
  double qy_conj = -current_qy;
  double qz_conj = -current_qz;
  double qw_conj = current_qw;

  // omega = 2 * q_dot * q^-1
  // Quaternion multiplication: q_dot * q_conj
  double omega_x = 2.0 * (dqw * qx_conj + dqx * qw_conj + dqy * qz_conj - dqz * qy_conj);
  double omega_y = 2.0 * (dqw * qy_conj - dqx * qz_conj + dqy * qw_conj + dqz * qx_conj);
  double omega_z = 2.0 * (dqw * qz_conj + dqx * qy_conj - dqy * qx_conj + dqz * qw_conj);

  angular_x = omega_x;
  angular_y = omega_y;
  angular_z = omega_z;
}

void MTGazeboOdometry::set_odometry_covariance(nav_msgs::msg::Odometry& odom)
{
  // Initialize all covariances to zero
  std::fill(odom.pose.covariance.begin(), odom.pose.covariance.end(), 0.0);
  std::fill(odom.twist.covariance.begin(), odom.twist.covariance.end(), 0.0);

  if (enable_pose_covariance_) {
    // Position covariance (first 3x3 block)
    if (position_covariance_.size() >= 3) {
      odom.pose.covariance[0] = position_covariance_[0];  // x
      odom.pose.covariance[7] = position_covariance_[1];  // y
      odom.pose.covariance[14] = position_covariance_[2]; // z
    }
    
    // Orientation covariance (second 3x3 block)
    if (orientation_covariance_.size() >= 3) {
      odom.pose.covariance[21] = orientation_covariance_[0]; // roll
      odom.pose.covariance[28] = orientation_covariance_[1]; // pitch
      odom.pose.covariance[35] = orientation_covariance_[2]; // yaw
    }
  }

  if (enable_twist_covariance_) {
    // Linear velocity covariance (first 3x3 block)
    if (linear_velocity_covariance_.size() >= 3) {
      odom.twist.covariance[0] = linear_velocity_covariance_[0];  // vx
      odom.twist.covariance[7] = linear_velocity_covariance_[1];  // vy
      odom.twist.covariance[14] = linear_velocity_covariance_[2]; // vz
    }
    
    // Angular velocity covariance (second 3x3 block)
    if (angular_velocity_covariance_.size() >= 3) {
      odom.twist.covariance[21] = angular_velocity_covariance_[0]; // omega_x
      odom.twist.covariance[28] = angular_velocity_covariance_[1]; // omega_y
      odom.twist.covariance[35] = angular_velocity_covariance_[2]; // omega_z
    }
  }
}

int MTGazeboOdometry::find_model_index(
  const std::vector<std::string>& names, 
  const std::string& model_name)
{
  for (size_t i = 0; i < names.size(); ++i) {
    if (names[i] == model_name) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

}  // namespace mt_gazebo_odometry

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  mt_gazebo_odometry::MTGazeboOdometry,
  controller_interface::ControllerInterface)
