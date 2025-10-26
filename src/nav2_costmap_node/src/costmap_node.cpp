#include "nav2_costmap_node/costmap_node.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <cmath>

namespace nav2_costmap_node {

CostmapNode::CostmapNode() : rclcpp::Node("costmap_node") {
  // Declare parameters
  this->declare_parameter("global_frame_id", std::string("odom"));
  this->declare_parameter("base_frame_id", std::string("base_link"));
  this->declare_parameter("resolution", 0.05);
  this->declare_parameter("width", 500);
  this->declare_parameter("height", 500);
  this->declare_parameter("origin_x", -2.5);
  this->declare_parameter("origin_y", -2.5);
  this->declare_parameter("inflation_radius", 0.1);
  this->declare_parameter("point_cloud_topic", std::string("/rosbot/camera_depth/point_cloud"));
  this->declare_parameter("costmap_topic", std::string("/costmap"));
  this->declare_parameter("publish_frequency", 100.0);

  // Get parameters
  global_frame_ = this->get_parameter("global_frame_id").as_string();
  robot_frame_ = this->get_parameter("base_frame_id").as_string();
  resolution_ = this->get_parameter("resolution").as_double();
  width_ = this->get_parameter("width").as_int();
  height_ = this->get_parameter("height").as_int();
  origin_x_ = this->get_parameter("origin_x").as_double();
  origin_y_ = this->get_parameter("origin_y").as_double();
  inflation_radius_ = this->get_parameter("inflation_radius").as_double();
  double publish_freq = this->get_parameter("publish_frequency").as_double();

  std::string pc_topic = this->get_parameter("point_cloud_topic").as_string();
  std::string costmap_topic = this->get_parameter("costmap_topic").as_string();

  // Initialize costmap data
  costmap_data_.resize(width_ * height_, 0);

  // Initialize TF buffer and listener
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Create subscription and publication
  pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      pc_topic, 10,
      std::bind(&CostmapNode::point_cloud_callback, this, std::placeholders::_1));

  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(costmap_topic, 10);

  // Create publish timer
  timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_freq),
      std::bind(&CostmapNode::publish_costmap, this));

  RCLCPP_INFO(this->get_logger(), "Costmap node initialized. Global frame: %s, Robot frame: %s, Publishing at %.1f Hz",
              global_frame_.c_str(), robot_frame_.c_str(), publish_freq);
  RCLCPP_INFO(this->get_logger(), "Costmap size: %dx%d (%.1f MB), Resolution: %.3fm",
              width_, height_, (width_ * height_ * sizeof(int8_t)) / 1024.0 / 1024.0, resolution_);
}

void CostmapNode::point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  try {
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                          "Received point cloud with %d points", msg->width * msg->height);

    // Decay existing costmap
    for (auto& cell : costmap_data_) {
      if (cell > 0) {
        cell = static_cast<int8_t>(cell * 0.95);
      }
    }

    // Get transform from point cloud frame to robot base frame
    geometry_msgs::msg::TransformStamped pc_to_base_transform;
    geometry_msgs::msg::TransformStamped base_to_odom_transform;
    try {
      // Transform point cloud to base_link
      pc_to_base_transform = tf_buffer_->lookupTransform(
          robot_frame_, msg->header.frame_id, msg->header.stamp, 
          tf2::durationFromSec(0.1));
      
      // Get robot position in odom
      base_to_odom_transform = tf_buffer_->lookupTransform(
          global_frame_, robot_frame_, tf2::TimePointZero);
    } catch (tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Could not get transforms: %s", ex.what());
      return;
    }

    double robot_x = base_to_odom_transform.transform.translation.x;
    double robot_y = base_to_odom_transform.transform.translation.y;
    
    // Calculate costmap origin (bottom-left corner in odom frame)
    double costmap_origin_x = robot_x - (width_ * resolution_ / 2.0);
    double costmap_origin_y = robot_y - (height_ * resolution_ / 2.0);

    // Extract points from PointCloud2
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
    
    int points_added = 0;

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      float x = *iter_x;
      float y = *iter_y;
      float z = *iter_z;

      // Filter invalid points
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }

      // Filter by height (0.1m to 2.0m to catch obstacles)
      if (z < 0.1f || z > 2.0f) {
        continue;
      }

      // Transform point from camera frame to base_link
      geometry_msgs::msg::PointStamped point_in_camera, point_in_base, point_in_odom;
      point_in_camera.header = msg->header;
      point_in_camera.point.x = x;
      point_in_camera.point.y = y;
      point_in_camera.point.z = z;
      
      tf2::doTransform(point_in_camera, point_in_base, pc_to_base_transform);
      tf2::doTransform(point_in_base, point_in_odom, base_to_odom_transform);

      // Convert odom coordinates to grid indices relative to costmap origin
      int grid_x = static_cast<int>((point_in_odom.point.x - costmap_origin_x) / resolution_);
      int grid_y = static_cast<int>((point_in_odom.point.y - costmap_origin_y) / resolution_);

      // Check bounds
      if (grid_x >= 0 && grid_x < width_ && grid_y >= 0 && grid_y < height_) {
        int idx = grid_y * width_ + grid_x;
        costmap_data_[idx] = 100;  // Mark as occupied
        points_added++;
      }
    }
    
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                          "Added %d obstacle points to costmap", points_added);

    // Apply inflation layer
    std::vector<int8_t> inflated_map = costmap_data_;
    int inflation_cells = static_cast<int>(inflation_radius_ / resolution_);

    for (int y = 0; y < height_; ++y) {
      for (int x = 0; x < width_; ++x) {
        if (costmap_data_[y * width_ + x] == 100) {
          // Inflate around this obstacle
          for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
            for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
              int nx = x + dx;
              int ny = y + dy;

              if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_) {
                int dist = static_cast<int>(std::sqrt(dx * dx + dy * dy));
                if (dist <= inflation_cells && dist > 0) {
                  int inflation_cost = static_cast<int>(
                      50.0 * (1.0 - static_cast<double>(dist) / inflation_cells));
                  int idx = ny * width_ + nx;
                  inflated_map[idx] = std::max(inflated_map[idx],
                                                static_cast<int8_t>(inflation_cost));
                }
              }
            }
          }
        }
      }
    }

    costmap_data_ = inflated_map;

  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error processing point cloud: %s", e.what());
  }
}

void CostmapNode::publish_costmap() {
  auto msg = std::make_unique<nav_msgs::msg::OccupancyGrid>();

  msg->header.stamp = this->now();
  msg->header.frame_id = global_frame_;  // Use odom frame to see robot moving

  msg->info.resolution = resolution_;
  msg->info.width = width_;
  msg->info.height = height_;
  
  // Get robot position in odom frame to center costmap on robot
  try {
    auto transform = tf_buffer_->lookupTransform(
        global_frame_, robot_frame_, tf2::TimePointZero);
    
    // Center costmap on robot's current position
    double robot_x = transform.transform.translation.x;
    double robot_y = transform.transform.translation.y;
    
    msg->info.origin.position.x = robot_x - (width_ * resolution_ / 2.0);
    msg->info.origin.position.y = robot_y - (height_ * resolution_ / 2.0);
    msg->info.origin.position.z = 0.0;
    msg->info.origin.orientation.w = 1.0;
    
  } catch (tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Could not get robot transform: %s", ex.what());
    // Fallback to fixed origin
    msg->info.origin.position.x = origin_x_;
    msg->info.origin.position.y = origin_y_;
    msg->info.origin.position.z = 0.0;
    msg->info.origin.orientation.w = 1.0;
  }

  msg->data = costmap_data_;

  RCLCPP_INFO_ONCE(this->get_logger(), "Publishing first costmap message (frame: %s)", global_frame_.c_str());
  
  costmap_pub_->publish(std::move(msg));
  
  RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                        "Published costmap in frame: %s", global_frame_.c_str());
}

}  // namespace nav2_costmap_node

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<nav2_costmap_node::CostmapNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
