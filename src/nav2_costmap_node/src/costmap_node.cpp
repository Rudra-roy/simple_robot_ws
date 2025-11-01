#include "nav2_costmap_node/costmap_node.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <cmath>
#include <algorithm>

namespace nav2_costmap_node {

CostmapNode::CostmapNode() : rclcpp::Node("costmap_node"), 
                             static_map_received_(false),
                             robot_x_(0.0), robot_y_(0.0) {
  // Declare parameters
  this->declare_parameter("global_frame_id", std::string("odom"));
  this->declare_parameter("base_frame_id", std::string("base_link"));
  this->declare_parameter("resolution", 0.05);
  this->declare_parameter("width", 500);
  this->declare_parameter("height", 500);
  this->declare_parameter("origin_x", -2.5);
  this->declare_parameter("origin_y", -2.5);
  this->declare_parameter("inflation_radius", 0.3);
  this->declare_parameter("cost_scaling_factor", 10.0);
  this->declare_parameter("track_unknown_space", true);
  this->declare_parameter("use_static_map", false);
  this->declare_parameter("obstacle_max_range", 5.0);
  this->declare_parameter("obstacle_min_range", 0.1);
  this->declare_parameter("point_cloud_topic", std::string("/rosbot/camera_depth/point_cloud"));
  this->declare_parameter("costmap_topic", std::string("/costmap"));
  this->declare_parameter("static_map_topic", std::string("/map"));
  this->declare_parameter("publish_frequency", 10.0);

  // Get parameters
  global_frame_ = this->get_parameter("global_frame_id").as_string();
  robot_frame_ = this->get_parameter("base_frame_id").as_string();
  resolution_ = this->get_parameter("resolution").as_double();
  width_ = this->get_parameter("width").as_int();
  height_ = this->get_parameter("height").as_int();
  origin_x_ = this->get_parameter("origin_x").as_double();
  origin_y_ = this->get_parameter("origin_y").as_double();
  inflation_radius_ = this->get_parameter("inflation_radius").as_double();
  cost_scaling_factor_ = this->get_parameter("cost_scaling_factor").as_double();
  track_unknown_space_ = this->get_parameter("track_unknown_space").as_bool();
  use_static_map_ = this->get_parameter("use_static_map").as_bool();
  obstacle_max_range_ = this->get_parameter("obstacle_max_range").as_double();
  obstacle_min_range_ = this->get_parameter("obstacle_min_range").as_double();
  double publish_freq = this->get_parameter("publish_frequency").as_double();

  std::string pc_topic = this->get_parameter("point_cloud_topic").as_string();
  std::string costmap_topic = this->get_parameter("costmap_topic").as_string();
  std::string map_topic = this->get_parameter("static_map_topic").as_string();

  // Initialize simple costmap
  costmap_data_.resize(width_ * height_, static_cast<int8_t>(CostValues::FREE_SPACE));

  // Initialize TF buffer and listener
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Create subscriptions and publications
  pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      pc_topic, 10,
      std::bind(&CostmapNode::point_cloud_callback, this, std::placeholders::_1));

  if (use_static_map_) {
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic, rclcpp::QoS(10).transient_local(),
        std::bind(&CostmapNode::map_callback, this, std::placeholders::_1));
  }

  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(costmap_topic, 10);

  // Create publish timer
  timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_freq),
      std::bind(&CostmapNode::publish_costmap, this));

  RCLCPP_INFO(this->get_logger(), "Simple costmap node initialized");
  RCLCPP_INFO(this->get_logger(), "  Global frame: %s, Robot frame: %s", global_frame_.c_str(), robot_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Size: %dx%d (%.2f MB), Resolution: %.3fm", 
              width_, height_, (width_ * height_ * sizeof(int8_t)) / 1024.0 / 1024.0, resolution_);
  RCLCPP_INFO(this->get_logger(), "  Inflation radius: %.2fm", inflation_radius_);
}

void CostmapNode::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received static map: %dx%d @ %.3fm resolution",
              msg->info.width, msg->info.height, msg->info.resolution);
  static_map_ = msg;
  static_map_received_ = true;
  
  // Simple map integration - just copy occupied cells
  if (static_map_received_) {
    for (size_t i = 0; i < static_map_->data.size() && i < costmap_data_.size(); ++i) {
      if (static_map_->data[i] > 50) {  // Occupied threshold
        costmap_data_[i] = static_cast<int8_t>(CostValues::LETHAL_OBSTACLE);
      }
    }
    RCLCPP_INFO(this->get_logger(), "Static map integrated into costmap");
  }
}

void CostmapNode::point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  try {
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                          "Received point cloud with %d points", msg->width * msg->height);

    // Get robot position in global frame
    try {
      auto transform = tf_buffer_->lookupTransform(
          global_frame_, robot_frame_, tf2::TimePointZero);
          
      robot_x_ = transform.transform.translation.x;
      robot_y_ = transform.transform.translation.y;
      
    } catch (tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Could not get robot transform: %s", ex.what());
      return;
    }

    // Simple obstacle detection - just mark obstacles
    update_obstacles_simple(msg);

  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error processing point cloud: %s", e.what());
  }
}

void CostmapNode::update_obstacles_simple(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  // Get robot position in map coordinates
  int robot_mx, robot_my;
  if (!world_to_map(robot_x_, robot_y_, robot_mx, robot_my)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Robot position outside costmap bounds");
    return;
  }

  // Decay all existing obstacles (except static ones)
  for (size_t i = 0; i < costmap_data_.size(); ++i) {
    int8_t current_cost = costmap_data_[i];
    
    // Skip if it's from static map (lethal obstacle)
    if (use_static_map_ && static_map_received_ && i < static_map_->data.size()) {
      if (static_map_->data[i] > 50) {
        continue; // Don't decay static obstacles
      }
    }
    
    // Decay dynamic obstacles
    if (current_cost > static_cast<int8_t>(CostValues::FREE_SPACE)) {
      // Aggressive decay - 50% each update
      int8_t new_cost = static_cast<int8_t>(current_cost * 0.5);
      if (new_cost <= 5) {
        new_cost = static_cast<int8_t>(CostValues::FREE_SPACE);
      }
      costmap_data_[i] = new_cost;
    }
  }

  // Get transform from point cloud frame to global frame
  geometry_msgs::msg::TransformStamped transform_to_global;
  try {
    transform_to_global = tf_buffer_->lookupTransform(
        global_frame_, msg->header.frame_id, msg->header.stamp,
        tf2::durationFromSec(0.1));
  } catch (tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Could not transform point cloud to %s: %s", 
                         global_frame_.c_str(), ex.what());
    return;
  }

  // Extract points and mark obstacles
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
  
  int points_marked = 0;
  int points_processed = 0;

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    float x = *iter_x;
    float y = *iter_y;
    float z = *iter_z;

    // Filter invalid points
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }

    // Filter by height (ignore ground and ceiling)
    // More permissive height range to catch all obstacles
    if (z < 0.05f || z > 2.5f) {
      continue;
    }

    points_processed++;

    // Transform point to global frame
    geometry_msgs::msg::PointStamped point_in, point_out;
    point_in.header = msg->header;
    point_in.point.x = x;
    point_in.point.y = y;
    point_in.point.z = z;
    
    tf2::doTransform(point_in, point_out, transform_to_global);
    
    double global_x = point_out.point.x;
    double global_y = point_out.point.y;
    
    // Check range from robot
    double dx = global_x - robot_x_;
    double dy = global_y - robot_y_;
    double range = std::sqrt(dx*dx + dy*dy);
    
    if (range < obstacle_min_range_ || range > obstacle_max_range_) {
      continue;
    }

    // Convert to grid coordinates
    int obstacle_mx, obstacle_my;
    if (!world_to_map(global_x, global_y, obstacle_mx, obstacle_my)) {
      continue;
    }

    // Mark obstacle with simple inflation
    mark_obstacle_with_inflation(obstacle_mx, obstacle_my);
    points_marked++;
  }
  
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                        "Processed %d points, marked %d obstacles (frame: %s)", 
                        points_processed, points_marked, msg->header.frame_id.c_str());
}

void CostmapNode::mark_obstacle_with_inflation(int mx, int my) {
  int inflation_cells = static_cast<int>(std::ceil(inflation_radius_ / resolution_));
  
  // Mark a larger core obstacle area (3x3 cells minimum)
  for (int dy = -1; dy <= 1; ++dy) {
    for (int dx = -1; dx <= 1; ++dx) {
      int nx = mx + dx;
      int ny = my + dy;
      if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_) {
        unsigned int idx = get_index(nx, ny);
        costmap_data_[idx] = static_cast<int8_t>(CostValues::LETHAL_OBSTACLE);
      }
    }
  }
  
  // Add inflation around the core obstacle
  for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
    for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
      int nx = mx + dx;
      int ny = my + dy;

      if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_) {
        double dist = std::sqrt(dx * dx + dy * dy) * resolution_;
        unsigned int idx = get_index(nx, ny);
        
        if (dist <= resolution_ * 2.0) {
          // Larger core area - lethal obstacle
          costmap_data_[idx] = static_cast<int8_t>(CostValues::LETHAL_OBSTACLE);
        } else if (dist <= inflation_radius_) {
          // Inflation zone - linear decay for better coverage
          double cost_factor = 1.0 - ((dist - resolution_ * 2.0) / (inflation_radius_ - resolution_ * 2.0));
          int8_t cost = static_cast<int8_t>(std::max(50.0, cost_factor * 253.0));
          
          // Only update if new cost is higher (don't reduce existing obstacles)
          costmap_data_[idx] = std::max(costmap_data_[idx], cost);
        }
      }
    }
  }
}

bool CostmapNode::world_to_map(double wx, double wy, int& mx, int& my) const {
  // Get current robot position to calculate costmap origin
  double costmap_origin_x = robot_x_ - (width_ * resolution_ / 2.0);
  double costmap_origin_y = robot_y_ - (height_ * resolution_ / 2.0);
  
  mx = static_cast<int>((wx - costmap_origin_x) / resolution_);
  my = static_cast<int>((wy - costmap_origin_y) / resolution_);
  
  return (mx >= 0 && mx < width_ && my >= 0 && my < height_);
}

void CostmapNode::map_to_world(int mx, int my, double& wx, double& wy) const {
  double costmap_origin_x = robot_x_ - (width_ * resolution_ / 2.0);
  double costmap_origin_y = robot_y_ - (height_ * resolution_ / 2.0);
  
  wx = costmap_origin_x + (mx + 0.5) * resolution_;
  wy = costmap_origin_y + (my + 0.5) * resolution_;
}

unsigned int CostmapNode::get_index(int mx, int my) const {
  return my * width_ + mx;
}

uint8_t CostmapNode::compute_cost(int8_t base_cost) const {
  if (base_cost < 0) {
    return static_cast<uint8_t>(CostValues::NO_INFORMATION);
  }
  return static_cast<uint8_t>(base_cost);
}

void CostmapNode::publish_costmap() {
  // Get current robot position
  try {
    auto transform = tf_buffer_->lookupTransform(
        global_frame_, robot_frame_, tf2::TimePointZero);
    robot_x_ = transform.transform.translation.x;
    robot_y_ = transform.transform.translation.y;
  } catch (tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Could not get robot transform: %s", ex.what());
    return;
  }

  auto stamp = this->now();
  
  // Calculate costmap origin (centered on robot)
  double costmap_origin_x = robot_x_ - (width_ * resolution_ / 2.0);
  double costmap_origin_y = robot_y_ - (height_ * resolution_ / 2.0);

  // Publish simple costmap
  auto costmap_msg = std::make_unique<nav_msgs::msg::OccupancyGrid>();
  costmap_msg->header.stamp = stamp;
  costmap_msg->header.frame_id = global_frame_;
  costmap_msg->info.resolution = resolution_;
  costmap_msg->info.width = width_;
  costmap_msg->info.height = height_;
  costmap_msg->info.origin.position.x = costmap_origin_x;
  costmap_msg->info.origin.position.y = costmap_origin_y;
  costmap_msg->info.origin.position.z = 0.0;
  costmap_msg->info.origin.orientation.w = 1.0;
  costmap_msg->data = costmap_data_;
  costmap_pub_->publish(std::move(costmap_msg));
  
  RCLCPP_INFO_ONCE(this->get_logger(), 
                   "Publishing simple costmap (frame: %s)", global_frame_.c_str());
  
  RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                        "Published costmap at (%.2f, %.2f)", robot_x_, robot_y_);
}

}  // namespace nav2_costmap_node

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<nav2_costmap_node::CostmapNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
