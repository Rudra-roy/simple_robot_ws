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

  // Initialize costmap layers
  costmap_data_.resize(width_ * height_, static_cast<int8_t>(CostValues::NO_INFORMATION));
  static_layer_.resize(width_ * height_, static_cast<int8_t>(CostValues::NO_INFORMATION));
  obstacle_layer_.resize(width_ * height_, static_cast<int8_t>(CostValues::FREE_SPACE));
  inflation_layer_.resize(width_ * height_, static_cast<int8_t>(CostValues::FREE_SPACE));

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
  
  // Publishers for individual layers (for debugging/visualization)
  static_layer_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(costmap_topic + "/static_layer", 10);
  obstacle_layer_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(costmap_topic + "/obstacle_layer", 10);
  inflation_layer_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(costmap_topic + "/inflation_layer", 10);

  // Create publish timer
  timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_freq),
      std::bind(&CostmapNode::publish_costmap, this));

  RCLCPP_INFO(this->get_logger(), "Costmap node initialized with layered architecture");
  RCLCPP_INFO(this->get_logger(), "  Global frame: %s, Robot frame: %s", global_frame_.c_str(), robot_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Size: %dx%d (%.2f MB), Resolution: %.3fm", 
              width_, height_, (width_ * height_ * sizeof(int8_t)) / 1024.0 / 1024.0, resolution_);
  RCLCPP_INFO(this->get_logger(), "  Inflation radius: %.2fm, Cost scaling: %.1f", 
              inflation_radius_, cost_scaling_factor_);
  RCLCPP_INFO(this->get_logger(), "  Static map: %s, Track unknown: %s",
              use_static_map_ ? "enabled" : "disabled", 
              track_unknown_space_ ? "yes" : "no");
}

void CostmapNode::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received static map: %dx%d @ %.3fm resolution",
              msg->info.width, msg->info.height, msg->info.resolution);
  static_map_ = msg;
  static_map_received_ = true;
  update_static_layer();
}

void CostmapNode::point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  try {
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                          "Received point cloud with %d points", msg->width * msg->height);

    // Get transforms
    geometry_msgs::msg::TransformStamped pc_to_base_transform;
    geometry_msgs::msg::TransformStamped base_to_global_transform;
    
    try {
      // Transform point cloud to base_link
      pc_to_base_transform = tf_buffer_->lookupTransform(
          robot_frame_, msg->header.frame_id, msg->header.stamp, 
          tf2::durationFromSec(0.1));
      
      // Get robot position in global frame
      base_to_global_transform = tf_buffer_->lookupTransform(
          global_frame_, robot_frame_, tf2::TimePointZero);
          
      robot_x_ = base_to_global_transform.transform.translation.x;
      robot_y_ = base_to_global_transform.transform.translation.y;
      
    } catch (tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Could not get transforms: %s", ex.what());
      return;
    }

    // Update obstacle layer with ray tracing
    update_obstacle_layer(msg, pc_to_base_transform, base_to_global_transform);
    
    // Update inflation layer
    update_inflation_layer();
    
    // Combine all layers
    combine_layers();

  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error processing point cloud: %s", e.what());
  }
}

void CostmapNode::update_static_layer() {
  if (!static_map_received_ || !static_map_) {
    return;
  }
  
  // Copy static map data to static layer
  // This would need proper transformation if resolutions/origins differ
  // For now, assume same resolution and aligned grids
  std::fill(static_layer_.begin(), static_layer_.end(), static_cast<int8_t>(CostValues::FREE_SPACE));
  
  // Copy occupied cells from static map
  for (size_t i = 0; i < static_map_->data.size() && i < static_layer_.size(); ++i) {
    if (static_map_->data[i] > 50) {  // Occupied threshold
      static_layer_[i] = static_cast<int8_t>(CostValues::LETHAL_OBSTACLE);
    } else if (static_map_->data[i] < 0) {  // Unknown
      static_layer_[i] = static_cast<int8_t>(CostValues::NO_INFORMATION);
    }
  }
  
  RCLCPP_INFO(this->get_logger(), "Static layer updated from map");
}

void CostmapNode::update_obstacle_layer(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg,
    const geometry_msgs::msg::TransformStamped& pc_to_base,
    const geometry_msgs::msg::TransformStamped& base_to_global) {
  
  // Decay existing obstacle layer (for dynamic obstacles)
  for (auto& cell : obstacle_layer_) {
    if (cell == static_cast<int8_t>(CostValues::LETHAL_OBSTACLE)) {
      cell = static_cast<int8_t>(90);  // Decay lethal to high cost
    } else if (cell > static_cast<int8_t>(CostValues::FREE_SPACE)) {
      cell = static_cast<int8_t>(cell * 0.9);  // Gradual decay
    }
  }
  
  // Get robot position in map coordinates
  int robot_mx, robot_my;
  if (!world_to_map(robot_x_, robot_y_, robot_mx, robot_my)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Robot position outside costmap bounds");
    return;
  }

  // Extract points from PointCloud2 and apply ray tracing
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
  
  int points_marked = 0;
  int rays_traced = 0;

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    float x = *iter_x;
    float y = *iter_y;
    float z = *iter_z;

    // Filter invalid points
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }

    // Filter by height (ignore ground and ceiling)
    if (z < 0.1f || z > 2.0f) {
      continue;
    }

    // Transform point to global frame
    geometry_msgs::msg::PointStamped point_in_camera, point_in_base, point_in_global;
    point_in_camera.header = msg->header;
    point_in_camera.point.x = x;
    point_in_camera.point.y = y;
    point_in_camera.point.z = z;
    
    tf2::doTransform(point_in_camera, point_in_base, pc_to_base);
    tf2::doTransform(point_in_base, point_in_global, base_to_global);
    
    // Check range
    double dx = point_in_global.point.x - robot_x_;
    double dy = point_in_global.point.y - robot_y_;
    double range = std::sqrt(dx*dx + dy*dy);
    
    if (range < obstacle_min_range_ || range > obstacle_max_range_) {
      continue;
    }

    // Convert to grid coordinates
    int obstacle_mx, obstacle_my;
    if (!world_to_map(point_in_global.point.x, point_in_global.point.y, 
                     obstacle_mx, obstacle_my)) {
      continue;
    }

    // Ray trace from robot to obstacle to clear free space
    raytrace_line(robot_mx, robot_my, obstacle_mx, obstacle_my, obstacle_layer_);
    rays_traced++;
    
    // Mark obstacle endpoint
    unsigned int idx = get_index(obstacle_mx, obstacle_my);
    if (idx < obstacle_layer_.size()) {
      obstacle_layer_[idx] = static_cast<int8_t>(CostValues::LETHAL_OBSTACLE);
      points_marked++;
    }
  }
  
  RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                        "Obstacle layer: %d rays traced, %d obstacles marked", 
                        rays_traced, points_marked);
}

void CostmapNode::update_inflation_layer() {
  // Reset inflation layer
  std::fill(inflation_layer_.begin(), inflation_layer_.end(), 
            static_cast<int8_t>(CostValues::FREE_SPACE));
  
  int inflation_cells = static_cast<int>(std::ceil(inflation_radius_ / resolution_));
  
  // Precompute inflation costs for different distances
  std::vector<uint8_t> cached_costs(inflation_cells + 2, 0);
  for (int i = 0; i <= inflation_cells; ++i) {
    double dist = i * resolution_;
    if (dist == 0.0) {
      cached_costs[i] = static_cast<uint8_t>(CostValues::LETHAL_OBSTACLE);
    } else {
      // Exponential decay formula similar to Nav2
      double factor = std::exp(-1.0 * cost_scaling_factor_ * (dist - resolution_) / inflation_radius_);
      cached_costs[i] = static_cast<uint8_t>(
        std::min(253.0, std::max(0.0, static_cast<uint8_t>(CostValues::LETHAL_OBSTACLE) * factor)));
    }
  }

  // Inflate around obstacles in both static and obstacle layers
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      unsigned int idx = get_index(x, y);
      
      // Check if this cell is an obstacle in either layer
      bool is_obstacle = (static_layer_[idx] == static_cast<int8_t>(CostValues::LETHAL_OBSTACLE)) ||
                        (obstacle_layer_[idx] >= 90);  // High confidence obstacle
      
      if (is_obstacle) {
        // Inflate around this obstacle
        for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
          for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
            int nx = x + dx;
            int ny = y + dy;

            if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_) {
              int dist_cells = static_cast<int>(std::round(std::sqrt(dx * dx + dy * dy)));
              if (dist_cells <= inflation_cells) {
                unsigned int n_idx = get_index(nx, ny);
                uint8_t cost = cached_costs[dist_cells];
                inflation_layer_[n_idx] = std::max(inflation_layer_[n_idx], 
                                                   static_cast<int8_t>(cost));
              }
            }
          }
        }
      }
    }
  }
  
  RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                        "Inflation layer updated (radius: %.2fm)", inflation_radius_);
}

void CostmapNode::combine_layers() {
  // Combine all layers into final costmap
  // Priority: Static > Obstacle > Inflation
  for (size_t i = 0; i < costmap_data_.size(); ++i) {
    // Start with static layer or unknown
    int8_t cost = static_layer_[i];
    
    // Override with obstacle layer if it has obstacles
    if (obstacle_layer_[i] >= static_cast<int8_t>(50)) {
      cost = std::max(cost, obstacle_layer_[i]);
    } else if (obstacle_layer_[i] == static_cast<int8_t>(CostValues::FREE_SPACE) && 
               cost == static_cast<int8_t>(CostValues::NO_INFORMATION)) {
      // Mark as free if obstacle layer cleared it
      cost = static_cast<int8_t>(CostValues::FREE_SPACE);
    }
    
    // Add inflation costs
    if (inflation_layer_[i] > cost && cost < static_cast<int8_t>(CostValues::LETHAL_OBSTACLE)) {
      cost = std::max(cost, inflation_layer_[i]);
    }
    
    costmap_data_[i] = cost;
  }
}

void CostmapNode::raytrace_line(int x0, int y0, int x1, int y1, std::vector<int8_t>& layer) {
  // Bresenham's line algorithm to trace ray and clear free space
  int dx = std::abs(x1 - x0);
  int dy = std::abs(y1 - y0);
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;

  int x = x0;
  int y = y0;

  while (true) {
    // Don't clear the endpoint (that's the obstacle)
    if (x == x1 && y == y1) {
      break;
    }

    // Mark this cell as free (ray passed through it)
    if (x >= 0 && x < width_ && y >= 0 && y < height_) {
      unsigned int idx = get_index(x, y);
      if (layer[idx] != static_cast<int8_t>(CostValues::LETHAL_OBSTACLE)) {
        layer[idx] = static_cast<int8_t>(CostValues::FREE_SPACE);
      }
    }

    int e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      x += sx;
    }
    if (e2 < dx) {
      err += dx;
      y += sy;
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

  // Create map info (shared by all layers)
  nav_msgs::msg::MapMetaData map_info;
  map_info.resolution = resolution_;
  map_info.width = width_;
  map_info.height = height_;
  map_info.origin.position.x = costmap_origin_x;
  map_info.origin.position.y = costmap_origin_y;
  map_info.origin.position.z = 0.0;
  map_info.origin.orientation.w = 1.0;

  // Publish main costmap
  auto costmap_msg = std::make_unique<nav_msgs::msg::OccupancyGrid>();
  costmap_msg->header.stamp = stamp;
  costmap_msg->header.frame_id = global_frame_;
  costmap_msg->info = map_info;
  costmap_msg->data = costmap_data_;
  costmap_pub_->publish(std::move(costmap_msg));

  // Publish individual layers for debugging/visualization
  auto publish_layer = [&](const std::vector<int8_t>& layer_data,
                          rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr& pub) {
    auto layer_msg = std::make_unique<nav_msgs::msg::OccupancyGrid>();
    layer_msg->header.stamp = stamp;
    layer_msg->header.frame_id = global_frame_;
    layer_msg->info = map_info;
    layer_msg->data = layer_data;
    pub->publish(std::move(layer_msg));
  };

  publish_layer(static_layer_, static_layer_pub_);
  publish_layer(obstacle_layer_, obstacle_layer_pub_);
  publish_layer(inflation_layer_, inflation_layer_pub_);
  
  RCLCPP_INFO_ONCE(this->get_logger(), 
                   "Publishing costmap with layered architecture (frame: %s)", 
                   global_frame_.c_str());
  
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
