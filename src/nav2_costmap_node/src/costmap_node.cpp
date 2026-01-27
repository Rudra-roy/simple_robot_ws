// costmap_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

namespace nav2_costmap_node
{

class CostmapNode : public rclcpp::Node
{
public:
  CostmapNode()
  : rclcpp::Node("costmap_node"),
    static_map_received_(false),
    robot_x_(0.0), robot_y_(0.0)
  {
    // Parameters
    this->declare_parameter("global_frame_id", std::string("odom"));
    this->declare_parameter("base_frame_id",   std::string("base_link"));
    this->declare_parameter("resolution", 0.05);            // m/cell
    this->declare_parameter("width", 1000);                  // cells
    this->declare_parameter("height", 1000);                 // cells
    this->declare_parameter("origin_x", 0.0);
    this->declare_parameter("origin_y", 0.0);
    this->declare_parameter("inflation_radius", 0.30);
    this->declare_parameter("cost_scaling_factor", 10.0);
    this->declare_parameter("track_unknown_space", true);
    this->declare_parameter("use_static_map", false);
    this->declare_parameter("obstacle_max_range", 20.0);    // was 5.0
    this->declare_parameter("obstacle_min_range", 0.8);
    this->declare_parameter("min_obstacle_z", 0.25);        // Ground filter: must be above 25cm
    this->declare_parameter("max_obstacle_z", 4.00);        // Max height: 2m ceiling
    this->declare_parameter("max_traversable_slope", 20.0);  // Max slope in degrees for traversability
    this->declare_parameter("point_cloud_topic", std::string("/depth_points"));
    this->declare_parameter("costmap_topic",     std::string("/costmap"));
    this->declare_parameter("static_map_topic",  std::string("/map"));
    this->declare_parameter("publish_frequency", 10.0);

    // Get params
    global_frame_          = this->get_parameter("global_frame_id").as_string();
    robot_frame_           = this->get_parameter("base_frame_id").as_string();
    resolution_            = this->get_parameter("resolution").as_double();
    width_                 = this->get_parameter("width").as_int();
    height_                = this->get_parameter("height").as_int();
    origin_x_              = this->get_parameter("origin_x").as_double(); // not used (centered grid)
    origin_y_              = this->get_parameter("origin_y").as_double();
    inflation_radius_      = this->get_parameter("inflation_radius").as_double();
    cost_scaling_factor_   = this->get_parameter("cost_scaling_factor").as_double();
    track_unknown_space_   = this->get_parameter("track_unknown_space").as_bool();
    use_static_map_        = this->get_parameter("use_static_map").as_bool();
    obstacle_max_range_    = this->get_parameter("obstacle_max_range").as_double();
    obstacle_min_range_    = this->get_parameter("obstacle_min_range").as_double();
    min_obstacle_z_        = this->get_parameter("min_obstacle_z").as_double();
    max_obstacle_z_        = this->get_parameter("max_obstacle_z").as_double();
    max_traversable_slope_ = this->get_parameter("max_traversable_slope").as_double();

    double publish_freq    = this->get_parameter("publish_frequency").as_double();
    std::string pc_topic   = this->get_parameter("point_cloud_topic").as_string();
    std::string cost_topic = this->get_parameter("costmap_topic").as_string();
    std::string map_topic  = this->get_parameter("static_map_topic").as_string();

    // Allocate costmap (0 = free). If you want "unknown", initialize with -1 instead.
    costmap_data_.assign(width_ * height_, 0);
    hit_count_.assign(width_ * height_, 0);  // Track point hits per cell

    // TF
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // I/O
    pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      pc_topic, rclcpp::SensorDataQoS(),
      std::bind(&CostmapNode::point_cloud_callback, this, std::placeholders::_1));

    if (use_static_map_) {
      map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic, rclcpp::QoS(10).transient_local(),
        std::bind(&CostmapNode::map_callback, this, std::placeholders::_1));
    }

    costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(cost_topic, 10);

    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(1.0, publish_freq)),
      std::bind(&CostmapNode::publish_costmap, this));

    RCLCPP_INFO(get_logger(), "Simple costmap node up. Frame=%s base=%s res=%.3f size=%dx%d",
                global_frame_.c_str(), robot_frame_.c_str(), resolution_, width_, height_);
  }

private:
  // ==== Callbacks ====
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    RCLCPP_INFO(get_logger(), "Received static map: %ux%u @ %.3fm",
                msg->info.width, msg->info.height, msg->info.resolution);
    static_map_ = msg;
    static_map_received_ = true;

    // Naive overlay: copy lethal cells
    const size_t N = std::min(static_map_->data.size(), costmap_data_.size());
    for (size_t i = 0; i < N; ++i) {
      if (static_map_->data[i] > 50) {
        costmap_data_[i] = 100;
      }
    }
  }

  void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // Transform cloud frame -> base_link (for z-filtering)
    // Use the point cloud timestamp for ALL transforms
    geometry_msgs::msg::TransformStamped tf_cloud_to_base;
    try {
      tf_cloud_to_base = tf_buffer_->lookupTransform(
        robot_frame_, msg->header.frame_id, msg->header.stamp, tf2::durationFromSec(0.1));
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "TF (cloud->%s): %s", robot_frame_.c_str(), ex.what());
      return;
    }

    // Transform cloud frame -> global frame (for mapping)
    geometry_msgs::msg::TransformStamped tf_cloud_to_global;
    try {
      tf_cloud_to_global = tf_buffer_->lookupTransform(
        global_frame_, msg->header.frame_id, msg->header.stamp, tf2::durationFromSec(0.1));
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "TF (cloud->%s): %s", global_frame_.c_str(), ex.what());
      return;
    }

    // Get robot pose at the SAME timestamp as the point cloud
    try {
      const auto tf = tf_buffer_->lookupTransform(global_frame_, robot_frame_, msg->header.stamp, tf2::durationFromSec(0.1));
      robot_x_ = tf.transform.translation.x;
      robot_y_ = tf.transform.translation.y;
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "TF (robot): %s", ex.what());
      return;
    }

    // Clear the entire costmap (rolling window approach)
    // Keep static map if present, otherwise clear everything
    if (use_static_map_ && static_map_received_) {
      // Restore static map obstacles only
      for (size_t i = 0; i < costmap_data_.size() && i < static_map_->data.size(); ++i) {
        if (static_map_->data[i] > 50) {
          costmap_data_[i] = 100;
        } else {
          costmap_data_[i] = 0;
        }
      }
    } else {
      // Clear everything for fresh update
      std::fill(costmap_data_.begin(), costmap_data_.end(), 0);
    }
    
    // Clear hit counter
    std::fill(hit_count_.begin(), hit_count_.end(), 0);

    // Iterate points
    sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");

    int points_marked = 0;
    int points_proc   = 0;

    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
      const float px = *it_x;
      const float py = *it_y;
      const float pz = *it_z;

      if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz)) {
        continue;
      }

      // First transform to base_link for z-height filtering
      geometry_msgs::msg::PointStamped pin, p_base;
      pin.header = msg->header;
      pin.point.x = px; pin.point.y = py; pin.point.z = pz;
      tf2::doTransform(pin, p_base, tf_cloud_to_base);

      const double bx = p_base.point.x;
      const double by = p_base.point.y;
      const double bz = p_base.point.z;

      // ===== FIRST: Filter points within 0.8m radius of robot =====
      // Remove all points too close to base_link (robot body + safety zone)
      const double range_base = std::sqrt(bx * bx + by * by);
      if (range_base < 0.8) {
        continue;  // Skip points within 0.8m radius
      }

      // ===== Filter ground points in base_link frame =====
      // Points are now relative to robot's base_link
      // Filter out ground (below min_z) and ceiling (above max_z)
      if (bz < min_obstacle_z_ || bz > max_obstacle_z_) {
        continue;  // Skip ground points and ceiling
      }

      // ===== Slope-based traversability filter =====
      // Apply ONLY to elevated terrain (not walls/vertical obstacles)
      // Calculate slope angle from robot base to point
      const double horizontal_dist = std::sqrt(bx * bx + by * by);
      if (horizontal_dist > 0.01) {  // Avoid division by zero
        const double slope_angle_rad = std::atan2(bz, horizontal_dist);
        const double slope_angle_deg = slope_angle_rad * 180.0 / M_PI;
        
        // Only filter gentle slopes on elevated terrain (0.35m - 1.0m height)
        // Vertical obstacles (walls, boxes) will have high slopes and pass through
        if (bz >= 0.35 && bz <= 1.0 && slope_angle_deg < max_traversable_slope_) {
          continue;  // Gently sloped elevated terrain - traversable
        }
        // Steep slopes (>=20°) or tall obstacles (>1m) = mark as obstacle
      }

      // Now transform to global frame for mapping
      geometry_msgs::msg::PointStamped p_global;
      tf2::doTransform(pin, p_global, tf_cloud_to_global);

      const double gx = p_global.point.x;
      const double gy = p_global.point.y;

      // 2D range from robot in global frame
      const double dx = gx - robot_x_;
      const double dy = gy - robot_y_;
      const double range = std::sqrt(dx * dx + dy * dy);
      if (range > obstacle_max_range_) {
        continue;  // Skip points beyond max range
      }

      int mx, my;
      if (!world_to_map(gx, gy, mx, my)) {
        continue;
      }

      // Increment hit count for this cell
      hit_count_[get_index(mx, my)]++;
      ++points_proc;
    }

    // Now mark obstacles only for cells with sufficient hits (filters spurious points)
    const int min_hits = 8;  // Increased from 3 to 8 to reduce false positives from noise
    for (int y = 0; y < height_; ++y) {
      for (int x = 0; x < width_; ++x) {
        const int idx = get_index(x, y);
        if (hit_count_[idx] >= min_hits) {
          mark_obstacle_with_inflation(x, y);
          ++points_marked;
        }
      }
    }

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Processed %d, marked %d (frame=%s)",
                         points_proc, points_marked, msg->header.frame_id.c_str());
  }

  // ==== Core grid ops ====
  void decay_dynamic_cells()
  {
    for (size_t i = 0; i < costmap_data_.size(); ++i) {
      int8_t c = costmap_data_[i];

      // Keep static map lethal if present
      if (use_static_map_ && static_map_received_ && i < static_map_->data.size()) {
        if (static_map_->data[i] > 50) {
          continue;
        }
      }

      if (c > 0) {
        // More aggressive decay - 30% persistence (was 90%)
        // This clears old obstacles faster
        int8_t n = static_cast<int8_t>(c * 0.3);
        if (n <= 5) n = 0;  // Clear cells below threshold 5
        costmap_data_[i] = n;
      }
    }
  }

  void mark_obstacle_with_inflation(int mx, int my)
  {
    if (mx < 0 || mx >= width_ || my < 0 || my >= height_) return;

    const int inflation_cells = static_cast<int>(std::ceil(inflation_radius_ / resolution_));
    // At least 0.15 m or 3 cells to get a solid core
    const int core_radius = std::max(3, static_cast<int>(std::ceil(0.15 / resolution_)));

    // Core lethal block
    for (int dy = -core_radius; dy <= core_radius; ++dy) {
      for (int dx = -core_radius; dx <= core_radius; ++dx) {
        const int nx = mx + dx;
        const int ny = my + dy;
        if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_) continue;

        const double dist_cells = std::sqrt(static_cast<double>(dx * dx + dy * dy));
        if (dist_cells <= core_radius) {
          costmap_data_[get_index(nx, ny)] = 100;
        }
      }
    }

    // Inflation halo
    for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
      for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
        const int nx = mx + dx;
        const int ny = my + dy;
        if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_) continue;

        const double dist = std::sqrt(static_cast<double>(dx * dx + dy * dy)) * resolution_;
        const auto idx = get_index(nx, ny);

        if (costmap_data_[idx] == 100) continue; // already lethal core
        if (dist <= inflation_radius_) {
          const double normalized = dist / inflation_radius_;
          const int8_t cost = static_cast<int8_t>(50 + (1.0 - normalized) * 49); // 50..99
          costmap_data_[idx] = std::max(costmap_data_[idx], cost);
        }
      }
    }
  }

  bool world_to_map(double wx, double wy, int &mx, int &my) const
  {
    const double origin_x = robot_x_ - (width_  * resolution_ / 2.0);
    const double origin_y = robot_y_ - (height_ * resolution_ / 2.0);

    mx = static_cast<int>((wx - origin_x) / resolution_);
    my = static_cast<int>((wy - origin_y) / resolution_);
    return (mx >= 0 && mx < width_ && my >= 0 && my < height_);
  }

  void map_to_world(int mx, int my, double &wx, double &wy) const
  {
    const double origin_x = robot_x_ - (width_  * resolution_ / 2.0);
    const double origin_y = robot_y_ - (height_ * resolution_ / 2.0);
    wx = origin_x + (mx + 0.5) * resolution_;
    wy = origin_y + (my + 0.5) * resolution_;
  }

  inline unsigned int get_index(int mx, int my) const
  {
    return static_cast<unsigned int>(my) * static_cast<unsigned int>(width_) + static_cast<unsigned int>(mx);
  }

  void publish_costmap()
  {
    // Update robot pose - get latest transform
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(global_frame_, robot_frame_, tf2::TimePointZero);
      robot_x_ = tf.transform.translation.x;
      robot_y_ = tf.transform.translation.y;
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "TF (robot): %s", ex.what());
      return;
    }
    
    // Use the TF timestamp for consistency with the transform
    const auto stamp = tf.header.stamp;
    const double origin_x = robot_x_ - (width_  * resolution_ / 2.0);
    const double origin_y = robot_y_ - (height_ * resolution_ / 2.0);

    auto msg = std::make_unique<nav_msgs::msg::OccupancyGrid>();
    msg->header.stamp = stamp;
    msg->header.frame_id = global_frame_;
    msg->info.resolution = resolution_;
    msg->info.width  = width_;
    msg->info.height = height_;
    msg->info.origin.position.x = origin_x;
    msg->info.origin.position.y = origin_y;
    msg->info.origin.position.z = 0.0;  // Costmap is at ground plane (z=0 in odom)
    msg->info.origin.orientation.x = 0.0;
    msg->info.origin.orientation.y = 0.0;
    msg->info.origin.orientation.z = 0.0;
    msg->info.origin.orientation.w = 1.0;
    msg->data = costmap_data_;

    costmap_pub_->publish(std::move(msg));
  }

private:
  // Params / state
  std::string global_frame_;
  std::string robot_frame_;
  double resolution_;
  int width_, height_;
  double origin_x_, origin_y_;        // not used when centering on robot
  double inflation_radius_;
  std::vector<int> hit_count_;  // Track point hits per cell to filter spurious points
  double cost_scaling_factor_;
  bool   track_unknown_space_;
  bool   use_static_map_;
  double obstacle_max_range_;
  double obstacle_min_range_;
  double min_obstacle_z_;
  double max_obstacle_z_;
  double max_traversable_slope_;  // Max traversable slope in degrees

  double robot_x_, robot_y_;

  // Data
  std::vector<int8_t> costmap_data_;
  nav_msgs::msg::OccupancyGrid::SharedPtr static_map_;
  bool static_map_received_;

  // ROS
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

} // namespace nav2_costmap_node

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<nav2_costmap_node::CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
