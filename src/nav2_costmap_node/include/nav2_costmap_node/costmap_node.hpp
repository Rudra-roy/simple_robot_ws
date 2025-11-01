#ifndef NAV2_COSTMAP_NODE__COSTMAP_NODE_HPP_
#define NAV2_COSTMAP_NODE__COSTMAP_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace nav2_costmap_node {

// Costmap layer values
enum class CostValues : uint8_t {
  FREE_SPACE = 0,
  INSCRIBED_INFLATED_OBSTACLE = 253,
  LETHAL_OBSTACLE = 254,
  NO_INFORMATION = 255
};

class CostmapNode : public rclcpp::Node {
 public:
  explicit CostmapNode();
  ~CostmapNode() = default;

 private:
  void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void publish_costmap();
  
  // Simple processing functions
  void update_obstacles_simple(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void mark_obstacle_with_inflation(int mx, int my);
  
  // Utility functions
  bool world_to_map(double wx, double wy, int& mx, int& my) const;
  void map_to_world(int mx, int my, double& wx, double& wy) const;
  unsigned int get_index(int mx, int my) const;
  uint8_t compute_cost(int8_t base_cost) const;

  // Parameters
  std::string global_frame_;
  std::string robot_frame_;
  double resolution_;
  int width_;
  int height_;
  double origin_x_;
  double origin_y_;
  double inflation_radius_;
  double cost_scaling_factor_;
  bool track_unknown_space_;
  bool use_static_map_;
  double obstacle_max_range_;
  double obstacle_min_range_;

  // Subscriptions and Publications
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // TF2
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Simple costmap data
  std::vector<int8_t> costmap_data_;      // Single layer costmap
  
  // Static map data
  nav_msgs::msg::OccupancyGrid::SharedPtr static_map_;
  bool static_map_received_;
  
  // Robot position for ray tracing
  double robot_x_, robot_y_;
};

}  // namespace nav2_costmap_node

#endif  // NAV2_COSTMAP_NODE__COSTMAP_NODE_HPP_
