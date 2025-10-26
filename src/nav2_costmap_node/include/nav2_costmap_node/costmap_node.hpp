#ifndef NAV2_COSTMAP_NODE__COSTMAP_NODE_HPP_
#define NAV2_COSTMAP_NODE__COSTMAP_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
namespace nav2_costmap_node {

class CostmapNode : public rclcpp::Node {
 public:
  explicit CostmapNode();
  ~CostmapNode() = default;

 private:
  void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void publish_costmap();

  // Parameters
  std::string global_frame_;
  std::string robot_frame_;
  double resolution_;
  int width_;
  int height_;
  double origin_x_;
  double origin_y_;
  double inflation_radius_;

  // Subscriptions and Publications
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // TF2
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Costmap data
  std::vector<int8_t> costmap_data_;
};

}  // namespace nav2_costmap_node

#endif  // NAV2_COSTMAP_NODE__COSTMAP_NODE_HPP_
