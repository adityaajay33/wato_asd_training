#ifndef MEMORY_MAPPER_NODE_HPP_
#define MEMORY_MAPPER_NODE_HPP_

#include <memory>
#include <vector>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "map_memory_core.hpp"

// MemoryMapperNode
// ----------------
// ROS 2 node that merges local costmaps into a global map after the robot
// has traveled a specified distance, publishing updates at a fixed rate.

class MemoryMapperNode : public rclcpp::Node {
public:
  MemoryMapperNode();

  // Subscribed callbacks
  void onLocalMapReceived(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void onOdomReceived(const nav_msgs::msg::Odometry::SharedPtr msg);

  // Timer event
  void onTimerEvent();

  // Convert quaternion to yaw
  double quaternionToYaw(double x, double y, double z, double w);

  void loadParams();

private:
  // Core logic for the global map
  memory_ns::MapMemoryHandler mem_handler_;

  // ROS 2 handles
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
  rclcpp::TimerBase::SharedPtr update_timer_;

  // Parameter-based topics
  std::string local_costmap_topic_;
  std::string odom_topic_;
  std::string merged_map_topic_;

  // Update frequency and distance threshold
  int publish_rate_ms_;
  double distance_threshold_;

  // Global map configuration
  double global_res_;
  int global_width_;
  int global_height_;
  geometry_msgs::msg::Pose global_origin_;

  // Robot pose tracking
  double robot_px_;
  double robot_py_;
  double robot_yaw_;

  // Storing last position used for distance checks
  double prev_x_;
  double prev_y_;
};

#endif  // MEMORY_MAPPER_NODE_HPP_
