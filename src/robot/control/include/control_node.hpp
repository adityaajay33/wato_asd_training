#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "control_core.hpp"

class ControlNode : public rclcpp::Node {
public:
  ControlNode();

  // Load ROS2 parameters
  void processParameters();

  // Convert quaternion to yaw angle
  double quaternionToYaw(double x, double y, double z, double w);

  // Callback for receiving a path message
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);

  // Callback for receiving odometry data
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // Core routine to follow the path continuously
  void followPath();

  // Timer callback to trigger control updates
  void timerCallback();

private:
  robot::ControlCore control_;

  // Subscribers and publisher declarations
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Robot state variables
  double robot_x_;
  double robot_y_;
  double robot_theta_;

  // Parameter names and settings
  std::string path_topic_;
  std::string odom_topic_;
  std::string cmd_vel_topic_;
  int control_period_ms_;
  double lookahead_distance_;
  double steering_gain_;
  double max_steering_angle_;
  double linear_velocity_;
};

#endif  // CONTROL_NODE_HPP_
