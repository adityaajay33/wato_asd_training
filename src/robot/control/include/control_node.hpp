#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include "control_core.hpp"
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>

class ControlNode : public rclcpp::Node {
public:
    ControlNode();

private:
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timerCallback();

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    robot::PurePursuitCore controller_;

    nav_msgs::msg::Path current_path_;
    geometry_msgs::msg::Pose robot_pose_;

    bool path_received_ = false;
};
#endif
