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

private:
    void declareNodeParameters();
    void controlLoop();
    void pathCallback(const nav_msgs::msg::Path::SharedPtr path_msg);
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
    double quaternionToYaw(const geometry_msgs::msg::Quaternion& quat) const;

    robot_control::ControlCore controller_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    geometry_msgs::msg::Point current_position_;
    double current_orientation_ = 0.0;

    std::string path_topic_;
    std::string odom_topic_;
    std::string velocity_topic_;
    int control_frequency_;
};

#endif // CONTROL_NODE_HPP_
