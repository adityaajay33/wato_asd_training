#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp" // Added to support the use of rclcpp::Logger
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cmath>

namespace robot {

class ControlCore {
public:
    ControlCore(const rclcpp::Logger& logger); // Adjusted to match constructor with logger parameter

    void initControlCore(
        double lookahead_distance,
        double max_steering_angle,
        double steering_gain,
        double linear_velocity
    ); // Added initialization function for the control parameters

    void updatePath(nav_msgs::msg::Path path); // Added method for updating the path

    bool isPathEmpty(); // Added method to check if the path is empty

    unsigned int findLookaheadPoint(double robot_x, double robot_y, double robot_theta); // Updated to match signature

    geometry_msgs::msg::Twist calculateControlCommand(
        double robot_x, double robot_y, double robot_theta); // Updated signature to match robot's state parameters

private:
    nav_msgs::msg::Path path_; // Added member to store the current path
    rclcpp::Logger logger_; // Added logger for ROS2 logging

    double lookahead_distance_;
    double max_steering_angle_;
    double steering_gain_;
    double linear_velocity_;

    double computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b);
    double extractYaw(const geometry_msgs::msg::Quaternion &quat);
};

} // namespace robot

#endif
