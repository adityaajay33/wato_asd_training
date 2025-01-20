#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace robot {

class ControlCore {
public:
    ControlCore(const rclcpp::Logger& logger, double lookahead_distance, double goal_tolerance, double linear_velocity);

    void updatePath(const nav_msgs::msg::Path& path);

    unsigned int findLookaheadPoint(const nav_msgs::msg::Path& path, const geometry_msgs::msg::Point& robot_position) const;

    geometry_msgs::msg::Twist computeVelocity(const geometry_msgs::msg::PoseStamped& target, const geometry_msgs::msg::Point& robot_position, double robot_orientation) const;

private:
    nav_msgs::msg::Path path_;
    rclcpp::Logger logger_;
    double lookahead_distance_;
    double goal_tolerance_;
    double linear_velocity_;
};

}  // namespace robot

#endif  // CONTROL_CORE_HPP_
