#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace robot_control {

class ControlCore {
public:
    explicit ControlCore(const rclcpp::Logger& logger);

    void initialize(double lookahead_dist, double max_steering_angle, double steering_gain, double base_velocity);

    void updatePath(const nav_msgs::msg::Path& new_path);

    bool hasPath() const;

    geometry_msgs::msg::Twist calculateVelocityCommand(const geometry_msgs::msg::Point& position, double orientation) const;

private:
    int findTargetPointIndex(const geometry_msgs::msg::Point& position, double orientation) const;

    double adjustAngle(double angle) const;

    rclcpp::Logger logger_;
    nav_msgs::msg::Path path_;

    double lookahead_distance_;
    double max_steering_angle_;
    double steering_gain_;
    double base_velocity_;
};

} // namespace robot_control

#endif // CONTROL_CORE_HPP_
