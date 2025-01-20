#include "control_core.hpp"
#include <cmath>
#include <algorithm>

namespace robot_control {

ControlCore::ControlCore(const rclcpp::Logger& logger)
    : logger_(logger), path_() {}

void ControlCore::initialize(double lookahead_dist, double max_steering_angle, double steering_gain, double base_velocity) {
    lookahead_distance_ = lookahead_dist;
    max_steering_angle_ = max_steering_angle;
    steering_gain_ = steering_gain;
    base_velocity_ = base_velocity;
}

void ControlCore::updatePath(const nav_msgs::msg::Path& new_path) {
    RCLCPP_INFO(logger_, "Path updated with %zu points.", new_path.poses.size());
    path_ = new_path;
}

bool ControlCore::hasPath() const {
    return !path_.poses.empty();
}

geometry_msgs::msg::Twist ControlCore::calculateVelocityCommand(const geometry_msgs::msg::Point& position, double orientation) const {
    geometry_msgs::msg::Twist cmd;

    int target_index = findTargetPointIndex(position, orientation);
    if (target_index == -1) {
        RCLCPP_WARN(logger_, "No valid target point found. Stopping robot.");
        return cmd;
    }

    const auto& target_point = path_.poses[target_index].pose.position;

    double dx = target_point.x - position.x;
    double dy = target_point.y - position.y;

    double target_angle = std::atan2(dy, dx);
    double angle_diff = adjustAngle(target_angle - orientation);

    if (std::abs(angle_diff) > max_steering_angle_) {
        cmd.linear.x = 0.0;
    } else {
        cmd.linear.x = base_velocity_;
    }

    cmd.angular.z = angle_diff * steering_gain_;
    return cmd;
}

int ControlCore::findTargetPointIndex(const geometry_msgs::msg::Point& position, double orientation) const {
    double min_distance = std::numeric_limits<double>::max();
    int closest_index = -1;

    for (size_t i = 0; i < path_.poses.size(); ++i) {
        double dx = path_.poses[i].pose.position.x - position.x;
        double dy = path_.poses[i].pose.position.y - position.y;

        double distance = std::hypot(dx, dy);

        if (distance >= lookahead_distance_ && distance < min_distance) {
            min_distance = distance;
            closest_index = static_cast<int>(i);
        }
    }

    if (closest_index == -1) {
        RCLCPP_WARN(logger_, "No target point found within lookahead distance.");
    }

    return closest_index;
}

double ControlCore::adjustAngle(double angle) const {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

} // namespace robot_control
