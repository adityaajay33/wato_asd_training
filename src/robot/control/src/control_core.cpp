#include "control_core.hpp"
#include <cmath>
#include <algorithm>

namespace robot {

ControlCore::ControlCore(const rclcpp::Logger& logger, double lookahead_distance, double goal_tolerance, double linear_velocity)
    : logger_(logger), lookahead_distance_(lookahead_distance), goal_tolerance_(goal_tolerance), linear_velocity_(linear_velocity) {}

void ControlCore::updatePath(const nav_msgs::msg::Path& path) {
    RCLCPP_INFO(logger_, "Path updated with %zu poses.", path.poses.size());
    path_ = path;
}

unsigned int ControlCore::findLookaheadPoint(const nav_msgs::msg::Path& path, const geometry_msgs::msg::Point& robot_position) const {
    double min_distance = std::numeric_limits<double>::max();
    int lookahead_index = -1;

    for (size_t i = 0; i < path.poses.size(); ++i) {
        double dx = path.poses[i].pose.position.x - robot_position.x;
        double dy = path.poses[i].pose.position.y - robot_position.y;
        double distance = std::hypot(dx, dy);

        if (distance >= lookahead_distance_ && distance < min_distance) {
            min_distance = distance;
            lookahead_index = i;
        }
    }

    if (lookahead_index == -1) {
        RCLCPP_WARN(logger_, "No valid lookahead point found.");
    }

    return lookahead_index;
}

geometry_msgs::msg::Twist ControlCore::computeVelocity(const geometry_msgs::msg::PoseStamped& target, const geometry_msgs::msg::Point& robot_position, double robot_orientation) const {
    geometry_msgs::msg::Twist cmd_vel;

    double target_angle = std::atan2(target.pose.position.y - robot_position.y, target.pose.position.x - robot_position.x);
    double angle_error = target_angle - robot_orientation;

    while (angle_error > M_PI) angle_error -= 2 * M_PI;
    while (angle_error < -M_PI) angle_error += 2 * M_PI;

    if (std::hypot(target.pose.position.x - robot_position.x, target.pose.position.y - robot_position.y) <= goal_tolerance_) {
        RCLCPP_INFO(logger_, "Goal reached. Stopping the robot.");
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
    } else {
        cmd_vel.linear.x = linear_velocity_;
        cmd_vel.angular.z = angle_error;
    }

    return cmd_vel;
}

}  // namespace robot
