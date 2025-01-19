#include "control_core.hpp"

namespace robot {

PurePursuitCore::PurePursuitCore(double lookahead_distance, double goal_tolerance, double linear_speed)
    : lookahead_distance_(lookahead_distance),
      goal_tolerance_(goal_tolerance),
      linear_speed_(linear_speed) {}

std::optional<geometry_msgs::msg::PoseStamped> PurePursuitCore::findLookaheadPoint(
    const nav_msgs::msg::Path &path, const geometry_msgs::msg::Pose &robot_pose) {

    for (const auto &pose : path.poses) {
        double distance = computeDistance(robot_pose.position, pose.pose.position);
        if (distance >= lookahead_distance_) {
            return pose;
        }
    }

    return std::nullopt;
}

geometry_msgs::msg::Twist PurePursuitCore::computeVelocity(
    const geometry_msgs::msg::PoseStamped &target, const geometry_msgs::msg::Pose &robot_pose) {

    geometry_msgs::msg::Twist cmd_vel;

    double robot_yaw = extractYaw(robot_pose.orientation);

    double dx = target.pose.position.x - robot_pose.position.x;
    double dy = target.pose.position.y - robot_pose.position.y;

    double target_angle = std::atan2(dy, dx);
    double angle_error = target_angle - robot_yaw;

    while (angle_error > M_PI) {
        angle_error -= 2.0 * M_PI;
    }
    while (angle_error < -M_PI) {
        angle_error += 2.0 * M_PI;
    }

    cmd_vel.linear.x = linear_speed_;
    cmd_vel.angular.z = 2.0 * angle_error;

    return cmd_vel;
}

double PurePursuitCore::computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
    return std::sqrt(std::pow(b.x - a.x, 2) + std::pow(b.y - a.y, 2));
}

double PurePursuitCore::extractYaw(const geometry_msgs::msg::Quaternion &quat) {
    double siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y);
    double cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

} // namespace robot
