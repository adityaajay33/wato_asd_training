#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <optional>
#include <cmath>

namespace robot {

class ControlCore {
public:
    ControlCore(double lookahead_distance, double goal_tolerance, double linear_speed);

    std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint(
        const nav_msgs::msg::Path &path, const geometry_msgs::msg::Pose &robot_pose);

    geometry_msgs::msg::Twist calculateControlCommand(
        const geometry_msgs::msg::PoseStamped &target, const geometry_msgs::msg::Pose &robot_pose);

private:
    double computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b);
    double extractYaw(const geometry_msgs::msg::Quaternion &quat);

    double lookahead_distance_;
    double goal_tolerance_;
    double linear_speed_;
};

} 

#endif
