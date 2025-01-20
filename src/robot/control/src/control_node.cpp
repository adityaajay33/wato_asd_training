#include "control_node.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

ControlNode::ControlNode()
    : Node("control_node"),
      core_(
          get_logger(),
          declare_parameter<double>("lookahead_distance", 1.0),
          declare_parameter<double>("goal_tolerance", 0.1),
          declare_parameter<double>("linear_speed", 0.5)) {
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) { current_path_ = msg; });

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { robot_odom_ = msg; });

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), [this]() { controlLoop(); });
}

void ControlNode::controlLoop() {
    if (!current_path_ || !robot_odom_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000, "Waiting for path or odometry data...");
        return;
    }

    robot_yaw_ = extractYaw(robot_odom_->pose.pose.orientation);

    unsigned int lookahead_index = core_.findLookaheadPoint(*current_path_, robot_odom_->pose.pose.position);

    if (lookahead_index >= current_path_->poses.size()) {
        RCLCPP_WARN(get_logger(), "No valid lookahead point found. Stopping the robot.");
        stopRobot();
        return;
    }

    const auto& lookahead_point = current_path_->poses[lookahead_index];
    auto cmd_vel = core_.computeVelocity(lookahead_point, robot_odom_->pose.pose.position, robot_yaw_);
    cmd_vel_pub_->publish(cmd_vel);
}

void ControlNode::stopRobot() {
    geometry_msgs::msg::Twist stop_msg;
    stop_msg.linear.x = 0.0;
    stop_msg.angular.z = 0.0;
    cmd_vel_pub_->publish(stop_msg);
}

double ControlNode::extractYaw(const geometry_msgs::msg::Quaternion& quat) const {
    tf2::Quaternion tf_quat(quat.x, quat.y, quat.z, quat.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
    return yaw;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}
