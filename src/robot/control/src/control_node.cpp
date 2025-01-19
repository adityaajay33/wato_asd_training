#include "control_node.hpp"

ControlNode::ControlNode() : Node("control_node"),
    controller_(1.0, 0.1, 0.5) {
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/path", 10, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10, std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&ControlNode::timerCallback, this));
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    current_path_ = *msg;
    path_received_ = true;
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    robot_pose_ = msg->pose.pose;
}

void ControlNode::timerCallback() {
    if (!path_received_ || current_path_.poses.empty()) {
        return;
    }

    auto lookahead_point = controller_.findLookaheadPoint(current_path_, robot_pose_);
    if (lookahead_point) {
        auto cmd_vel = controller_.computeVelocity(*lookahead_point, robot_pose_);
        cmd_vel_pub_->publish(cmd_vel);
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}
