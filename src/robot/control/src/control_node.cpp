#include "control_node.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

ControlNode::ControlNode() 
    : Node("control_node"), controller_(robot_control::ControlCore(this->get_logger())) 
{
    declareNodeParameters();

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        path_topic_, rclcpp::QoS(10), std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, rclcpp::QoS(10), std::bind(&ControlNode::odometryCallback, this, std::placeholders::_1));

    velocity_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(velocity_topic_, rclcpp::QoS(10));

    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(control_frequency_),
        std::bind(&ControlNode::controlLoop, this));
}

void ControlNode::declareNodeParameters() {
    this->declare_parameter<std::string>("path_topic", "/path");
    this->declare_parameter<std::string>("odom_topic", "/odom");
    this->declare_parameter<std::string>("velocity_topic", "/cmd_vel");
    this->declare_parameter<int>("control_frequency", 100);
    this->declare_parameter<double>("lookahead_distance", 1.0);
    this->declare_parameter<double>("steering_gain", 1.0);
    this->declare_parameter<double>("max_steering_angle", 1.0);
    this->declare_parameter<double>("base_velocity", 0.5);

    path_topic_ = this->get_parameter("path_topic").as_string();
    odom_topic_ = this->get_parameter("odom_topic").as_string();
    velocity_topic_ = this->get_parameter("velocity_topic").as_string();
    control_frequency_ = this->get_parameter("control_frequency").as_int();

    double lookahead_distance = this->get_parameter("lookahead_distance").as_double();
    double steering_gain = this->get_parameter("steering_gain").as_double();
    double max_steering_angle = this->get_parameter("max_steering_angle").as_double();
    double base_velocity = this->get_parameter("base_velocity").as_double();

    controller_.initialize(lookahead_distance, max_steering_angle, steering_gain, base_velocity);
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr path_msg) {
    controller_.updatePath(*path_msg);
}

void ControlNode::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
    current_position_.x = odom_msg->pose.pose.position.x;
    current_position_.y = odom_msg->pose.pose.position.y;
    current_orientation_ = quaternionToYaw(odom_msg->pose.pose.orientation);
}

void ControlNode::controlLoop() {
    if (!controller_.hasPath()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for path...");
        return;
    }

    auto velocity_command = controller_.calculateVelocityCommand(current_position_, current_orientation_);
    velocity_pub_->publish(velocity_command);
}

double ControlNode::quaternionToYaw(const geometry_msgs::msg::Quaternion& quat) const {
    tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf2::Matrix3x3 rotation(q);
    double roll, pitch, yaw;
    rotation.getRPY(roll, pitch, yaw);
    return yaw;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}
