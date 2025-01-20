#include "control_node.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

ControlNode::ControlNode()
  : Node("control"), control_(robot::ControlCore(this->get_logger())),
    robot_x_(0.0), robot_y_(0.0), robot_theta_(0.0)
{
  processParameters();

  // Set up subscribers and publisher
  path_subscriber_ = this->create_subscription<nav_msgs::msg::Path>(
    path_topic_, 10,
    std::bind(&ControlNode::pathCallback, this, std::placeholders::_1)
  );

  odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, 10,
    std::bind(&ControlNode::odomCallback, this, std::placeholders::_1)
  );

  cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(control_period_ms_),
    std::bind(&ControlNode::timerCallback, this)
  );

  control_.initControlCore(lookahead_distance_, max_steering_angle_, steering_gain_, linear_velocity_);
}

void ControlNode::processParameters() {
  this->declare_parameter<std::string>("path_topic", "/path");
  this->declare_parameter<std::string>("odom_topic", "/odom/filtered");
  this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  this->declare_parameter<int>("control_period_ms", 100);
  this->declare_parameter<double>("lookahead_distance", 1.0);
  this->declare_parameter<double>("steering_gain", 1.5);
  this->declare_parameter<double>("max_steering_angle", 1.5);
  this->declare_parameter<double>("linear_velocity", 1.5);

  path_topic_ = this->get_parameter("path_topic").as_string();
  odom_topic_ = this->get_parameter("odom_topic").as_string();
  cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();
  control_period_ms_ = this->get_parameter("control_period_ms").as_int();
  lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
  steering_gain_ = this->get_parameter("steering_gain").as_double();
  max_steering_angle_ = this->get_parameter("max_steering_angle").as_double();
  linear_velocity_ = this->get_parameter("linear_velocity").as_double();
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  control_.updatePath(*msg);
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;
  robot_theta_ = quaternionToYaw(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w
  );
}

void ControlNode::followPath() {
  if (control_.isPathEmpty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "Path is empty, waiting for new data.");
    return;
  }
  geometry_msgs::msg::Twist command = control_.calculateControlCommand(robot_x_, robot_y_, robot_theta_);
  cmd_vel_publisher_->publish(command);
}

void ControlNode::timerCallback() {
  followPath();
}

double ControlNode::quaternionToYaw(double x, double y, double z, double w) {
  tf2::Quaternion q(x, y, z, w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
