#include "planner_node.hpp"
#include <cmath>
#include <chrono>
#include <rclcpp/rclcpp.hpp>

PlannerNode::PlannerNode()
    : Node("planner_node"),
      planner_(robot::PlannerCore(this->get_logger())),
      robot_position_x_(0.0),
      robot_position_y_(0.0),
      has_active_goal_(false),
      has_odometry_(false) {

    initializeParameters();

    map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic_, 10,
        std::bind(&PlannerNode::handleMapUpdate, this, std::placeholders::_1));

    goal_subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        goal_topic_, 10,
        std::bind(&PlannerNode::handleNewGoal, this, std::placeholders::_1));

    odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 10,
        std::bind(&PlannerNode::handleOdometry, this, std::placeholders::_1));

    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>(path_topic_, 10);

    update_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&PlannerNode::periodicCheck, this));

    planner_.configurePlanner(smoothing_factor_, max_iterations_);
}

void PlannerNode::initializeParameters() {
    this->declare_parameter<std::string>("map_topic", "/map");
    this->declare_parameter<std::string>("goal_topic", "/goal_pose");
    this->declare_parameter<std::string>("odom_topic", "/odom/filtered");
    this->declare_parameter<std::string>("path_topic", "/planned_path");
    this->declare_parameter<double>("smoothing_factor", 0.2);
    this->declare_parameter<int>("max_iterations", 25);
    this->declare_parameter<double>("timeout_limit", 10.0);
    this->declare_parameter<double>("goal_tolerance", 0.25);

    map_topic_ = this->get_parameter("map_topic").as_string();
    goal_topic_ = this->get_parameter("goal_topic").as_string();
    odom_topic_ = this->get_parameter("odom_topic").as_string();
    path_topic_ = this->get_parameter("path_topic").as_string();
    smoothing_factor_ = this->get_parameter("smoothing_factor").as_double();
    max_iterations_ = this->get_parameter("max_iterations").as_int();
    timeout_limit_ = this->get_parameter("timeout_limit").as_double();
    goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
}

void PlannerNode::handleMapUpdate(const nav_msgs::msg::OccupancyGrid::SharedPtr map) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_map_ = map;

    if (has_active_goal_) {
        double elapsed_time = (this->now() - goal_start_time_).seconds();
        if (elapsed_time <= timeout_limit_) {
            RCLCPP_INFO(this->get_logger(), "Replanning for active goal.");
            attemptPathPlanning();
        }
    }
}

void PlannerNode::handleNewGoal(const geometry_msgs::msg::PointStamped::SharedPtr goal_msg) {
    if (has_active_goal_) {
        RCLCPP_WARN(this->get_logger(), "New goal ignored; an active goal exists.");
        return;
    }

    if (!current_map_) {
        RCLCPP_WARN(this->get_logger(), "No map available; cannot process new goal.");
        return;
    }

    active_goal_ = *goal_msg;
    has_active_goal_ = true;
    goal_start_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "Received new goal at (%.2f, %.2f).", goal_msg->point.x, goal_msg->point.y);

    attemptPathPlanning();
}

void PlannerNode::handleOdometry(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
    robot_position_x_ = odom_msg->pose.pose.position.x;
    robot_position_y_ = odom_msg->pose.pose.position.y;
    has_odometry_ = true;
}

void PlannerNode::periodicCheck() {
    if (!has_active_goal_) {
        return;
    }

    double elapsed_time = (this->now() - goal_start_time_).seconds();
    if (elapsed_time > timeout_limit_) {
        RCLCPP_WARN(this->get_logger(), "Path planning timed out. Resetting goal.");
        clearCurrentGoal();
        return;
    }

    double distance_to_goal = std::hypot(
        robot_position_x_ - active_goal_.point.x,
        robot_position_y_ - active_goal_.point.y);

    if (distance_to_goal < goal_tolerance_) {
        RCLCPP_INFO(this->get_logger(), "Goal reached successfully.");
        clearCurrentGoal();
    }
}

void PlannerNode::attemptPathPlanning() {
    if (!has_odometry_) {
        RCLCPP_WARN(this->get_logger(), "Odometry unavailable. Cannot plan path.");
        clearCurrentGoal();
        return;
    }

    double start_x = robot_position_x_;
    double start_y = robot_position_y_;

    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (!planner_.computePath(start_x, start_y, active_goal_.point.x, active_goal_.point.y, current_map_)) {
            RCLCPP_ERROR(this->get_logger(), "Path planning failed.");
            clearCurrentGoal();
            return;
        }
    }

    auto planned_path = planner_.retrievePath();
    planned_path->header.stamp = this->now();
    planned_path->header.frame_id = current_map_->header.frame_id;

    path_publisher_->publish(*planned_path);
}

void PlannerNode::clearCurrentGoal() {
    has_active_goal_ = false;
    RCLCPP_INFO(this->get_logger(), "Active goal cleared.");

    nav_msgs::msg::Path empty_path;
    empty_path.header.stamp = this->now();
    empty_path.header.frame_id = current_map_ ? current_map_->header.frame_id : "map";

    path_publisher_->publish(empty_path);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlannerNode>());
    rclcpp::shutdown();
    return 0;
}