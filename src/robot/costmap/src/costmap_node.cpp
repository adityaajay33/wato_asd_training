#include "costmap_node.hpp"

CostmapNode::CostmapNode()
    : Node("costmap_node"), costmap_(robot::CostmapCore(this->get_logger())) {
    loadParameters();

    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        laser_topic_, 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));

    costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(costmap_topic_, 10);

    costmap_.initialize(resolution_, width_, height_, origin_, inflation_radius_);

    RCLCPP_INFO(this->get_logger(), "CostmapNode initialized.");
}

void CostmapNode::loadParameters() {
    this->declare_parameter<std::string>("laser_topic", "/scan");
    this->declare_parameter<std::string>("costmap_topic", "/costmap");
    this->declare_parameter<double>("resolution", 0.1);
    this->declare_parameter<int>("width", 100);
    this->declare_parameter<int>("height", 100);
    this->declare_parameter<double>("origin_x", -5.0);
    this->declare_parameter<double>("origin_y", -5.0);
    this->declare_parameter<double>("origin_orientation_w", 1.0);
    this->declare_parameter<double>("inflation_radius", 1.0);

    laser_topic_ = this->get_parameter("laser_topic").as_string();
    costmap_topic_ = this->get_parameter("costmap_topic").as_string();
    resolution_ = this->get_parameter("resolution").as_double();
    width_ = this->get_parameter("width").as_int();
    height_ = this->get_parameter("height").as_int();
    origin_.position.x = this->get_parameter("origin_x").as_double();
    origin_.position.y = this->get_parameter("origin_y").as_double();
    origin_.orientation.w = this->get_parameter("origin_orientation_w").as_double();
    inflation_radius_ = this->get_parameter("inflation_radius").as_double();
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    costmap_.updateFromLaserScan(scan);

    auto costmap_message = costmap_.getCostmap();
    costmap_message->header.stamp = this->now();
    costmap_message->header.frame_id = "map";

    costmap_pub_->publish(*costmap_message);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CostmapNode>());
    rclcpp::shutdown();
    return 0;
}
