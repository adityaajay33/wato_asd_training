#include <memory>
#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
    initializeSettings();

    laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        laser_input_topic_, rclcpp::QoS(10),
        std::bind(&CostmapNode::processLaserData, this, std::placeholders::_1));

    grid_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(grid_output_topic_, rclcpp::QoS(10));

    costmap_.initializeCostmap(grid_resolution_, grid_width_, grid_height_, grid_origin_);
}

void CostmapNode::initializeSettings() {
    laser_input_topic_ = retrieveParameter<std::string>("laser.input_topic", "/scan");
    grid_output_topic_ = retrieveParameter<std::string>("grid.output_topic", "/grid_map");
    grid_resolution_ = retrieveParameter<double>("grid.resolution", 0.05);
    grid_width_ = retrieveParameter<int>("grid.dimensions.width", 150);
    grid_height_ = retrieveParameter<int>("grid.dimensions.height", 150);
    grid_origin_.position.x = retrieveParameter<double>("grid.origin.x", -7.5);
    grid_origin_.position.y = retrieveParameter<double>("grid.origin.y", -7.5);
    grid_origin_.orientation.w = retrieveParameter<double>("grid.origin.orientation", 1.0);
    obstacle_expansion_radius_ = retrieveParameter<double>("grid.expansion.radius", 0.75);
}

template <typename T>
T CostmapNode::retrieveParameter(const std::string &key, const T &default_value) {
    this->declare_parameter<T>(key, default_value);
    return this->get_parameter(key).template get_value<T>();
}

void CostmapNode::processLaserData(const sensor_msgs::msg::LaserScan::SharedPtr laser_scan) {
    if (laser_scan->ranges.empty()) {
        RCLCPP_WARN(this->get_logger(), "Empty LaserScan received, no update performed.");
        return;
    }
    costmap_.updateFromLaserScan(laser_scan);

    auto grid_message = costmap_.getCostmap();
    grid_message->header.stamp = this->now();
    grid_message->header.frame_id = "map";
    grid_publisher_->publish(*grid_message);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CostmapNode>());
    rclcpp::shutdown();
    return 0;
}
