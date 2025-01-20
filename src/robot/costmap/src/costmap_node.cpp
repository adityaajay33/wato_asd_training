#include <memory>
#include "costmap_node.hpp"

CostmapNode::CostmapNode()
: Node("costmap"),
  costmap_(robot::CostmapCore(this->get_logger()))
{
  processParameters();

  laser_scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    laserscan_topic_, 10,
    std::bind(&CostmapNode::laserScanCallback, this, std::placeholders::_1));

  costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(costmap_topic_, 10);

  RCLCPP_INFO(get_logger(), "ROS entities created");

  costmap_.initCostmap(resolution_, width_, height_, origin_, inflation_radius_);
  RCLCPP_INFO(get_logger(), "Costmap initialized");
}

void CostmapNode::processParameters() {
  declare_parameter<std::string>("laserscan_topic", "/lidar");
  declare_parameter<std::string>("costmap_topic", "/costmap");
  declare_parameter<double>("costmap.resolution", 0.1);
  declare_parameter<int>("costmap.width", 100);
  declare_parameter<int>("costmap.height", 100);
  declare_parameter<double>("costmap.origin.position.x", -5.0);
  declare_parameter<double>("costmap.origin.position.y", -5.0);
  declare_parameter<double>("costmap.origin.orientation.w", 1.0);
  declare_parameter<double>("costmap.inflation_radius", 1.0);

  laserscan_topic_ = get_parameter("laserscan_topic").as_string();
  costmap_topic_ = get_parameter("costmap_topic").as_string();
  resolution_ = get_parameter("costmap.resolution").as_double();
  width_ = get_parameter("costmap.width").as_int();
  height_ = get_parameter("costmap.height").as_int();
  origin_.position.x = get_parameter("costmap.origin.position.x").as_double();
  origin_.position.y = get_parameter("costmap.origin.position.y").as_double();
  origin_.orientation.w = get_parameter("costmap.origin.orientation.w").as_double();
  inflation_radius_ = get_parameter("costmap.inflation_radius").as_double();
}

void CostmapNode::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) const {
  costmap_.updateCostmap(msg);

  auto costmap_msg = *costmap_.getCostmapData();
  costmap_msg.header = msg->header;
  costmap_pub_->publish(costmap_msg);
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
