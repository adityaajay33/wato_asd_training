#include <chrono>
#include <memory>
 
#include "costmap_node.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Initialize the constructs and their parameters
  laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));

  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500), std::bind(&&CostmapNode::publishCostmap, this);
  )
}
 
// Define the timer to publish a message every 500ms
void CostmapNode::publishCostmap() {
  auto msg = nav_msgs::msg::OccupancyGrid();
  msg.header.stamp = this->now();
  msg.header.frame_id = "map";

  costmap_.fillOccupancyGrid(msg);
  costmap_pub_->publish(msg);
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg){ 

  costmap_.updateFromLaserScan(msg);
}
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}