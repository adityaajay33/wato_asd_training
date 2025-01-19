#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode()
    : Node("map_memory_node"), last_x_(0.0), last_y_(0.0), map_memory_(robot::MapMemoryCore(this->get_logger())) {
    costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

    timer_ = this->create_wall_timer(
        std::chrono::seconds(1), std::bind(&MapMemoryNode::updateMap, this));
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    latest_costmap_ = *msg;
    costmap_received_ = true; 
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;

    double distance = std::sqrt(std::pow(x - last_x_, 2) + std::pow(y - last_y_, 2));
    if (distance >= update_distance_) { 
        last_x_ = x; 
        last_y_ = y;
        should_update_map_ = true; 
    }
}

// Timer-based map update function
void MapMemoryNode::updateMap() {
    if (should_update_map_ && costmap_received_) { 
        map_memory_.mergeCostmap(latest_costmap_, last_x_, last_y_); 
        map_pub_->publish(map_memory_.getGlobalMap());
        should_update_map_ = false; 
        costmap_received_ = false; 
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapMemoryNode>());
    rclcpp::shutdown();
    return 0;
}
