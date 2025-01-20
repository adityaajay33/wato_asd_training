#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "costmap_core.hpp"

class CostmapNode : public rclcpp::Node {
public:
    CostmapNode();

private:
    void loadParameters();
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;

    robot::CostmapCore costmap_;

    std::string laser_topic_;
    std::string costmap_topic_;
    double resolution_;
    int width_, height_;
    geometry_msgs::msg::Pose origin_;
    double inflation_radius_;
};

#endif  // COSTMAP_NODE_HPP_
