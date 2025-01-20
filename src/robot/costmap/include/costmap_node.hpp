#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "costmap_core.hpp"

class CostmapNode : public rclcpp::Node {
public:
    CostmapNode();

private:
    robot::CostmapCore costmap_;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscriber_;

    std::string laser_input_topic_;
    std::string grid_output_topic_;
    double grid_resolution_;
    int grid_width_;
    int grid_height_;
    geometry_msgs::msg::Pose grid_origin_;
    double obstacle_expansion_radius_;

    void initializeSettings();
    void processLaserData(const sensor_msgs::msg::LaserScan::SharedPtr laser_scan);

    template <typename T>
    T retrieveParameter(const std::string &key, const T &default_value);
};

#endif
