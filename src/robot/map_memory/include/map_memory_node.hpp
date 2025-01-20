#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
public:
    MapMemoryNode();

private:
    // Callbacks
    void localCostmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timerCallback();

    // Helper Functions
    void processParameters();
    double quaternionToYaw(double x, double y, double z, double w);

    // ROS Components
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr global_costmap_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Core Map Memory
    robot::MapMemoryCore map_memory_;

    // Parameters
    std::string local_costmap_topic_;
    std::string odom_topic_;
    std::string map_topic_;

    double resolution_;
    int width_;
    int height_;
    geometry_msgs::msg::Pose origin_;
    double update_distance_;
    int map_pub_rate_;

    // Robot State
    double robot_x_;
    double robot_y_;
    double robot_theta_;
    double last_robot_x_;
    double last_robot_y_;
};

#endif
