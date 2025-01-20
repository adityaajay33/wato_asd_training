#ifndef PLANNER_NODE_HPP
#define PLANNER_NODE_HPP

#include <mutex>
#include <memory>
#include <string>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include "planner_core.hpp"

class PlannerNode : public rclcpp::Node {
public:
    PlannerNode();

private:
    void initializeParameters();
    void handleMapUpdate(const nav_msgs::msg::OccupancyGrid::SharedPtr map);
    void handleNewGoal(const geometry_msgs::msg::PointStamped::SharedPtr goal_msg);
    void handleOdometry(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
    void periodicCheck();

    void attemptPathPlanning();
    void clearCurrentGoal();

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::TimerBase::SharedPtr update_timer_;

    robot::PlannerCore planner_;
    std::mutex data_mutex_;

    nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;
    geometry_msgs::msg::PointStamped active_goal_;
    double robot_position_x_, robot_position_y_;
    bool has_active_goal_, has_odometry_;

    std::string map_topic_, goal_topic_, odom_topic_, path_topic_;
    double smoothing_factor_, timeout_limit_, goal_tolerance_;
    int max_iterations_;

    rclcpp::Time goal_start_time_;
};

#endif // PLANNER_NODE_HPP