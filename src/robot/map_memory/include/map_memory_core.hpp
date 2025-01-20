#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <memory>
#include <vector>

namespace robot {

class MapMemoryCore {
public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);

    void initializeGlobalMap(double resolution, int width, int height, const geometry_msgs::msg::Pose& origin);

    void mergeCostmap(const nav_msgs::msg::OccupancyGrid& costmap, double robot_x, double robot_y, double robot_theta);

    bool robotToMap(double rx, double ry, int& mx, int& my) const;

    nav_msgs::msg::OccupancyGrid::SharedPtr getGlobalMap() const;

private:
    nav_msgs::msg::OccupancyGrid::SharedPtr global_map_;
    rclcpp::Logger logger_;
    double resolution_;
    int width_;
    int height_;
};

} // namespace robot

#endif
