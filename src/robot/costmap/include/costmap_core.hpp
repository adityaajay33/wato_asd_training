#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <vector>
#include <memory>

namespace robot {

class CostmapCore {
public:
    explicit CostmapCore(const rclcpp::Logger& logger);

    void initializeCostmap(double resolution, int width, int height, const geometry_msgs::msg::Pose& origin);

    void updateFromLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr scan);

    nav_msgs::msg::OccupancyGrid::SharedPtr getCostmap() const;

private:
    rclcpp::Logger logger_;
    nav_msgs::msg::OccupancyGrid::SharedPtr costmap_;
    double resolution_;
    int size_x_;
    int size_y_;

    void convertToGrid(double range, double angle, int& x, int& y) const;
    void markObstacle(int x, int y);
    bool isValidGridCell(int x, int y) const;
};

} // namespace robot

#endif
