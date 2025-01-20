#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <vector>
#include <queue>
#include <memory>

namespace robot {

class CostmapCore {
public:
    explicit CostmapCore(const rclcpp::Logger& logger);

    void initialize(double resolution, int width, int height, const geometry_msgs::msg::Pose& origin, double inflation_radius);
    void updateFromLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    nav_msgs::msg::OccupancyGrid::SharedPtr getCostmap() const;

private:
    void markObstacle(int x, int y);
    void inflateObstacles(int origin_x, int origin_y) const;
    void convertToGridCoordinates(double range, double angle, int& grid_x, int& grid_y) const;
    bool isValidCell(int x, int y) const;

    rclcpp::Logger logger_;
    nav_msgs::msg::OccupancyGrid::SharedPtr costmap_;
    double resolution_;
    int size_x_, size_y_;
    double inflation_radius_;
    int inflation_cells_;
};

}  // namespace robot

#endif  // COSTMAP_CORE_HPP_
