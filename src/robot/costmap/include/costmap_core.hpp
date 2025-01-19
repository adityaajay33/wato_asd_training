#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <vector>
#include <cmath>

namespace robot
{

class CostmapCore {
  public:
    explicit CostmapCore(const rclcpp::Logger& logger);

    void initializeCostmap(int width, int height, double resolution);

    void convertToGrid(double range, double angle, int &x_grid, int &y_grid);

    void markObstacle(int x_grid, int y_grid);

    void inflateObstacles(double inflation_radius, int max_cost);

    void publishCostmap(const rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr &publisher);

  private:
    rclcpp::Logger logger_;
    std::vector<std::vector<int>> costmap_;
    int width_;
    int height_;
    double resolution_;
    double origin_x_;
    double origin_y_;

    bool isValidGridCoordinate(int x, int y);
};
};