#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <vector>
#include <memory>

namespace robot
{

class CostmapCore {
  public:
    explicit CostmapCore(const rclcpp::Logger& logger);

    void initializeCostmap(double resolution, int width, int height, const geometry_msgs::msg::Pose& origin, double inflation_radius);

    void updateWithLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr& scan);

    nav_msgs::msg::OccupancyGrid::SharedPtr getCostmap() const;

    void inflateRegion(int x, int y);

  private:
    rclcpp::Logger logger_;
    nav_msgs::msg::OccupancyGrid::SharedPtr costmap_;
    double resolution_;
    int width_;
    int height_;
    geometry_msgs::msg::Pose origin_;
    double inflation_radius_;
    int inflation_cells_;

    bool isWithinBounds(int x, int y) const;

    void convertToGrid(double range, double angle, int& x, int& y) const;
    void markCell(int x, int y);
};

} // namespace robot

#endif
