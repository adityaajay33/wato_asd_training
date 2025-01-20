#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace robot {

class CostmapCore {
public:
  void initCostmap(
    double resolution, 
    int width, 
    int height, 
    geometry_msgs::msg::Pose origin, 
    double inflation_radius
  );
  
  explicit CostmapCore(const rclcpp::Logger& logger);

  void updateCostmap(const sensor_msgs::msg::LaserScan::SharedPtr laserscan) const;
  void inflateObstacle(int origin_x, int origin_y) const;

  nav_msgs::msg::OccupancyGrid::SharedPtr getCostmapData() const;

private:
  nav_msgs::msg::OccupancyGrid::SharedPtr costmap_data_;
  rclcpp::Logger logger_;

  double inflation_radius_;
  int inflation_cells_;
};

}  // namespace robot

#endif  // COSTMAP_CORE_HPP_
