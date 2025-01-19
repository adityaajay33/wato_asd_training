#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <cmath>

namespace robot
{

class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);

    void initializeGlobalMap(int width, int height, double resolution);
    void mergeCostmap(cont nav_msgs::msg::OccupancyGrid &costmap, double robot_x, double robot_y);

    const nav_msgs::msg::OccupancyGrid &getGlobalMap() const;

  private:
    rclcpp::Logger logger_;

    nav_msgs::msg::OccupancyGrid global_map_;
    
    int worldToGridIndex(double world_x, double world_y) const;
    void updateGlobalMapCell(int x, int y, int cost);
};

}  

#endif  
