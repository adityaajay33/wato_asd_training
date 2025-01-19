#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <cmath>
#include <vector>

namespace robot {

class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);

    void initMapMemory(double resolution, int width, int height, geometry_msgs::msg::Pose origin);
    void updateMap(nav_msgs::msg::OccupancyGrid::SharedPtr local_costmap, double robot_x, double robot_y, double robot_theta);
    bool robotToMap(double rx, double ry, int &mx, int &my);

    nav_msgs::msg::OccupancyGrid::SharedPtr getMapData() const;

  private:
    rclcpp::Logger logger_;
    nav_msgs::msg::OccupancyGrid::SharedPtr global_map_;

    void updateGlobalMapCell(int x, int y, int cost);
    int worldToGridIndex(double world_x, double world_y) const;
};

}  // namespace robot

#endif
