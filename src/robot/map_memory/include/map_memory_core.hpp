#ifndef MAP_MEMORY_HANDLER_HPP_
#define MAP_MEMORY_HANDLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose.hpp>

namespace memory_ns
{

class MapMemoryHandler {
public:
  explicit MapMemoryHandler(const rclcpp::Logger & logger);

  void initializeMemory(
    double resolution,
    int width,
    int height,
    const geometry_msgs::msg::Pose & map_origin
  );

  void mergeLocalCostmap(
    const nav_msgs::msg::OccupancyGrid::SharedPtr & local_costmap,
    double robot_x, double robot_y, double robot_heading
  );

  // Convert world coordinates (rx, ry) into map indices (mx, my)
  bool convertToMapIndices(double rx, double ry, int &mx, int &my);

  // Retrieve the global occupancy grid
  nav_msgs::msg::OccupancyGrid::SharedPtr retrieveMemoryGrid() const;

private:
  nav_msgs::msg::OccupancyGrid::SharedPtr global_memory_;  
  rclcpp::Logger logger_;
};

}  // namespace memory_ns

#endif  // MAP_MEMORY_HANDLER_HPP_
