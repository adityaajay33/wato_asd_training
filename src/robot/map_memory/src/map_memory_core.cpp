#include "map_memory_core.hpp"
#include <algorithm>
#include <cmath>

namespace memory_ns
{

MapMemoryHandler::MapMemoryHandler(const rclcpp::Logger & logger)
: global_memory_(std::make_shared<nav_msgs::msg::OccupancyGrid>()),
  logger_(logger)
{
  // Constructor: optionally do some defaults or logging
}

void MapMemoryHandler::initializeMemory(
  double resolution,
  int width,
  int height,
  const geometry_msgs::msg::Pose & map_origin
)
{
  global_memory_->info.resolution = resolution;
  global_memory_->info.width      = width;
  global_memory_->info.height     = height;
  global_memory_->info.origin     = map_origin;

  // Initialize all cells to 0 (free space)
  global_memory_->data.assign(width * height, 0);

  RCLCPP_INFO(logger_, "Global map set up (res=%.2f, W=%d, H=%d)", resolution, width, height);
}

void MapMemoryHandler::mergeLocalCostmap(
  const nav_msgs::msg::OccupancyGrid::SharedPtr & local_costmap,
  double robot_x, double robot_y, double robot_heading
)
{
  double local_res  = local_costmap->info.resolution;
  double origin_lx  = local_costmap->info.origin.position.x;
  double origin_ly  = local_costmap->info.origin.position.y;
  unsigned int loc_w= local_costmap->info.width;
  unsigned int loc_h= local_costmap->info.height;

  const auto & local_vals = local_costmap->data;

  // Transform each local cell to world space, then to global map indices
  for (unsigned int row = 0; row < loc_h; ++row) {
    for (unsigned int col = 0; col < loc_w; ++col) {
      int8_t cell_val = local_vals[row * loc_w + col];

      // Skip unknown
      if (cell_val < 0) {
        continue;
      }
      // Convert (col, row) -> local metric coords
      double lx = origin_lx + (col + 0.5) * local_res;
      double ly = origin_ly + (row + 0.5) * local_res;

      // Robot transform
      double cos_heading = std::cos(robot_heading);
      double sin_heading = std::sin(robot_heading);
      double wx = robot_x + (lx * cos_heading - ly * sin_heading);
      double wy = robot_y + (lx * sin_heading + ly * cos_heading);

      // Convert world coords -> global map
      int gx, gy;
      if (!convertToMapIndices(wx, wy, gx, gy)) {
        // out of bounds
        continue;
      }

      // Merge by taking maximum cost
      int8_t &global_cell = global_memory_->data[gy * global_memory_->info.width + gx];
      int existing_val = (global_cell < 0) ? 0 : global_cell;  // treat -1 as 0
      int new_val = static_cast<int>(cell_val);
      int combined = std::max(existing_val, new_val);
      global_cell = static_cast<int8_t>(combined);
    }
  }
}

bool MapMemoryHandler::convertToMapIndices(double rx, double ry, int &mx, int &my)
{
  double gx_origin = global_memory_->info.origin.position.x;
  double gy_origin = global_memory_->info.origin.position.y;
  double grid_res  = global_memory_->info.resolution;
  int grid_w       = static_cast<int>(global_memory_->info.width);
  int grid_h       = static_cast<int>(global_memory_->info.height);

  if (rx < gx_origin || ry < gy_origin) {
    return false;
  }

  my = static_cast<int>((ry - gy_origin) / grid_res);
  mx = static_cast<int>((rx - gx_origin) / grid_res);

  if (mx < 0 || mx >= grid_w || my < 0 || my >= grid_h) {
    return false;
  }
  return true;
}

nav_msgs::msg::OccupancyGrid::SharedPtr MapMemoryHandler::retrieveMemoryGrid() const
{
  return global_memory_;
}

}  // namespace memory_ns
