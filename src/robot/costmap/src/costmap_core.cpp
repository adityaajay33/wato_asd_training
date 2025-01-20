#include <algorithm>
#include <queue>
#include "costmap_core.hpp"

namespace robot {

CostmapCore::CostmapCore(const rclcpp::Logger& logger)
: costmap_data_(std::make_shared<nav_msgs::msg::OccupancyGrid>()),
  logger_(logger) 
{}

void CostmapCore::initCostmap(double resolution, int width, int height, 
  geometry_msgs::msg::Pose origin, double inflation_radius) 
{
  costmap_data_->info.resolution = resolution;
  costmap_data_->info.width = width;
  costmap_data_->info.height = height;
  costmap_data_->info.origin = origin;
  costmap_data_->data.assign(width * height, -1);

  inflation_radius_ = inflation_radius;
  inflation_cells_ = static_cast<int>(inflation_radius_ / resolution);

  RCLCPP_INFO(logger_, "Costmap: res=%.2f, w=%d, h=%d", resolution, width, height);
}

void CostmapCore::updateCostmap(const sensor_msgs::msg::LaserScan::SharedPtr laserscan) const {
  std::fill(costmap_data_->data.begin(), costmap_data_->data.end(), 0);

  double angle = laserscan->angle_min;
  for (size_t i = 0; i < laserscan->ranges.size(); ++i, angle += laserscan->angle_increment) {
    double range = laserscan->ranges[i];
    if (range >= laserscan->range_min && range <= laserscan->range_max) {
      double x = range * std::cos(angle);
      double y = range * std::sin(angle);

      int gx = static_cast<int>((x - costmap_data_->info.origin.position.x) / costmap_data_->info.resolution);
      int gy = static_cast<int>((y - costmap_data_->info.origin.position.y) / costmap_data_->info.resolution);

      if (gx >= 0 && gx < static_cast<int>(costmap_data_->info.width) &&
          gy >= 0 && gy < static_cast<int>(costmap_data_->info.height)) {
        costmap_data_->data[gy * costmap_data_->info.width + gx] = 100;
        inflateObstacle(gx, gy);
      }
    }
  }
}

void CostmapCore::inflateObstacle(int origin_x, int origin_y) const {
  std::queue<std::pair<int, int>> q;
  q.emplace(origin_x, origin_y);

  std::vector<std::vector<bool>> visited(
    costmap_data_->info.width, 
    std::vector<bool>(costmap_data_->info.height, false)
  );
  visited[origin_x][origin_y] = true;

  while (!q.empty()) {
    auto [x, y] = q.front();
    q.pop();

    for (int dx = -1; dx <= 1; dx++) {
      for (int dy = -1; dy <= 1; dy++) {
        if (!dx && !dy) continue;
        int nx = x + dx, ny = y + dy;
        if (nx >= 0 && nx < static_cast<int>(costmap_data_->info.width) &&
            ny >= 0 && ny < static_cast<int>(costmap_data_->info.height) &&
            !visited[nx][ny]) 
        {
          double dist = std::hypot(nx - origin_x, ny - origin_y) * costmap_data_->info.resolution;
          if (dist <= inflation_radius_) {
            int idx = ny * costmap_data_->info.width + nx;
            int val = static_cast<int>((1 - dist / inflation_radius_) * 100);
            if (costmap_data_->data[idx] < val) {
              costmap_data_->data[idx] = val;
            }
            q.emplace(nx, ny);
          }
          visited[nx][ny] = true;
        }
      }
    }
  }
}

nav_msgs::msg::OccupancyGrid::SharedPtr CostmapCore::getCostmapData() const {
  return costmap_data_;
}

}  // namespace robot
