#include "costmap_core.hpp"
#include <cmath>
#include <algorithm>

namespace robot {

CostmapCore::CostmapCore(const rclcpp::Logger& logger)
    : logger_(logger), costmap_(std::make_shared<nav_msgs::msg::OccupancyGrid>()) {}

void CostmapCore::initialize(double resolution, int width, int height, const geometry_msgs::msg::Pose& origin, double inflation_radius) {
    resolution_ = resolution;
    size_x_ = width;
    size_y_ = height;
    inflation_radius_ = inflation_radius;
    inflation_cells_ = static_cast<int>(inflation_radius / resolution);

    costmap_->info.resolution = resolution;
    costmap_->info.width = width;
    costmap_->info.height = height;
    costmap_->info.origin = origin;
    costmap_->data.assign(width * height, 0);

    RCLCPP_INFO(logger_, "Costmap initialized (Resolution: %.2f, Width: %d, Height: %d, Inflation: %.2f)", resolution, width, height, inflation_radius);
}

void CostmapCore::updateFromLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    if (!scan || scan->ranges.empty()) {
        RCLCPP_WARN(logger_, "Invalid or empty LaserScan. Skipping costmap update.");
        return;
    }

    std::fill(costmap_->data.begin(), costmap_->data.end(), 0);  // Clear costmap

    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        double angle = scan->angle_min + i * scan->angle_increment;
        double range = scan->ranges[i];

        if (std::isnan(range) || std::isinf(range) || range < scan->range_min || range > scan->range_max) {
            continue;  // Skip invalid readings
        }

        int grid_x, grid_y;
        convertToGridCoordinates(range, angle, grid_x, grid_y);

        if (isValidCell(grid_x, grid_y)) {
            markObstacle(grid_x, grid_y);
            inflateObstacles(grid_x, grid_y);
        }
    }

    RCLCPP_INFO(logger_, "Costmap updated with LaserScan data.");
}

void CostmapCore::convertToGridCoordinates(double range, double angle, int& grid_x, int& grid_y) const {
    double world_x = range * std::cos(angle);
    double world_y = range * std::sin(angle);

    grid_x = static_cast<int>((world_x - costmap_->info.origin.position.x) / resolution_);
    grid_y = static_cast<int>((world_y - costmap_->info.origin.position.y) / resolution_);
}

bool CostmapCore::isValidCell(int x, int y) const {
    return x >= 0 && x < size_x_ && y >= 0 && y < size_y_;
}

void CostmapCore::markObstacle(int x, int y) {
    if (isValidCell(x, y)) {
        costmap_->data[y * size_x_ + x] = 100;  // Mark as occupied
    }
}

void CostmapCore::inflateObstacles(int origin_x, int origin_y) const {
    // Use a queue-based BFS to mark cells within the inflation radius
    std::queue<std::pair<int, int>> queue;
    queue.emplace(origin_x, origin_y);

    std::vector<std::vector<bool>> visited(size_x_, std::vector<bool>(size_y_, false));
    visited[origin_x][origin_y] = true;

    while (!queue.empty()) {
        auto [x, y] = queue.front();
        queue.pop();

        // Iterate over neighboring cells
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                if (dx == 0 && dy == 0) continue;  // Skip the center cell

                int nx = x + dx;
                int ny = y + dy;

                // Ensure the neighbor cell is within bounds
                if (nx >= 0 && nx < size_x_ && ny >= 0 && ny < size_y_ && !visited[nx][ny]) {
                    // Calculate the distance to the original obstacle cell
                    double distance = std::hypot(nx - origin_x, ny - origin_y) * resolution_;

                    // If within inflation radius, mark as inflated and add to queue
                    if (distance <= inflation_radius_) {
                        int index = ny * size_x_ + nx;
                        costmap_->data[index] = std::max(
                            static_cast<int>(costmap_->data[index]),  // Cast existing value to int
                            static_cast<int>((1 - (distance / inflation_radius_)) * 100));  // Inflation value
                        queue.emplace(nx, ny);
                    }

                    visited[nx][ny] = true;
                }
            }
        }
    }
}


nav_msgs::msg::OccupancyGrid::SharedPtr CostmapCore::getCostmap() const {
    return costmap_;
}

}  // namespace robot
