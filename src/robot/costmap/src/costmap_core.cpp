#include "costmap_core.hpp"

namespace robot {

CostmapCore::CostmapCore(const rclcpp::Logger& logger)
    : logger_(logger), resolution_(0.1), size_x_(100), size_y_(100) {
    costmap_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    costmap_->info.resolution = resolution_;
    costmap_->info.width = size_x_;
    costmap_->info.height = size_y_;
    costmap_->data.resize(size_x_ * size_y_, 0);
    RCLCPP_INFO(logger_, "CostmapCore initialized with size %dx%d and resolution %.2f", size_x_, size_y_, resolution_);
}

void CostmapCore::initializeCostmap(double resolution, int width, int height, const geometry_msgs::msg::Pose& origin) {
    resolution_ = resolution;
    size_x_ = width;
    size_y_ = height;

    costmap_->info.resolution = resolution;
    costmap_->info.width = width;
    costmap_->info.height = height;
    costmap_->info.origin = origin;

    costmap_->data.resize(width * height, 0); // Initialize with empty cells
    RCLCPP_INFO(logger_, "Costmap initialized with resolution %.2f, width %d, height %d", resolution, width, height);
}

void CostmapCore::updateFromLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    RCLCPP_DEBUG(logger_, "Starting costmap update from LaserScan...");
    if (!scan || scan->ranges.empty()) {
        RCLCPP_WARN(logger_, "Empty or invalid LaserScan received. Skipping update.");
        return;
    }

    // Clear the costmap
    std::fill(costmap_->data.begin(), costmap_->data.end(), 0);

    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        double angle = scan->angle_min + i * scan->angle_increment;
        double range = scan->ranges[i];

        if (std::isnan(range) || std::isinf(range) || range < scan->range_min || range > scan->range_max) {
            RCLCPP_DEBUG(logger_, "Skipping invalid range[%zu]: %.2f", i, range);
            continue;
        }

        int x_grid, y_grid;
        convertToGrid(range, angle, x_grid, y_grid);

        if (isValidGridCell(x_grid, y_grid)) {
            markObstacle(x_grid, y_grid);
        } else {
            RCLCPP_DEBUG(logger_, "Grid cell (%d, %d) is out of bounds. Skipping.", x_grid, y_grid);
        }
    }

    RCLCPP_INFO(logger_, "Costmap update from LaserScan completed.");
}

void CostmapCore::convertToGrid(double range, double angle, int& x, int& y) const {
    double wx = range * std::cos(angle);
    double wy = range * std::sin(angle);

    x = static_cast<int>((wx + size_x_ * resolution_ / 2) / resolution_);
    y = static_cast<int>((wy + size_y_ * resolution_ / 2) / resolution_);

    RCLCPP_DEBUG(logger_, "Converted world coordinates (%.2f, %.2f) to grid (%d, %d)", wx, wy, x, y);
}

void CostmapCore::markObstacle(int x, int y) {
    if (isValidGridCell(x, y)) {
        costmap_->data[y * size_x_ + x] = 100; // Mark cell as occupied
        RCLCPP_DEBUG(logger_, "Marked obstacle at (%d, %d)", x, y);
    }
}

bool CostmapCore::isValidGridCell(int x, int y) const {
    return x >= 0 && x < size_x_ && y >= 0 && y < size_y_;
}

nav_msgs::msg::OccupancyGrid::SharedPtr CostmapCore::getCostmap() const {
    return costmap_;
}

} // namespace robot
