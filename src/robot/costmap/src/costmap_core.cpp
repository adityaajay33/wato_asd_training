#include "costmap_core.hpp"

namespace robot {

CostmapCore::CostmapCore(const rclcpp::Logger& logger)
    : logger_(logger), resolution_(0.1), size_x_(100), size_y_(100) {
    costmap_.resize(size_x_ * size_y_, 0);
    RCLCPP_INFO(logger_, "CostmapCore initialized with size %dx%d and resolution %.2f", size_x_, size_y_, resolution_);
}

void CostmapCore::updateFromLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    RCLCPP_DEBUG(logger_, "Starting costmap update from LaserScan...");
    
    if (scan->ranges.empty()) {
        RCLCPP_WARN(logger_, "Received empty LaserScan data! Skipping update.");
        return;
    }

    // Clear costmap before updating
    std::fill(costmap_.begin(), costmap_.end(), 0);

    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        double angle = scan->angle_min + i * scan->angle_increment;
        double range = scan->ranges[i];

        if (std::isnan(range) || std::isinf(range)) {
            RCLCPP_DEBUG(logger_, "Skipping range[%lu]: invalid value (NaN/Inf)", i);
            continue;
        }

        if (range <= scan->range_min || range >= scan->range_max) {
            RCLCPP_DEBUG(logger_, "Skipping range[%lu]: out of bounds (range = %.2f)", i, range);
            continue;
        }

        int x_grid, y_grid;
        convertToGrid(range, angle, x_grid, y_grid);

        if (isValidGridCell(x_grid, y_grid)) {
            markObstacle(x_grid, y_grid);
            RCLCPP_DEBUG(logger_, "Marked obstacle at grid (%d, %d)", x_grid, y_grid);
        } else {
            RCLCPP_DEBUG(logger_, "Converted grid (%d, %d) is out of bounds! Skipping.", x_grid, y_grid);
        }
    }

    RCLCPP_INFO(logger_, "Costmap update from LaserScan completed.");
}

void CostmapCore::convertToGrid(double range, double angle, int &x_grid, int &y_grid) {
    double x = range * std::cos(angle);
    double y = range * std::sin(angle);

    x_grid = static_cast<int>((x + size_x_ * resolution_ / 2) / resolution_);
    y_grid = static_cast<int>((y + size_y_ * resolution_ / 2) / resolution_);

    RCLCPP_DEBUG(logger_, "Converted range %.2f and angle %.2f to grid (%d, %d)", range, angle, x_grid, y_grid);
}

void CostmapCore::markObstacle(int x, int y) {
    if (isValidGridCell(x, y)) {
        costmap_[y * size_x_ + x] = 100;
        RCLCPP_DEBUG(logger_, "Obstacle marked on costmap at grid (%d, %d)", x, y);
    } else {
        RCLCPP_WARN(logger_, "Cannot mark obstacle at grid (%d, %d): out of bounds!", x, y);
    }
}

bool CostmapCore::isValidGridCell(int x, int y) const {
    return x >= 0 && x < size_x_ && y >= 0 && y < size_y_;
}

} // namespace robot
