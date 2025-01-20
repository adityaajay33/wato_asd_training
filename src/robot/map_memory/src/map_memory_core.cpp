#include "map_memory_core.hpp"

namespace robot {

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger)
    : global_map_(std::make_shared<nav_msgs::msg::OccupancyGrid>()), logger_(logger) {}

void MapMemoryCore::initializeGlobalMap(double resolution, int width, int height, const geometry_msgs::msg::Pose& origin) {
    resolution_ = resolution;
    width_ = width;
    height_ = height;

    global_map_->info.resolution = resolution;
    global_map_->info.width = width;
    global_map_->info.height = height;
    global_map_->info.origin = origin;

    global_map_->data.resize(width * height, -1); // Initialize cells as unknown (-1)
}

void MapMemoryCore::mergeCostmap(const nav_msgs::msg::OccupancyGrid& costmap, double robot_x, double robot_y, double robot_theta) {
    if (!global_map_) {
        RCLCPP_ERROR(logger_, "Global map is not initialized.");
        return;
    }

    for (unsigned int y = 0; y < costmap.info.height; ++y) {
        for (unsigned int x = 0; x < costmap.info.width; ++x) {
            int costmap_idx = y * costmap.info.width + x;

            // Compute global map indices based on robot position
            int mx = static_cast<int>(robot_x + x);
            int my = static_cast<int>(robot_y + y);

            if (mx >= 0 && mx < width_ && my >= 0 && my < height_) {
                int global_idx = my * width_ + mx;
                global_map_->data[global_idx] = costmap.data[costmap_idx];
            }
        }
    }
}

bool MapMemoryCore::robotToMap(double rx, double ry, int& mx, int& my) const {
    if (!global_map_) return false;

    mx = static_cast<int>((rx - global_map_->info.origin.position.x) / resolution_);
    my = static_cast<int>((ry - global_map_->info.origin.position.y) / resolution_);

    return mx >= 0 && mx < width_ && my >= 0 && my < height_;
}

nav_msgs::msg::OccupancyGrid::SharedPtr MapMemoryCore::getGlobalMap() const {
    return global_map_;
}

} // namespace robot
