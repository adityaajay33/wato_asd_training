#include "map_memory_core.hpp"

namespace robot {

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger) 
  : logger_(logger) {

    global_map_.info.resolution = 0.1; // 
    global_map_.info.width = 100;
    global_map_.info.height = 100;
    global_map_.data.resize(100 * 100, -1);
}

void MapMemoryCore::initializeGlobalMap(int width, int height, double resolution) {
    global_map_.info.width = width;
    global_map_.info.height = height;
    global_map_.info.resolution = resolution;
    global_map_.data.resize(width * height, -1); 
}

void MapMemoryCore::mergeCostmap(const nav_msgs::msg::OccupancyGrid &costmap, double robot_x, double robot_y) {
    // Iterate over the costmap and merge it into the global map
    for (int y = 0; y < costmap.info.height; ++y) {
        for (int x = 0; x < costmap.info.width; ++x) {
            int costmap_index = y * costmap.info.width + x;
            int cost = costmap.data[costmap_index];
            if (cost != -1) { // Valid data
                // Transform costmap cell to global map coordinates
                double world_x = robot_x + (x - costmap.info.width / 2) * costmap.info.resolution;
                double world_y = robot_y + (y - costmap.info.height / 2) * costmap.info.resolution;

                // Update global map
                int global_index = worldToGridIndex(world_x, world_y);
                if (global_index >= 0 && global_index < global_map_.data.size()) {
                    global_map_.data[global_index] = cost; // Overwrite with new data
                }
            }
        }
    }
}

int MapMemoryCore::worldToGridIndex(double world_x, double world_y) const {
    int x_index = static_cast<int>(world_x / global_map_.info.resolution);
    int y_index = static_cast<int>(world_y / global_map_.info.resolution);
    return y_index * global_map_.info.width + x_index;
}

const nav_msgs::msg::OccupancyGrid &MapMemoryCore::getGlobalMap() const {
    return global_map_;
}

} // namespace robot
