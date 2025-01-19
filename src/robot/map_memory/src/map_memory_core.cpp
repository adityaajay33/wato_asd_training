#include "map_memory_core.hpp"

namespace robot {

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger)
    : logger_(logger), global_map_(std::make_shared<nav_msgs::msg::OccupancyGrid>()) {}

void MapMemoryCore::initializeGlobalMap(double resolution, int width, int height, const geometry_msgs::msg::Pose& origin) {
    global_map_->info.resolution = resolution;
    global_map_->info.width = width;
    global_map_->info.height = height;
    global_map_->info.origin = origin;
    global_map_->data.assign(width * height, -1);
}

void MapMemoryCore::mergeCostmap(const nav_msgs::msg::OccupancyGrid& costmap, double robot_x, double robot_y, double robot_theta) {
    double local_res = costmap.info.resolution;
    double local_origin_x = costmap.info.origin.position.x;
    double local_origin_y = costmap.info.origin.position.y;

    for (int y = 0; y < costmap.info.height; ++y) {
        for (int x = 0; x < costmap.info.width; ++x) {
            int costmap_index = y * costmap.info.width + x;
            int8_t cost = costmap.data[costmap_index];
            if (cost == -1) continue;

            double lx = local_origin_x + (x + 0.5) * local_res;
            double ly = local_origin_y + (y + 0.5) * local_res;

            double cos_t = std::cos(robot_theta);
            double sin_t = std::sin(robot_theta);
            double wx = robot_x + lx * cos_t - ly * sin_t;
            double wy = robot_y + lx * sin_t + ly * cos_t;

            int gx, gy;
            if (!robotToMap(wx, wy, gx, gy)) continue;

            int global_index = gy * global_map_->info.width + gx;
            int current_global_cost = (global_map_->data[global_index] < 0) ? 0 : global_map_->data[global_index];
            global_map_->data[global_index] = std::max(current_global_cost, static_cast<int>(cost));
        }
    }
}

bool MapMemoryCore::robotToMap(double rx, double ry, int& mx, int& my) const {
    double origin_x = global_map_->info.origin.position.x;
    double origin_y = global_map_->info.origin.position.y;
    double resolution = global_map_->info.resolution;

    if (rx < origin_x || ry < origin_y) return false;

    mx = static_cast<int>((rx - origin_x) / resolution);
    my = static_cast<int>((ry - origin_y) / resolution);

    return mx >= 0 && mx < global_map_->info.width && my >= 0 && my < global_map_->info.height;
}

nav_msgs::msg::OccupancyGrid::SharedPtr MapMemoryCore::getGlobalMap() const {
    return global_map_;
}

} // namespace robot
