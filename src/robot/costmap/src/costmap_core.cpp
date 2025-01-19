#include "costmap_core.hpp"

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger), resolution_(0.1), size_x_(100), size_y_(100) {
    costmap.resize(size_x_*size_y_, 0);
    RCLCPP_INFO(logger_, "CostmapCore initialized with size %dx%d and resolution %.2f", size_x_, size_y_, resolution_);

}

void CostmapCore::updateFromLaserScan(const sensor_msgs::msg::LaserScan::updateFromLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr scan)){
    RCLCPP_INFO(logger_, "Update costmap from LaserScan data.")

    for(size_t i=0; i<scan->ranges.size(); ++i)){
        double angle = scan->angle_min + i *scan->angle_increment;
        double range = scan->ranges[i];

        if (range < scan->range_max && range>scan->range_min){
            int x_grid
        }
    }
}

}