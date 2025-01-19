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

        if (range < scan->range_max && range> scan->range_min){
            int x_grid, y_grid;
            convertToGrid(range, angle, x_grid, y_grid);
            markObstacle(x_grid, y_grid);

        }
    }
}

void CostmapCore::convertToGrid(double range, double angle, int &x_grid, int &y_grid) {
    double x = range * std::cos(angle);
    double y = range * std::sin(angle);

    x_grid = static_cast<int>((x + size_x_ * resolution_/2) / resolution_);
    y_grid = static_cast<int>((y + size_y_ *resolution_ /2) / resolution_);
}

voice CostmapCore::markObstacle(int x, int y){

    if (x>=0 && x<size_x_ && y>=0 && y<size_y_){
        costmap_[y*size_x_ + x] = 100;
    }
}