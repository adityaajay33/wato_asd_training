#ifndef PLANNER_CORE_HPP
#define PLANNER_CORE_HPP

#include <rclcpp/logging.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <unordered_map>
#include <vector>
#include <cmath>

namespace robot {

/**
 * @brief A struct to represent a cell index in the costmap grid.
 */
struct CellIndex {
    int x;
    int y;

    bool operator==(const CellIndex& other) const {
        return x == other.x && y == other.y;
    }
};

/**
 * @brief Hash function for CellIndex.
 */
struct CellIndexHash {
    std::size_t operator()(const CellIndex& idx) const {
        return std::hash<int>()(idx.x) ^ std::hash<int>()(idx.y);
    }
};

/**
 * @brief Core class for path planning.
 */
class PlannerCore {
public:
    explicit PlannerCore(const rclcpp::Logger& logger);
    void configurePlanner(double smoothing_factor, int max_iterations);
    bool computePath(double start_x, double start_y, double goal_x, double goal_y,
                     nav_msgs::msg::OccupancyGrid::SharedPtr map);
    nav_msgs::msg::Path::SharedPtr retrievePath() const;

private:
    rclcpp::Logger logger_;
    nav_msgs::msg::Path::SharedPtr path_;
    nav_msgs::msg::OccupancyGrid::SharedPtr map_;
    double smoothing_factor_;
    int max_iterations_;

    bool worldToMap(double wx, double wy, CellIndex& idx) const;
    void mapToWorld(const CellIndex& idx, double& wx, double& wy) const;
    std::vector<CellIndex> findNeighbors(const CellIndex& idx) const;
    double calculateHeuristic(const CellIndex& source, const CellIndex& target) const;
    double computeStepCost(const CellIndex& from, const CellIndex& to) const;
    void buildPath(const std::unordered_map<CellIndex, CellIndex, CellIndexHash>& previous_nodes,
                   const CellIndex& current, std::vector<CellIndex>& path_result) const;
    bool performAStar(const CellIndex& start, const CellIndex& goal, std::vector<CellIndex>& path_result);
};

} // namespace robot

#endif // PLANNER_CORE_HPP