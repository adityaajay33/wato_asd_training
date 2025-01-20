#include "planner_core.hpp"
#include <cmath>
#include <queue>
#include <limits>
#include <algorithm>

namespace robot {

PlannerCore::PlannerCore(const rclcpp::Logger& logger)
    : logger_(logger),
      path_(std::make_shared<nav_msgs::msg::Path>()),
      map_(std::make_shared<nav_msgs::msg::OccupancyGrid>()),
      smoothing_factor_(0.1),
      max_iterations_(10) {}

void PlannerCore::configurePlanner(double smoothing_factor, int max_iterations) {
    smoothing_factor_ = smoothing_factor;
    max_iterations_ = max_iterations;
}

bool PlannerCore::computePath(double start_x, double start_y, double goal_x, double goal_y, 
                              nav_msgs::msg::OccupancyGrid::SharedPtr map) {
    map_ = map;

    CellIndex start_idx, goal_idx;
    if (!worldToMap(start_x, start_y, start_idx) || !worldToMap(goal_x, goal_y, goal_idx)) {
        RCLCPP_WARN(logger_, "Start or Goal position is out of bounds.");
        return false;
    }

    RCLCPP_INFO(logger_, "Planning path from (%.2f, %.2f) to (%.2f, %.2f)", start_x, start_y, goal_x, goal_y);

    std::vector<CellIndex> path_cells;
    if (!performAStar(start_idx, goal_idx, path_cells)) {
        RCLCPP_WARN(logger_, "A* algorithm failed to find a valid path.");
        return false;
    }

    path_->poses.clear();
    for (const auto& cell : path_cells) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = map_->header;

        double wx, wy;
        mapToWorld(cell, wx, wy);

        pose.pose.position.x = wx;
        pose.pose.position.y = wy;
        pose.pose.orientation.w = 1.0;

        path_->poses.push_back(pose);
    }

    return true;
}

bool PlannerCore::performAStar(const CellIndex& start, const CellIndex& goal, std::vector<CellIndex>& path_result) {
    const int width = map_->info.width;
    const int height = map_->info.height;

    std::unordered_map<CellIndex, double, CellIndexHash> g_cost;
    std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
    std::unordered_map<CellIndex, double, CellIndexHash> f_cost;

    auto getScore = [&](const auto& container, const CellIndex& idx) {
        auto it = container.find(idx);
        return it != container.end() ? it->second : std::numeric_limits<double>::infinity();
    };

    auto cellCost = [&](const CellIndex& idx) {
        if (idx.x < 0 || idx.x >= width || idx.y < 0 || idx.y >= height) {
            return 100;  // High cost for out-of-bounds
        }
        int index = idx.y * width + idx.x;
        return std::max(0, static_cast<int>(map_->data[index]));
    };

    g_cost[start] = 0.0;
    f_cost[start] = calculateHeuristic(start, goal);

    auto compareF = [&](const CellIndex& a, const CellIndex& b) {
        return f_cost[a] > f_cost[b];
    };

    std::priority_queue<CellIndex, std::vector<CellIndex>, decltype(compareF)> open_set(compareF);
    open_set.push(start);

    while (!open_set.empty()) {
        CellIndex current = open_set.top();
        open_set.pop();

        if (current == goal) {
            buildPath(came_from, current, path_result);
            return true;
        }

        double current_g = g_cost[current];
        for (const auto& neighbor : findNeighbors(current)) {
            if (cellCost(neighbor) > 90) {
                continue;  // Treat high-cost cells as obstacles
            }

            double tentative_g = current_g + computeStepCost(current, neighbor);
            if (tentative_g < getScore(g_cost, neighbor)) {
                g_cost[neighbor] = tentative_g;
                f_cost[neighbor] = tentative_g + calculateHeuristic(neighbor, goal);
                came_from[neighbor] = current;
                open_set.push(neighbor);
            }
        }
    }

    return false;  // Path not found
}

void PlannerCore::buildPath(const std::unordered_map<CellIndex, CellIndex, CellIndexHash>& previous_nodes, 
                            const CellIndex& current, std::vector<CellIndex>& path_result) const {
    CellIndex c = current;
    path_result.clear();
    path_result.push_back(c);

    while (previous_nodes.find(c) != previous_nodes.end()) {
        c = previous_nodes.at(c);
        path_result.push_back(c);
    }
    std::reverse(path_result.begin(), path_result.end());
}

std::vector<CellIndex> PlannerCore::findNeighbors(const CellIndex& current) const {
    std::vector<CellIndex> neighbors;
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            if (dx == 0 && dy == 0) {
                continue;
            }
            neighbors.emplace_back(CellIndex{current.x + dx, current.y + dy});
        }
    }
    return neighbors;
}

double PlannerCore::calculateHeuristic(const CellIndex& source, const CellIndex& target) const {
    return std::hypot(static_cast<double>(source.x - target.x), static_cast<double>(source.y - target.y));
}

double PlannerCore::computeStepCost(const CellIndex& from, const CellIndex& to) const {
    return (from.x != to.x && from.y != to.y) ? std::sqrt(2.0) : 1.0;
}

bool PlannerCore::worldToMap(double wx, double wy, CellIndex& idx) const {
    double origin_x = map_->info.origin.position.x;
    double origin_y = map_->info.origin.position.y;
    double resolution = map_->info.resolution;

    int mx = static_cast<int>((wx - origin_x) / resolution);
    int my = static_cast<int>((wy - origin_y) / resolution);

    if (mx < 0 || mx >= static_cast<int>(map_->info.width) || my < 0 || my >= static_cast<int>(map_->info.height)) {
        return false;
    }

    idx = {mx, my};
    return true;
}

void PlannerCore::mapToWorld(const CellIndex& idx, double& wx, double& wy) const {
    double origin_x = map_->info.origin.position.x;
    double origin_y = map_->info.origin.position.y;
    double resolution = map_->info.resolution;

    wx = origin_x + (idx.x + 0.5) * resolution;
    wy = origin_y + (idx.y + 0.5) * resolution;
}

nav_msgs::msg::Path::SharedPtr PlannerCore::retrievePath() const {
    return path_;
}

}  // namespace robot