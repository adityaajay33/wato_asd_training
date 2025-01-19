#include "planner_core.hpp"

PlannerCore::PlannerCore() {}

nav_msgs::msg::Path PlannerCore::computePath(const nav_msgs::msg::OccupancyGrid &map,
                                             const geometry_msgs::msg::Pose &start,
                                             const geometry_msgs::msg::PointStamped &goal) {
    nav_msgs::msg::Path path;
    path.header.stamp = rclcpp::Clock().now();
    path.header.frame_id = "map";

    std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_list;
    std::unordered_map<CellIndex, double, CellIndexHash> g_score;
    std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;

    CellIndex start_idx = worldToGrid(start.position.x, start.position.y, map);
    CellIndex goal_idx = worldToGrid(goal.point.x, goal.point.y, map);

    open_list.emplace(start_idx, 0.0);
    g_score[start_idx] = 0.0;

    while (!open_list.empty()) {
        AStarNode current = open_list.top();
        open_list.pop();

        if (current.index == goal_idx) {
            CellIndex temp = current.index;
            while (came_from.find(temp) != came_from.end()) {
                geometry_msgs::msg::PoseStamped pose;
                pose.pose.position.x = temp.x * map.info.resolution + map.info.origin.position.x;
                pose.pose.position.y = temp.y * map.info.resolution + map.info.origin.position.y;
                path.poses.push_back(pose);
                temp = came_from[temp];
            }
            std::reverse(path.poses.begin(), path.poses.end());
            return path;
        }

        for (const auto &neighbor : getNeighbors(current.index, map)) {
            double tentative_g_score = g_score[current.index] + distance(current.index, neighbor);

            if (g_score.find(neighbor) == g_score.end() || tentative_g_score < g_score[neighbor]) {
                g_score[neighbor] = tentative_g_score;
                double f_score = tentative_g_score + heuristic(neighbor, goal_idx);
                open_list.emplace(neighbor, f_score);
                came_from[neighbor] = current.index;
            }
        }
    }

    return path;
}

std::vector<CellIndex> PlannerCore::getNeighbors(const CellIndex &index, const nav_msgs::msg::OccupancyGrid &map) {
    std::vector<CellIndex> neighbors;
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            if (dx == 0 && dy == 0) continue;
            CellIndex neighbor(index.x + dx, index.y + dy);
            if (isValidCell(neighbor, map)) {
                neighbors.push_back(neighbor);
            }
        }
    }
    return neighbors;
}

bool PlannerCore::isValidCell(const CellIndex &index, const nav_msgs::msg::OccupancyGrid &map) {
    int idx = index.y * map.info.width + index.x;
    return idx >= 0 && idx < static_cast<int>(map.data.size()) && map.data[idx] == 0;
}

double PlannerCore::heuristic(const CellIndex &a, const CellIndex &b) {
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

double PlannerCore::distance(const CellIndex &a, const CellIndex &b) {
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

CellIndex PlannerCore::worldToGrid(double x, double y, const nav_msgs::msg::OccupancyGrid &map) {
    int grid_x = static_cast<int>((x - map.info.origin.position.x) / map.info.resolution);
    int grid_y = static_cast<int>((y - map.info.origin.position.y) / map.info.resolution);
    return CellIndex(grid_x, grid_y);
}
