#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include <vector>
#include <unordered_map>
#include <queue>
#include <cmath>

struct CellIndex {
    int x, y;
    CellIndex(int xx, int yy) : x(xx), y(yy) {}
    CellIndex() : x(0), y(0) {}
    bool operator==(const CellIndex &other) const {
        return x == other.x && y == other.y;
    }
};

struct CellIndexHash {
    std::size_t operator()(const CellIndex &idx) const {
        return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
    }
};

struct AStarNode {
    CellIndex index;
    double f_score;
    AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

struct CompareF {
    bool operator()(const AStarNode &a, const AStarNode &b) {
        return a.f_score > b.f_score;
    }
};

class PlannerCore {
public:
    PlannerCore();
    nav_msgs::msg::Path computePath(const nav_msgs::msg::OccupancyGrid &map,
                                    const geometry_msgs::msg::Pose &start,
                                    const geometry_msgs::msg::PointStamped &goal);

private:
    std::vector<CellIndex> getNeighbors(const CellIndex &index, const nav_msgs::msg::OccupancyGrid &map);
    bool isValidCell(const CellIndex &index, const nav_msgs::msg::OccupancyGrid &map);
    double heuristic(const CellIndex &a, const CellIndex &b);
    double distance(const CellIndex &a, const CellIndex &b);
    CellIndex worldToGrid(double x, double y, const nav_msgs::msg::OccupancyGrid &map);
};

#endif
