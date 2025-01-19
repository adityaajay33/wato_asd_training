#include "planner_node.hpp"

PlannerNode::PlannerNode() : Node("planner_node"), state_(State::WAITING_FOR_GOAL) {
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    current_map_ = *msg;
    if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
        planPath();
    }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    goal_ = *msg;
    goal_received_ = true;
    state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
    planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    robot_pose_ = msg->pose.pose;
}

void PlannerNode::timerCallback() {
    if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
        if (goalReached()) {
            RCLCPP_INFO(this->get_logger(), "Goal reached!");
            state_ = State::WAITING_FOR_GOAL;
        } else {
            RCLCPP_INFO(this->get_logger(), "Replanning due to timeout or progress...");
            planPath();
        }
    }
}

bool PlannerNode::goalReached() {
    double dx = goal_.point.x - robot_pose_.position.x;
    double dy = goal_.point.y - robot_pose_.position.y;
    return std::sqrt(dx * dx + dy * dy) < 0.5;
}

void PlannerNode::planPath() {
    if (!goal_received_ || current_map_.data.empty()) {
        RCLCPP_WARN(this->get_logger(), "Cannot plan path: Missing map or goal!");
        return;
    }

    nav_msgs::msg::Path path;
    path.header.stamp = this->get_clock()->now();
    path.header.frame_id = "map";

    // A* implementation
    std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_list;
    std::unordered_map<CellIndex, double, CellIndexHash> g_score;
    std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;

    CellIndex start = worldToGrid(robot_pose_.position.x, robot_pose_.position.y);
    CellIndex goal = worldToGrid(goal_.point.x, goal_.point.y);

    open_list.emplace(start, 0.0);
    g_score[start] = 0.0;

    while (!open_list.empty()) {
        AStarNode current = open_list.top();
        open_list.pop();

        if (current.index == goal) {
            reconstructPath(path, came_from, current.index);
            path_pub_->publish(path);
            return;
        }

        for (const auto& neighbor : getNeighbors(current.index)) {
            double tentative_g_score = g_score[current.index] + distance(current.index, neighbor);

            if (g_score.find(neighbor) == g_score.end() || tentative_g_score < g_score[neighbor]) {
                g_score[neighbor] = tentative_g_score;
                double f_score = tentative_g_score + heuristic(neighbor, goal);
                open_list.emplace(neighbor, f_score);
                came_from[neighbor] = current.index;
            }
        }
    }

    RCLCPP_WARN(this->get_logger(), "Failed to find a path!");
}

std::vector<CellIndex> PlannerNode::getNeighbors(const CellIndex& index) {
    std::vector<CellIndex> neighbors;
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            if (dx == 0 && dy == 0) continue;
            CellIndex neighbor(index.x + dx, index.y + dy);
            if (isValidCell(neighbor)) {
                neighbors.push_back(neighbor);
            }
        }
    }
    return neighbors;
}

bool PlannerNode::isValidCell(const CellIndex& index) {
    int idx = index.y * current_map_.info.width + index.x;
    return idx >= 0 && idx < static_cast<int>(current_map_.data.size()) &&
           current_map_.data[idx] == 0;
}

double PlannerNode::heuristic(const CellIndex& a, const CellIndex& b) {
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

double PlannerNode::distance(const CellIndex& a, const CellIndex& b) {
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

CellIndex PlannerNode::worldToGrid(double x, double y) {
    int grid_x = static_cast<int>((x - current_map_.info.origin.position.x) / current_map_.info.resolution);
    int grid_y = static_cast<int>((y - current_map_.info.origin.position.y) / current_map_.info.resolution);
    return CellIndex(grid_x, grid_y);
}

void PlannerNode::reconstructPath(nav_msgs::msg::Path& path, const std::unordered_map<CellIndex, CellIndex, CellIndexHash>& came_from, const CellIndex& current) {
    CellIndex temp = current;
    while (came_from.find(temp) != came_from.end()) {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = temp.x * current_map_.info.resolution + current_map_.info.origin.position.x;
        pose.pose.position.y = temp.y * current_map_.info.resolution + current_map_.info.origin.position.y;
        path.poses.push_back(pose);
        temp = came_from.at(temp);
    }
    std::reverse(path.poses.begin(), path.poses.end());
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlannerNode>());
    rclcpp::shutdown();
    return 0;
}
