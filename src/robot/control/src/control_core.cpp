#include "control_core.hpp"
#include <cmath>
#include <algorithm>

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger & logger)
  : path_(), logger_(logger)
{}

void ControlCore::initControlCore(
  double lookahead_distance,
  double max_steering_angle,
  double steering_gain,
  double linear_velocity
) {
  lookahead_distance_ = lookahead_distance;
  max_steering_angle_ = max_steering_angle;
  steering_gain_ = steering_gain;
  linear_velocity_ = linear_velocity;
}

void ControlCore::updatePath(nav_msgs::msg::Path path) {
  RCLCPP_INFO(logger_, "Path has been updated.");
  path_ = path;
}

bool ControlCore::isPathEmpty() {
  return path_.poses.empty();
}

geometry_msgs::msg::Twist ControlCore::calculateControlCommand(double robot_x, double robot_y, double robot_theta) {
  geometry_msgs::msg::Twist twist;

  unsigned int lookahead_index = findLookaheadPoint(robot_x, robot_y, robot_theta);
  if (lookahead_index >= path_.poses.size())
    return twist;  // Return zero command if index is invalid

  // Retrieve the lookahead point
  double target_x = path_.poses[lookahead_index].pose.position.x;
  double target_y = path_.poses[lookahead_index].pose.position.y;
  double dx = target_x - robot_x;
  double dy = target_y - robot_y;

  // Compute the target angle and error relative to robot's current heading
  double target_angle = std::atan2(dy, dx);
  double error_angle = target_angle - robot_theta;

  // Normalize error angle to [-pi, pi]
  if (error_angle > M_PI)
    error_angle -= 2 * M_PI;
  else if (error_angle < -M_PI)
    error_angle += 2 * M_PI;

  // If the steering error is too large, stop forward motion
  if (std::abs(error_angle) > std::abs(max_steering_angle_))
    twist.linear.x = 0;
  else
    twist.linear.x = linear_velocity_;

  // Constrain steering error within limits
  error_angle = std::max(-max_steering_angle_, std::min(error_angle, max_steering_angle_));
  twist.angular.z = error_angle * steering_gain_;

  return twist;
}

unsigned int ControlCore::findLookaheadPoint(double robot_x, double robot_y, double robot_theta) {
  double best_distance = std::numeric_limits<double>::max();
  unsigned int index = 0;
  bool found_forward = false;

  // Iterate over all path points to choose a forward, lookahead point
  for (size_t i = 0; i < path_.poses.size(); ++i) {
    double dx = path_.poses[i].pose.position.x - robot_x;
    double dy = path_.poses[i].pose.position.y - robot_y;
    double distance = std::sqrt(dx * dx + dy * dy);
    
    // Skip points that are too near
    if (distance < lookahead_distance_)
      continue;

    double angle_to_point = std::atan2(dy, dx);
    double angle_diff = angle_to_point - robot_theta;
    if (angle_diff > M_PI) angle_diff -= 2 * M_PI;
    if (angle_diff < -M_PI) angle_diff += 2 * M_PI;

    // Check if point is forward-facing (within ±90°)
    if (std::abs(angle_diff) < M_PI / 2) {
      if (distance < best_distance) {
        best_distance = distance;
        index = i;
        found_forward = true;
      }
    }
  }

  // If no forward point is found, pick the closest valid point
  if (!found_forward) {
    for (size_t i = 0; i < path_.poses.size(); ++i) {
      double dx = path_.poses[i].pose.position.x - robot_x;
      double dy = path_.poses[i].pose.position.y - robot_y;
      double distance = std::sqrt(dx * dx + dy * dy);
      
      if (distance < lookahead_distance_)
        continue;

      if (distance < best_distance) {
        best_distance = distance;
        index = i;
      }
    }
  }
  return index;
}

}  // namespace robot
