#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace robot
{

class ControlCore {
public:
  explicit ControlCore(const rclcpp::Logger & logger);

  // Initialize control parameters
  void initControlCore(
    double lookahead_distance,
    double max_steering_angle,
    double steering_gain,
    double linear_velocity
  );

  // Update the stored path
  void updatePath(nav_msgs::msg::Path path);

  // Check if a valid path is available
  bool isPathEmpty();

  // Determine the lookahead point index based on the robot's position and heading
  unsigned int findLookaheadPoint(double robot_x, double robot_y, double robot_theta);

  // Compute the control command (Twist) for the robot
  geometry_msgs::msg::Twist calculateControlCommand(double robot_x, double robot_y, double robot_theta);

private:
  nav_msgs::msg::Path path_;
  rclcpp::Logger logger_;

  double lookahead_distance_;
  double max_steering_angle_;
  double steering_gain_;
  double linear_velocity_;
};

}  // namespace robot

#endif  // CONTROL_CORE_HPP_
