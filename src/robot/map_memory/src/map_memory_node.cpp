#include "map_memory_node.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

MemoryMapperNode::MemoryMapperNode()
: Node("memory_mapper"),
  mem_handler_(memory_ns::MapMemoryHandler(this->get_logger())),
  robot_px_(0.0), robot_py_(0.0), robot_yaw_(0.0),
  prev_x_(std::numeric_limits<double>::quiet_NaN()),
  prev_y_(std::numeric_limits<double>::quiet_NaN())
{
  // Load parameters (topics, resolution, etc.)
  loadParams();

  // Create subscribers
  local_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    local_costmap_topic_,
    10,
    std::bind(&MemoryMapperNode::onLocalMapReceived, this, std::placeholders::_1)
  );

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_,
    10,
    std::bind(&MemoryMapperNode::onOdomReceived, this, std::placeholders::_1)
  );

  // Create publisher
  map_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(merged_map_topic_, 10);

  // Create timer
  update_timer_ = create_wall_timer(
    std::chrono::milliseconds(publish_rate_ms_),
    std::bind(&MemoryMapperNode::onTimerEvent, this)
  );

  // Initialize global map in core
  mem_handler_.initializeMemory(global_res_, global_width_, global_height_, global_origin_);

  RCLCPP_INFO(get_logger(), "MemoryMapperNode initialized. Publish freq: %d ms", publish_rate_ms_);
}

void MemoryMapperNode::loadParams()
{
  // Declare & get parameters
  declare_parameter<std::string>("local_costmap_topic", "/costmap");
  declare_parameter<std::string>("odom_topic", "/odom/filtered");
  declare_parameter<std::string>("map_topic", "/map");
  declare_parameter<int>("map_pub_rate", 500);
  declare_parameter<double>("update_distance", 1.0);

  declare_parameter<double>("global_map.resolution", 0.1);
  declare_parameter<int>("global_map.width", 100);
  declare_parameter<int>("global_map.height", 100);
  declare_parameter<double>("global_map.origin.position.x", -5.0);
  declare_parameter<double>("global_map.origin.position.y", -5.0);
  declare_parameter<double>("global_map.origin.orientation.w", 1.0);

  local_costmap_topic_  = get_parameter("local_costmap_topic").as_string();
  odom_topic_           = get_parameter("odom_topic").as_string();
  merged_map_topic_     = get_parameter("map_topic").as_string();
  publish_rate_ms_      = get_parameter("map_pub_rate").as_int();
  distance_threshold_   = get_parameter("update_distance").as_double();

  global_res_           = get_parameter("global_map.resolution").as_double();
  global_width_         = get_parameter("global_map.width").as_int();
  global_height_        = get_parameter("global_map.height").as_int();
  global_origin_.position.x = get_parameter("global_map.origin.position.x").as_double();
  global_origin_.position.y = get_parameter("global_map.origin.position.y").as_double();
  global_origin_.orientation.w = get_parameter("global_map.origin.orientation.w").as_double();
}

void MemoryMapperNode::onLocalMapReceived(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  // Example check for all-zero data
  bool all_zero = std::all_of(msg->data.begin(), msg->data.end(),
                              [](int8_t val){ return val == 0; });
  if (all_zero) {
    RCLCPP_INFO(get_logger(), "Local costmap is entirely zero, skipping merge.");
    return;
  }

  // Check movement distance
  if (!std::isnan(prev_x_)) {
    double dist_moved = std::hypot(robot_px_ - prev_x_, robot_py_ - prev_y_);
    if (dist_moved < distance_threshold_) {
      // Robot hasn’t moved enough since last merge
      return;
    }
  }
  // Update reference position
  prev_x_ = robot_px_;
  prev_y_ = robot_py_;

  // Merge new local costmap into global
  mem_handler_.mergeLocalCostmap(msg, robot_px_, robot_py_, robot_yaw_);
}

void MemoryMapperNode::onOdomReceived(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // Extract current pose
  robot_px_ = msg->pose.pose.position.x;
  robot_py_ = msg->pose.pose.position.y;

  double qx = msg->pose.pose.orientation.x;
  double qy = msg->pose.pose.orientation.y;
  double qz = msg->pose.pose.orientation.z;
  double qw = msg->pose.pose.orientation.w;
  robot_yaw_ = quaternionToYaw(qx, qy, qz, qw);
}

void MemoryMapperNode::onTimerEvent()
{
  // Publish global map at each timer tick
  auto grid_ptr = mem_handler_.retrieveMemoryGrid();
  nav_msgs::msg::OccupancyGrid outgoing_map = *grid_ptr;
  outgoing_map.header.stamp = now();
  outgoing_map.header.frame_id = "sim_world";
  map_publisher_->publish(outgoing_map);
}

double MemoryMapperNode::quaternionToYaw(double x, double y, double z, double w)
{
  tf2::Quaternion q(x, y, z, w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MemoryMapperNode>());
  rclcpp::shutdown();
  return 0;
}
