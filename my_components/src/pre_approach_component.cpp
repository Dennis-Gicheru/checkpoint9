#include "my_components/pre_approach_component.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <cmath>

namespace my_components
{
PreApproach::PreApproach(const rclcpp::NodeOptions & options)
: Node("pre_approach", options), state_(0), initial_yaw_(0.0)
{
  // Acquire parameters 
  this->declare_parameter<double>("obstacle", 0.5);
  this->declare_parameter<int>("degrees", -90);
  target_obstacle_dist_ = this->get_parameter("obstacle").as_double();
  target_degrees_ = this->get_parameter("degrees").as_int();

  publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "/diffbot_base_controller/cmd_vel_unstamped", 10);
  
  subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", rclcpp::SensorDataQoS(),
    std::bind(&PreApproach::scan_callback, this, std::placeholders::_1));

  // Odometry Subscription
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10,
    std::bind(&PreApproach::odom_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Pre-approach Odom Component initialized.");
}

// Convert Quaternion to Yaw 
double PreApproach::get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion& q)
{
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

void PreApproach::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  if (state_ != 0) return; 

  auto twist = geometry_msgs::msg::Twist();
  int center_idx = msg->ranges.size() / 2;

  if (msg->ranges[center_idx] > target_obstacle_dist_) {
    twist.linear.x = 0.2;
    publisher_->publish(twist);
  } else {
    twist.linear.x = 0.0;
    publisher_->publish(twist);
    state_ = 1; 
    RCLCPP_INFO(this->get_logger(), "Obstacle reached. Initializing Odom rotation.");
  }
}

void PreApproach::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (state_ == 0 || state_ == 3) return;

  double current_yaw = get_yaw_from_quaternion(msg->pose.pose.orientation);

  // STATE 1: Capture starting angle once
  if (state_ == 1) {
    initial_yaw_ = current_yaw;
    state_ = 2; // Move to active rotation
    return;
  }

  // STATE 2: Monitor rotation progress
  if (state_ == 2) {
    double target_rad = target_degrees_ * M_PI / 180.0;
    double current_rotated = current_yaw - initial_yaw_;

    // Handle the -PI to PI wrap-around
    while (current_rotated > M_PI) current_rotated -= 2.0 * M_PI;
    while (current_rotated < -M_PI) current_rotated += 2.0 * M_PI;

    auto twist = geometry_msgs::msg::Twist();
    double error = std::abs(target_rad) - std::abs(current_rotated);

    if (error > 0.02) { // 0.02 rad tolerance for high precision
      // Slowly reduce speed as we get closer to prevent "losing degrees" from inertia
      double speed = (error < 0.2) ? 0.15 : 0.3; 
      twist.angular.z = (target_degrees_ < 0) ? -speed : speed;
      publisher_->publish(twist);
    } else {
      // STOP
      twist.angular.z = 0.0;
      publisher_->publish(twist);
      state_ = 3; 
      RCLCPP_INFO(this->get_logger(), "Perfect %d degree turn completed via Odometry.", target_degrees_);
    }
  }
}
} // namespace my_components

RCLCPP_COMPONENTS_REGISTER_NODE(my_components::PreApproach)