#ifndef MY_COMPONENTS__PRE_APPROACH_COMPONENT_HPP_
#define MY_COMPONENTS__PRE_APPROACH_COMPONENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp" // Added for Odometry
#include <cmath>

namespace my_components
{
class PreApproach : public rclcpp::Node
{
public:
  explicit PreApproach(const rclcpp::NodeOptions & options);

private:
  // Callbacks
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg); // Added

  // Helper Math Function
  double get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q); // Added

  // Logic Variables
  int state_; 
  double initial_yaw_; // Added to store starting orientation
  
  // Parameters 
  double target_obstacle_dist_; 
  int target_degrees_;          
  
  // ROS 2 Interfaces
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_; // Added
};
} // namespace my_components

#endif