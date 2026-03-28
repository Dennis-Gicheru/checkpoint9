#ifndef MY_COMPONENTS__PRE_APPROACH_COMPONENT_HPP_
#define MY_COMPONENTS__PRE_APPROACH_COMPONENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>

namespace my_components
{
class PreApproach : public rclcpp::Node
{
public:
  explicit PreApproach(const rclcpp::NodeOptions & options);

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void start_rotation();

  int state_; 
  const double target_obstacle_dist_ = 0.3; 
  const int target_degrees_ = -90;          
  
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
};
} // namespace my_components

#endif