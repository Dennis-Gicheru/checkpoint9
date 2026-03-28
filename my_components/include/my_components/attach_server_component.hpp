#ifndef MY_COMPONENTS__ATTACH_SERVER_COMPONENT_HPP_
#define MY_COMPONENTS__ATTACH_SERVER_COMPONENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/empty.hpp"
#include "attach_shelf/srv/go_to_loading.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

namespace my_components
{
struct Point2D { double x; double y; };

class AttachServer : public rclcpp::Node
{
public:
  explicit AttachServer(const rclcpp::NodeOptions & options);

private:
  void handle_approach(
    const std::shared_ptr<attach_shelf::srv::GoToLoading::Request> request,
    std::shared_ptr<attach_shelf::srv::GoToLoading::Response> response);

  void broadcast_tf(double x, double y);

  // Subscribers & Publishers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr elevator_pub_;
  
  // Service & TF
  rclcpp::Service<attach_shelf::srv::GoToLoading>::SharedPtr service_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Data storage
  sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
  nav_msgs::msg::Odometry::SharedPtr last_odom_;
  rclcpp::CallbackGroup::SharedPtr cb_group_;
};
} // namespace my_components

#endif