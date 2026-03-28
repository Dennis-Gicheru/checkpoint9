#include "my_components/pre_approach_component.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace my_components
{
Preapproach::Preapproach(const rclcpp::NodeOptions & options)
: Node("pre_approach", options), state_(0)
{
  publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "/diffbot_base_controller/cmd_vel_unstamped", 10);
  
  subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", rclcpp::SensorDataQoS(),
    std::bind(&Preapproach::scan_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Preapproach Component initialized in my_components package.");
}

void Preapproach::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  if (state_ == 2) return;
  auto twist = geometry_msgs::msg::Twist();
  int center_idx = msg->ranges.size() / 2;

  if (state_ == 0) { // Forward
    if (msg->ranges[center_idx] > target_obstacle_dist_) {
      twist.linear.x = 0.2;
      publisher_->publish(twist);
    } else {
      twist.linear.x = 0.0;
      publisher_->publish(twist);
      start_rotation();
    }
  } else if (state_ == 1) { // Rotate
    twist.angular.z = -0.5; // -90 degrees hardcoded direction
    publisher_->publish(twist);
  }
}

void Preapproach::start_rotation()
{
  state_ = 1;
  double rotation_time = (std::abs(target_degrees_ * M_PI / 180.0)) / 0.5;

  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(rotation_time),
    [this]() {
      state_ = 2;
      auto stop = geometry_msgs::msg::Twist();
      publisher_->publish(stop);
      timer_->cancel();
      RCLCPP_INFO(this->get_logger(), "Rotation complete. Preapproach finished.");
    });
}
} // namespace my_components

RCLCPP_COMPONENTS_REGISTER_NODE(my_components::PreApproach)