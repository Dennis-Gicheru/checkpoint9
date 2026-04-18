#include "my_components/attach_server_component.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <cmath>
#include <vector>

namespace my_components
{
AttachServer::AttachServer(const rclcpp::NodeOptions & options)
: Node("attach_server", options)
{
  cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = cb_group_;

  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) { last_scan_ = msg; }, sub_opts);

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10,
    [this](const nav_msgs::msg::Odometry::SharedPtr msg) { last_odom_ = msg; }, sub_opts);

  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/diffbot_base_controller/cmd_vel_unstamped", 10);
  elevator_pub_ = this->create_publisher<std_msgs::msg::Empty>("/elevator_up", 10);
  
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  service_ = this->create_service<attach_shelf::srv::GoToLoading>(
    "/approach_shelf",
    std::bind(&AttachServer::handle_approach, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, cb_group_);

  RCLCPP_INFO(this->get_logger(), "AttachServer Component Ready - Precision Mode.");
}

void AttachServer::handle_approach(
  const std::shared_ptr<attach_shelf::srv::GoToLoading::Request> request,
  std::shared_ptr<attach_shelf::srv::GoToLoading::Response> response)
{
  if (!request->attach_to_shelf) {
    response->complete = false;
    return;
  }

  rclcpp::Rate rate(10);
  bool reached_front = false;

  // --- PHASE 1: ALIGNED APPROACH (Visual Servoing) ---
  while (rclcpp::ok() && !reached_front) {
    if (!last_scan_) { 
      rate.sleep(); 
      continue; 
    }

    // 1. Leg Detection
    std::vector<Point2D> high_intensity;
    for (size_t i = 0; i < last_scan_->ranges.size(); ++i) {
      if (last_scan_->intensities[i] >= 8000.0) {
        double angle = last_scan_->angle_min + (i * last_scan_->angle_increment);
        high_intensity.push_back({last_scan_->ranges[i] * std::cos(angle), 
                                  last_scan_->ranges[i] * std::sin(angle)});
      }
    }

    // 2. Grouping legs
    std::vector<Point2D> leg1, leg2;
    if (high_intensity.size() >= 2) {
      leg1.push_back(high_intensity[0]);
      for (size_t i = 1; i < high_intensity.size(); ++i) {
        double d = std::hypot(high_intensity[i].x - high_intensity[i-1].x, 
                              high_intensity[i].y - high_intensity[i-1].y);
        if (d > 0.1) leg2.push_back(high_intensity[i]);
        else (leg2.empty() ? leg1 : leg2).push_back(high_intensity[i]);
      }
    }

    if (leg1.empty() || leg2.empty()) {
      cmd_pub_->publish(geometry_msgs::msg::Twist());
      rate.sleep();
      continue; 
    }

    // 3. Broadcast TF
    auto avg = [](const std::vector<Point2D>& pts) {
      double sx=0, sy=0; for(auto p : pts) { sx+=p.x; sy+=p.y; }
      return Point2D{sx/(double)pts.size(), sy/(double)pts.size()};
    };
    Point2D mid = {(avg(leg1).x + avg(leg2).x) / 2.0, (avg(leg1).y + avg(leg2).y) / 2.0};
    
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "robot_front_laser_base_link";
    t.child_frame_id = "cart_frame";
    t.transform.translation.x = mid.x;
    t.transform.translation.y = mid.y;
    t.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(t);

    // 4. Navigation Control with Dampened Steering
    try {
      auto transform = tf_buffer_->lookupTransform("robot_base_link", "cart_frame", tf2::TimePointZero);
      double dx = transform.transform.translation.x;
      double dy = transform.transform.translation.y;
      double dist = std::hypot(dx, dy);
      double heading_err = std::atan2(dy, dx);

      if (dist < 0.35) {
        reached_front = true;
        cmd_pub_->publish(geometry_msgs::msg::Twist()); 
      } else {
        auto twist = geometry_msgs::msg::Twist();
        
        // SLOW DOWN ON CURVES: Reduce speed if heading error is high
        double base_speed = 0.08;
        twist.linear.x = (std::abs(heading_err) > 0.05) ? (base_speed * 0.5) : base_speed;

        // DAMPENED P-CONTROLLER: Gain 0.6 prevents aggressive swerving
        if (std::abs(heading_err) > 0.01) { 
          twist.angular.z = 0.6 * heading_err; 
        } else {
          twist.angular.z = 0.0;
        }
        cmd_pub_->publish(twist);
      }
    } catch (const tf2::TransformException & ex) { (void)ex; }

    rate.sleep(); 
  }

  // --- PHASE 2: FINAL ODOMETRY PUSH (30cm) ---
  if (reached_front && last_odom_) {
    RCLCPP_INFO(this->get_logger(), "Starting final push phase.");
    double sx = last_odom_->pose.pose.position.x;
    double sy = last_odom_->pose.pose.position.y;
    double traveled = 0.0;
    
    while (rclcpp::ok() && traveled < 0.30) { 
      traveled = std::hypot(last_odom_->pose.pose.position.x - sx, 
                            last_odom_->pose.pose.position.y - sy);
      auto twist = geometry_msgs::msg::Twist();
      twist.linear.x = 0.05; // Snail speed
      twist.angular.z = 0.0; // Straight
      cmd_pub_->publish(twist);
      rate.sleep();
    }
    
    // Stop and Lift
    cmd_pub_->publish(geometry_msgs::msg::Twist()); 
    rclcpp::sleep_for(std::chrono::seconds(1));
    elevator_pub_->publish(std_msgs::msg::Empty()); 
    
    RCLCPP_INFO(this->get_logger(), "Success: Docking complete.");
    response->complete = true;
  }
}

void AttachServer::broadcast_tf(double x, double y) { (void)x; (void)y; }

} // namespace my_components

RCLCPP_COMPONENTS_REGISTER_NODE(my_components::AttachServer)