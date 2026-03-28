#include "my_components/attach_server_component.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

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

  RCLCPP_INFO(this->get_logger(), "AttachServer Component Ready.");
}

void AttachServer::handle_approach(
  const std::shared_ptr<attach_shelf::srv::GoToLoading::Request> request,
  std::shared_ptr<attach_shelf::srv::GoToLoading::Response> response)
{
  (void)request; 
  rclcpp::Rate rate(10);
  bool reached = false;

  while (rclcpp::ok() && !reached) {
    if (!last_scan_) { rate.sleep(); continue; }

    // 1. Leg Detection Logic
    std::vector<Point2D> high_intensity_points;
    for (size_t i = 0; i < last_scan_->ranges.size(); ++i) {
      if (last_scan_->intensities[i] >= 8000.0) {
        double angle = last_scan_->angle_min + (i * last_scan_->angle_increment);
        high_intensity_points.push_back({last_scan_->ranges[i] * std::cos(angle), 
                                         last_scan_->ranges[i] * std::sin(angle)});
      }
    }

    std::vector<Point2D> leg1, leg2;
    if (high_intensity_points.size() >= 2) {
      leg1.push_back(high_intensity_points[0]);
      for (size_t i = 1; i < high_intensity_points.size(); ++i) {
        double d = std::hypot(high_intensity_points[i].x - high_intensity_points[i-1].x, 
                              high_intensity_points[i].y - high_intensity_points[i-1].y);
        if (d > 0.1) leg2.push_back(high_intensity_points[i]);
        else if (leg2.empty()) leg1.push_back(high_intensity_points[i]);
        else leg2.push_back(high_intensity_points[i]);
      }
    }

    // SAFETY CATCH: Stop moving if legs disappear
    if (leg1.empty() || leg2.empty()) { 
      auto stop_twist = geometry_msgs::msg::Twist();
      cmd_pub_->publish(stop_twist);
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for shelf legs...");
      rate.sleep(); 
      continue; 
    }

    // 2. Calculate the center of the cart in the Laser's frame
    auto get_c = [](const std::vector<Point2D>& pts) {
      double sx=0, sy=0;
      for(auto p : pts) { sx+=p.x; sy+=p.y; }
      return Point2D{sx/pts.size(), sy/pts.size()};
    };
    Point2D p1 = get_c(leg1), p2 = get_c(leg2);
    
    double cart_x = (p1.x + p2.x) / 2.0;
    double cart_y = (p1.y + p2.y) / 2.0;

    // 3. Broadcast TF dynamically to RViz
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = last_scan_->header.stamp;
    t.header.frame_id = last_scan_->header.frame_id;
    t.child_frame_id = "cart_frame";
    t.transform.translation.x = cart_x;
    t.transform.translation.y = cart_y;
    t.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(t);

    // 4. Tank-like straight movement (Zero rotation)
    double distance_to_cart = std::hypot(cart_x, cart_y);

    if (distance_to_cart < 0.35) {
      reached = true;
      auto stop_twist = geometry_msgs::msg::Twist();
      cmd_pub_->publish(stop_twist);
      RCLCPP_INFO(this->get_logger(), "Reached front of shelf. Starting 30cm penetration.");
    } else {
      auto twist = geometry_msgs::msg::Twist();
      twist.linear.x = 0.15;  // Drive strictly forward
      twist.angular.z = 0.0;  // Zero rotation
      cmd_pub_->publish(twist);
    }
    
    rate.sleep();
  }

  // 5. Final Odom-based push (Extra 30cm straight under the shelf)
  if (reached && last_odom_) {
    double sx = last_odom_->pose.pose.position.x;
    double sy = last_odom_->pose.pose.position.y;
    double distance_traveled = 0.0;
    
    while (rclcpp::ok() && distance_traveled < 0.30) { 
      distance_traveled = std::hypot(last_odom_->pose.pose.position.x - sx, 
                                     last_odom_->pose.pose.position.y - sy);
                                     
      auto twist = geometry_msgs::msg::Twist();
      twist.linear.x = 0.15; // Drive strictly forward
      twist.angular.z = 0.0; // Zero rotation
      cmd_pub_->publish(twist);
      rate.sleep();
    }
    
    // Stop and Lift
    cmd_pub_->publish(geometry_msgs::msg::Twist()); 
    elevator_pub_->publish(std_msgs::msg::Empty()); 
    response->complete = true;
    RCLCPP_INFO(this->get_logger(), "Shelf Attached Successfully.");
  }
}

void AttachServer::broadcast_tf(double x, double y)
{
  // TF logic handled dynamically inside the loop, keep this to satisfy header
  (void)x;
  (void)y;
}
} // namespace my_components

RCLCPP_COMPONENTS_REGISTER_NODE(my_components::AttachServer)