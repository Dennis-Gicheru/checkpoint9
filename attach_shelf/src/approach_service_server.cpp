#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/empty.hpp" 
#include "attach_shelf/srv/go_to_loading.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <cmath>
#include <vector>
#include <algorithm>
#include <chrono>

using namespace std::chrono_literals;

struct Point2D { double x; double y; };

class ApproachServiceServer : public rclcpp::Node {
public:
    ApproachServiceServer() : Node("approach_service_server") {
        
        // while the service loop is running.
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
            std::bind(&ApproachServiceServer::handle_approach, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default, cb_group_);

        RCLCPP_INFO(this->get_logger(), "Standalone Approach Service Server Ready. Waiting for service call...");
    }

private:
    void find_legs(std::vector<Point2D>& leg1, std::vector<Point2D>& leg2) {
        leg1.clear(); leg2.clear();
        if (!last_scan_) return;

        std::vector<Point2D> high_intensity;
        for (size_t i = 0; i < last_scan_->ranges.size(); ++i) {
            if (last_scan_->intensities[i] >= 8000.0) {
                double angle = last_scan_->angle_min + (i * last_scan_->angle_increment);
                high_intensity.push_back({last_scan_->ranges[i] * std::cos(angle), 
                                          last_scan_->ranges[i] * std::sin(angle)});
            }
        }

        if (high_intensity.empty()) return;
        leg1.push_back(high_intensity[0]);
        for (size_t i = 1; i < high_intensity.size(); ++i) {
            double dist = std::hypot(high_intensity[i].x - high_intensity[i-1].x, 
                                     high_intensity[i].y - high_intensity[i-1].y);
            if (dist > 0.1) leg2.push_back(high_intensity[i]);
            else if (leg2.empty()) leg1.push_back(high_intensity[i]);
            else leg2.push_back(high_intensity[i]);
        }
    }

    void handle_approach(const std::shared_ptr<attach_shelf::srv::GoToLoading::Request> request,
                         std::shared_ptr<attach_shelf::srv::GoToLoading::Response> response) {
        
        if (!request->attach_to_shelf) {
            response->complete = false;
            return; 
        }

        if (!last_scan_) rclcpp::sleep_for(500ms);

        // 1. Check Legs
        std::vector<Point2D> leg1, leg2;
        find_legs(leg1, leg2);

        if (leg1.empty() || leg2.empty()) {
            RCLCPP_WARN(this->get_logger(), "Detected 1 or 0 legs. Returning False.");
            response->complete = false;
            return; 
        }

        RCLCPP_INFO(this->get_logger(), "Both legs detected! Starting TF navigation.");
        
        rclcpp::Rate rate(10);
        bool reached_tf = false; 

        // 2 & 3. Broadcast TF and Navigate
        while (rclcpp::ok() && !reached_tf) {
            find_legs(leg1, leg2);

            if (!leg1.empty() && !leg2.empty()) {
                auto get_c = [](const std::vector<Point2D>& pts) {
                    Point2D c = {0, 0};
                    for (const auto& p : pts) { c.x += p.x; c.y += p.y; }
                    return Point2D{c.x / pts.size(), c.y / pts.size()};
                };
                Point2D mid = {(get_c(leg1).x + get_c(leg2).x) / 2.0, 
                               (get_c(leg1).y + get_c(leg2).y) / 2.0};
                broadcast_tf(mid.x, mid.y);
            }

            try {
                auto t = tf_buffer_->lookupTransform("robot_base_link", "cart_frame", tf2::TimePointZero);
                double err_x = t.transform.translation.x;
                double err_y = t.transform.translation.y;
                
                double distance_to_tf = std::hypot(err_x, err_y);

                if (distance_to_tf < 0.15) { 
                    reached_tf = true; 
                    auto stop_twist = geometry_msgs::msg::Twist();
                    cmd_pub_->publish(stop_twist);
                    RCLCPP_INFO(this->get_logger(), "Reached TF Coordinates.");
                } else {
                    double heading_error = std::atan2(err_y, err_x);
                    auto twist = geometry_msgs::msg::Twist();
                    
                    twist.angular.z = std::clamp(1.0 * heading_error, -0.4, 0.4); 
                    if (std::abs(heading_error) < 0.1) {
                        twist.linear.x = 0.15; 
                    } else {
                        twist.linear.x = 0.0; 
                    }
                    cmd_pub_->publish(twist);
                }
            } catch (const tf2::TransformException & ex) {
                auto twist = geometry_msgs::msg::Twist();
                cmd_pub_->publish(twist);
            }
            rate.sleep(); 
        }

        // 4. Odometry Push (30cm)
        if (reached_tf && last_odom_) {
            RCLCPP_INFO(this->get_logger(), "Executing final 30cm odometry push.");
            
            double start_x = last_odom_->pose.pose.position.x;
            double start_y = last_odom_->pose.pose.position.y;
            double dist_traveled = 0.0;

            while (rclcpp::ok() && dist_traveled < 0.30) {
                dist_traveled = std::hypot(last_odom_->pose.pose.position.x - start_x, 
                                           last_odom_->pose.pose.position.y - start_y);
                auto twist = geometry_msgs::msg::Twist();
                twist.linear.x = 0.15;
                twist.angular.z = 0.0; 
                cmd_pub_->publish(twist);
                rate.sleep();
            }

            auto stop_twist = geometry_msgs::msg::Twist();
            cmd_pub_->publish(stop_twist); 

            auto msg = std_msgs::msg::Empty();
            elevator_pub_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Elevator UP command published.");

            response->complete = true;

            rclcpp::sleep_for(500ms);
            RCLCPP_INFO(this->get_logger(), "Task complete. Shutting down.");
            rclcpp::shutdown();
        }
    }

    void broadcast_tf(double x, double y) {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "robot_front_laser_base_link";
        t.child_frame_id = "cart_frame";
        t.transform.translation.x = x;
        t.transform.translation.y = y;
        t.transform.rotation.w = 1.0;
        tf_broadcaster_->sendTransform(t);
    }

    sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
    nav_msgs::msg::Odometry::SharedPtr last_odom_;
    rclcpp::CallbackGroup::SharedPtr cb_group_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr elevator_pub_; 
    rclcpp::Service<attach_shelf::srv::GoToLoading>::SharedPtr service_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

// STANDARD EXECUTABLE MAIN FUNCTION
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    
    // MultiThreadedExecutor allows the callbacks and the service loop to run concurrently
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<ApproachServiceServer>();
    executor.add_node(node);
    
    executor.spin();
    rclcpp::shutdown();
    return 0;
}