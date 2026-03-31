#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <cmath>

class PreApproach : public rclcpp::Node {
public:
    PreApproach() : Node("pre_approach"), state_(0), initial_yaw_(0.0) {
        // Declare and acquire parameters
        this->declare_parameter<double>("obstacle", 0.5);
        this->declare_parameter<int>("degrees", 90);
        obstacle_dist_ = this->get_parameter("obstacle").as_double();
        degrees_ = this->get_parameter("degrees").as_int();

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/diffbot_base_controller/cmd_vel_unstamped", 10);
        
        subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            std::bind(&PreApproach::scan_callback, this, std::placeholders::_1));
            
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&PreApproach::odom_callback, this, std::placeholders::_1));
            
        RCLCPP_INFO(this->get_logger(), "Pre-approach node initialized. Moving forward.");
    }

private:
    // Helper function to extract yaw from a quaternion without needing tf2 dependencies
    double get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion& q) {
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        return std::atan2(siny_cosp, cosy_cosp);
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        if (state_ != 0) return;

        auto twist = geometry_msgs::msg::Twist();
        int center_idx = msg->ranges.size() / 2; 
        double front_distance = msg->ranges[center_idx];

        if (front_distance > obstacle_dist_) {
            twist.linear.x = 0.2; 
            publisher_->publish(twist);
        } else {
            twist.linear.x = 0.0;
            publisher_->publish(twist);
            RCLCPP_INFO(this->get_logger(), "Obstacle reached. Starting rotation.");
            state_ = 1; // Trigger odometry-based rotation
        }
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (state_ == 0 || state_ == 3) return;

        double current_yaw = get_yaw_from_quaternion(msg->pose.pose.orientation);

        if (state_ == 1) {
            initial_yaw_ = current_yaw;
            state_ = 2; // Move to active rotation state
        }

        if (state_ == 2) {
            double target_yaw = initial_yaw_ + (degrees_ * M_PI / 180.0);
            
            // Calculate the shortest angular distance (handles the -PI to PI wrap around)
            double angle_diff = std::atan2(std::sin(target_yaw - current_yaw), std::cos(target_yaw - current_yaw));

            auto twist = geometry_msgs::msg::Twist();
            if (std::abs(angle_diff) > 0.05) { 
                twist.angular.z = (angle_diff > 0) ? 0.3 : -0.3;
                publisher_->publish(twist);
            } else {
                twist.angular.z = 0.0;
                publisher_->publish(twist);
                RCLCPP_INFO(this->get_logger(), "Rotation complete. Terminating pre-approach node.");
                state_ = 3; 
                rclcpp::shutdown();
            }
        }
    }

    int state_; // 0: Forward, 1: Init Rotation, 2: Rotating, 3: Done
    double obstacle_dist_;
    int degrees_;
    double initial_yaw_;
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PreApproach>());
    return 0;
}