#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>

class PreApproach : public rclcpp::Node {
public:
    PreApproach() : Node("pre_approach"), state_(0) {
        // Declare and acquire parameters
        this->declare_parameter<double>("obstacle", 0.5);
        this->declare_parameter<int>("degrees", 90);
        obstacle_dist_ = this->get_parameter("obstacle").as_double();
        degrees_ = this->get_parameter("degrees").as_int();

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/diffbot_base_controller/cmd_vel_unstamped", 10);
        subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            std::bind(&PreApproach::scan_callback, this, std::placeholders::_1));
            
        RCLCPP_INFO(this->get_logger(), "Pre-approach node initialized. Moving forward.");
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        auto twist = geometry_msgs::msg::Twist();
        
        // Isolate the ray directly in front of the robot (assuming 0 is front or center of array)
        int center_idx = msg->ranges.size() / 2; 
        double front_distance = msg->ranges[center_idx];

        if (state_ == 0) { // Moving Forward
            if (front_distance > obstacle_dist_) {
                twist.linear.x = 0.2; // Move forward at 0.2 m/s
            } else {
                twist.linear.x = 0.0;
                publisher_->publish(twist); // Stop immediately
                RCLCPP_INFO(this->get_logger(), "Obstacle reached. Starting rotation.");
                start_rotation();
            }
        } 
        
        if (state_ == 0) {
             publisher_->publish(twist);
        }
    }

    void start_rotation() {
        state_ = 1;
        auto twist = geometry_msgs::msg::Twist();
        
        // Convert degrees to radians
        double radians = degrees_ * (M_PI / 180.0);
        double angular_speed = 0.5; // rad/s
        double rotation_time = std::abs(radians) / angular_speed;
        
        twist.angular.z = (degrees_ > 0) ? angular_speed : -angular_speed;
        publisher_->publish(twist);

        // Timer to stop rotation after calculated time
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(rotation_time),
            [this]() {
                auto stop_twist = geometry_msgs::msg::Twist();
                publisher_->publish(stop_twist);
                RCLCPP_INFO(this->get_logger(), "Rotation complete. Final position achieved.");
                state_ = 2; // Done
                timer_->cancel();
            });
    }

    int state_; // 0: Forward, 1: Rotating, 2: Done
    double obstacle_dist_;
    int degrees_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PreApproach>());
    rclcpp::shutdown();
    return 0;
}