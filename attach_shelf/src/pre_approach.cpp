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
        // Prevent callback execution if the node is shutting down
        if (state_ == 2) return;

        auto twist = geometry_msgs::msg::Twist();
        
        // Isolate the ray directly in front of the robot
        int center_idx = msg->ranges.size() / 2; 
        double front_distance = msg->ranges[center_idx];

        if (state_ == 0) { // State 0: Moving Forward
            if (front_distance > obstacle_dist_) {
                twist.linear.x = 0.2; 
                publisher_->publish(twist); // Continuous heartbeat
            } else {
                twist.linear.x = 0.0;
                publisher_->publish(twist); // Hard stop
                RCLCPP_INFO(this->get_logger(), "Obstacle reached. Starting rotation.");
                start_rotation();
            }
        } 
        else if (state_ == 1) { // State 1: Rotating
            // Continuously publish the rotation command to feed the velocity watchdog
            double angular_speed = 0.5; // rad/s
            twist.angular.z = (degrees_ > 0) ? angular_speed : -angular_speed;
            publisher_->publish(twist);
        }
    }

    void start_rotation() {
        state_ = 1; // Transition to rotating state
        
        // Convert degrees to radians and calculate required time
        double radians = degrees_ * (M_PI / 180.0);
        double angular_speed = 0.5; // rad/s
        double rotation_time = std::abs(radians) / angular_speed;

        // Timer to stop rotation and kill the node after calculated time
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(rotation_time),
            [this]() {
                state_ = 2; // Done
                
                // 1. Publish the final hard stop to prevent drifting
                auto stop_twist = geometry_msgs::msg::Twist();
                stop_twist.linear.x = 0.0;
                stop_twist.angular.z = 0.0;
                publisher_->publish(stop_twist);

                RCLCPP_INFO(this->get_logger(), "Rotation complete. Terminating pre-approach node.");
                
                // 2. Cancel the timer and gracefully shutdown the ROS 2 node
                timer_->cancel();
                rclcpp::shutdown();
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
    // rclcpp::shutdown() is called internally by the timer, unblocking spin()
    return 0;
}