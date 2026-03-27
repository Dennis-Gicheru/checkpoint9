#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "attach_shelf/srv/go_to_loading.hpp" // Header for the custom service
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

class PreApproachV2 : public rclcpp::Node {
public:
    PreApproachV2() : Node("pre_approach_v2"), state_(0) {
        // 1. Declare and acquire parameters from the launch file
        this->declare_parameter<double>("obstacle", 0.3);
        this->declare_parameter<int>("degrees", -90);
        this->declare_parameter<bool>("final_approach", false);

        obstacle_dist_ = this->get_parameter("obstacle").as_double();
        degrees_ = this->get_parameter("degrees").as_int();
        final_approach_ = this->get_parameter("final_approach").as_bool();

        // 2. Setup Publishers, Subscribers, and Service Client
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/diffbot_base_controller/cmd_vel_unstamped", 10);
        
        subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            std::bind(&PreApproachV2::scan_callback, this, std::placeholders::_1));

        client_ = this->create_client<attach_shelf::srv::GoToLoading>("/approach_shelf");

        RCLCPP_INFO(this->get_logger(), "Pre-approach V2 initialized. Final Approach set to: %s", 
                    final_approach_ ? "TRUE" : "FALSE");
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        if (state_ >= 2) return; // Prevent logic overlap during service call

        auto twist = geometry_msgs::msg::Twist();
        int center_idx = msg->ranges.size() / 2; 
        double front_distance = msg->ranges[center_idx];

        if (state_ == 0) { // Moving Forward
            if (front_distance > obstacle_dist_) {
                twist.linear.x = 0.2; 
                publisher_->publish(twist);
            } else {
                twist.linear.x = 0.0;
                publisher_->publish(twist);
                RCLCPP_INFO(this->get_logger(), "Obstacle reached. Starting rotation.");
                start_rotation();
            }
        } 
        else if (state_ == 1) { // Rotating
            double angular_speed = 0.5;
            twist.angular.z = (degrees_ > 0) ? angular_speed : -angular_speed;
            publisher_->publish(twist);
        }
    }

    void start_rotation() {
        state_ = 1;
        double radians = degrees_ * (M_PI / 180.0);
        double angular_speed = 0.5;
        double rotation_time = std::abs(radians) / angular_speed;

        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(rotation_time),
            std::bind(&PreApproachV2::on_timer_complete, this));
    }

    void on_timer_complete() {
        state_ = 2; // Movement Done
        timer_->cancel();

        // Stop the robot
        auto stop_twist = geometry_msgs::msg::Twist();
        publisher_->publish(stop_twist);
        RCLCPP_INFO(this->get_logger(), "Rotation complete.");

        // DECISION LOGIC: Call service or Terminate
        if (final_approach_) {
            call_approach_service();
        } else {
            RCLCPP_INFO(this->get_logger(), "Final approach disabled. Shutting down.");
            rclcpp::shutdown();
        }
    }

    void call_approach_service() {
        // Wait for service to be available
        while (!client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for /approach_shelf service...");
        }

        auto request = std::make_shared<attach_shelf::srv::GoToLoading::Request>();
        request->attach_to_shelf = true;

        RCLCPP_INFO(this->get_logger(), "Calling /approach_shelf service...");
        
        auto result = client_->async_send_request(request, 
            [this](rclcpp::Client<attach_shelf::srv::GoToLoading>::SharedFuture future) {
                auto response = future.get();
                if (response->complete) {
                    RCLCPP_INFO(this->get_logger(), "Approach Service SUCCESSFUL.");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Approach Service FAILED.");
                }
                rclcpp::shutdown(); // Kill this node once the mission is over
            });
    }

    int state_; // 0: Forward, 1: Rotating, 2: Finished/Service Call
    double obstacle_dist_;
    int degrees_;
    bool final_approach_;
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
    rclcpp::Client<attach_shelf::srv::GoToLoading>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PreApproachV2>());
    rclcpp::shutdown();
    return 0;
}