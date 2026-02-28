#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "custom_interfaces/srv/go_to_loading.hpp"
#include <cmath>

class PreApproachV2 : public rclcpp::Node {
public:
    PreApproachV2() : Node("pre_approach_v2"), state_(0) {
        this->declare_parameter<double>("obstacle", 0.5);
        this->declare_parameter<int>("degrees", 90);
        this->declare_parameter<bool>("final_approach", true); // New Parameter
        
        obstacle_dist_ = this->get_parameter("obstacle").as_double();
        degrees_ = this->get_parameter("degrees").as_int();
        final_approach_ = this->get_parameter("final_approach").as_bool();

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/diffbot_base_controller/cmd_vel_unstamped", 10);
        subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            std::bind(&PreApproachV2::scan_callback, this, std::placeholders::_1));
            
        approach_client_ = this->create_client<custom_interfaces::srv::GoToLoading>("/approach_shelf");
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        auto twist = geometry_msgs::msg::Twist();
        int center_idx = msg->ranges.size() / 2; 
        double front_distance = msg->ranges[center_idx];

        if (state_ == 0) {
            if (front_distance > obstacle_dist_) {
                twist.linear.x = 0.2; 
                publisher_->publish(twist);
            } else {
                twist.linear.x = 0.0;
                publisher_->publish(twist); 
                start_rotation();
            }
        } else if (state_ == 1) {
            double angular_speed = 0.5;
            twist.angular.z = (degrees_ > 0) ? angular_speed : -angular_speed;
            publisher_->publish(twist);
        }
    }

    void start_rotation() {
        state_ = 1;
        double radians = degrees_ * (M_PI / 180.0);
        double rotation_time = std::abs(radians) / 0.5;
        
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(rotation_time),
            [this]() {
                state_ = 2; 
                auto stop_twist = geometry_msgs::msg::Twist();
                publisher_->publish(stop_twist);
                timer_->cancel();
                
                // Call the final approach service
                call_approach_service();
            });
    }

    void call_approach_service() {
        RCLCPP_INFO(this->get_logger(), "Pre-approach complete. Calling /approach_shelf service.");
        while (!approach_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Waiting for /approach_shelf service to be available...");
        }

        auto request = std::make_shared<custom_interfaces::srv::GoToLoading::Request>();
        request->attach_to_shelf = final_approach_;

        approach_client_->async_send_request(request, 
            [this](rclcpp::Client<custom_interfaces::srv::GoToLoading>::SharedFuture response) {
                if (response.get()->complete) {
                    RCLCPP_INFO(this->get_logger(), "Final approach executed successfully.");
                } else {
                    RCLCPP_WARN(this->get_logger(), "Final approach failed or was bypassed.");
                }
            });
    }

    int state_;
    double obstacle_dist_;
    int degrees_;
    bool final_approach_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<custom_interfaces::srv::GoToLoading>::SharedPtr approach_client_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PreApproachV2>());
    rclcpp::shutdown();
    return 0;
}