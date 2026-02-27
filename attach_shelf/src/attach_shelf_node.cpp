#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <cmath>

class AttachShelfNode : public rclcpp::Node {
public:
    AttachShelfNode() : Node("attach_shelf_node"), state_(0) {
        // QoS Profile tailored for high-frequency sensor data
        rclcpp::QoS sensor_qos(rclcpp::KeepLast(10));
        sensor_qos.best_effort();
        sensor_qos.durability_volatile();

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", sensor_qos, std::bind(&AttachShelfNode::scan_callback, this, std::placeholders::_1));
            
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/diffbot_base_controller/cmd_vel_unstamped", 10);
        
        elevator_client_ = this->create_client<std_srvs::srv::Empty>("/elevator_up");
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        RCLCPP_INFO(this->get_logger(), "Attach Shelf Node Started. Awaiting laser data...");
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        if (state_ == 0) {
            // STEP 1: PERCEPTION
            // TODO: Parse msg->ranges to find the two shelf legs.
            // Calculate the midpoint (target_x, target_y) between the two legs.
            
            // For now, let's assume you calculated the midpoint 1.0 meter straight ahead:
            double target_x = 1.0; 
            double target_y = 0.0;
            
            broadcast_shelf_frame(target_x, target_y);
            
            // Once found and broadcasted, transition to approach
            // state_ = 1; 
        } 
        else if (state_ == 1) {
            // STEP 2: KINEMATICS (Approaching)
            // Execute a P-controller to drive towards the broadcasted TF frame
            auto twist = geometry_msgs::msg::Twist();
            twist.linear.x = 0.2; // Move forward
            
            // TODO: Calculate distance error. If distance < 0.1m, stop and lift.
            bool is_under_shelf = false; 
            
            if (is_under_shelf) {
                twist.linear.x = 0.0;
                twist.angular.z = 0.0;
                cmd_pub_->publish(twist);
                state_ = 2;
                call_elevator_service();
            } else {
                cmd_pub_->publish(twist);
            }
        }
    }

    void broadcast_shelf_frame(double x, double y) {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "robot_front_laser_base_link"; // Parent frame
        t.child_frame_id = "cart_frame";                   // Child frame

        t.transform.translation.x = x;
        t.transform.translation.y = y;
        t.transform.translation.z = 0.0;
        
        // No rotation needed for the point
        t.transform.rotation.x = 0.0;
        t.transform.rotation.y = 0.0;
        t.transform.rotation.z = 0.0;
        t.transform.rotation.w = 1.0;

        tf_broadcaster_->sendTransform(t);
        RCLCPP_INFO_ONCE(this->get_logger(), "Broadcasting cart_frame...");
    }

    void call_elevator_service() {
        RCLCPP_INFO(this->get_logger(), "Position reached. Calling elevator service...");
        while (!elevator_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Waiting for /elevator_up service...");
        }

        auto request = std::make_shared<std_srvs::srv::Empty::Request>();
        
        // Asynchronous call in C++ requires a callback for the response
        elevator_client_->async_send_request(request, [this](rclcpp::Client<std_srvs::srv::Empty>::SharedFuture future) {
            RCLCPP_INFO(this->get_logger(), "Elevator lift successful. Attachment complete.");
            state_ = 3; // Done
        });
    }

    int state_; // 0: Detecting, 1: Approaching, 2: Lifting, 3: Done
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr elevator_client_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AttachShelfNode>());
    rclcpp::shutdown();
    return 0;
}