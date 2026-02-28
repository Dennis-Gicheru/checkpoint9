#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <cmath>


struct Point2D {
    double x;
    double y;
};

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
            // STEP 1: PERCEPTION & CLUSTERING
            std::vector<Point2D> high_intensity_points;
            
            // 1. Filter for reflective tape (high intensity) and convert to Cartesian
            double intensity_threshold = 8000.0; // Standard Gazebo reflective threshold
            
            for (size_t i = 0; i < msg->ranges.size(); ++i) {
                if (msg->intensities[i] >= intensity_threshold) {
                    double angle = msg->angle_min + (i * msg->angle_increment);
                    double range = msg->ranges[i];
                    
                    Point2D p;
                    p.x = range * std::cos(angle);
                    p.y = range * std::sin(angle);
                    high_intensity_points.push_back(p);
                }
            }

            // 2. We need exactly two clusters (the two legs). 
            // If we don't have enough data points yet, return and wait for the next scan.
            if (high_intensity_points.size() < 2) {
                return; 
            }

            // 3. Separate the points into two discrete clusters based on spatial distance.
            // Assuming the legs are far enough apart, a large jump in the 'y' or 'x' distance signifies the gap.
            std::vector<Point2D> leg1_points, leg2_points;
            leg1_points.push_back(high_intensity_points[0]);
            
            for (size_t i = 1; i < high_intensity_points.size(); ++i) {
                // Calculate Euclidean distance between consecutive high-intensity points
                double dx = high_intensity_points[i].x - high_intensity_points[i-1].x;
                double dy = high_intensity_points[i].y - high_intensity_points[i-1].y;
                double distance = std::sqrt(dx*dx + dy*dy);

                // If the distance between points is greater than 0.1m, it's the second leg
                if (distance > 0.1) {
                    leg2_points.push_back(high_intensity_points[i]);
                } else {
                    if (leg2_points.empty()) {
                        leg1_points.push_back(high_intensity_points[i]);
                    } else {
                        leg2_points.push_back(high_intensity_points[i]);
                    }
                }
            }

            // Ensure we successfully isolated two distinct legs
            if (leg1_points.empty() || leg2_points.empty()) {
                return;
            }

            // 4. Calculate the centroid of each leg
            auto calc_centroid = [](const std::vector<Point2D>& points) {
                Point2D centroid = {0.0, 0.0};
                for (const auto& p : points) {
                    centroid.x += p.x;
                    centroid.y += p.y;
                }
                centroid.x /= points.size();
                centroid.y /= points.size();
                return centroid;
            };

            Point2D leg1_center = calc_centroid(leg1_points);
            Point2D leg2_center = calc_centroid(leg2_points);

            // 5. Calculate the final target midpoint between the two legs
            double target_x = (leg1_center.x + leg2_center.x) / 2.0;
            double target_y = (leg1_center.y + leg2_center.y) / 2.0;
            
            // Broadcast the frame and transition to the Approach state
            broadcast_shelf_frame(target_x, target_y);
            RCLCPP_INFO(this->get_logger(), "Shelf legs detected. Midpoint published to TF.");
            state_ = 1; 
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