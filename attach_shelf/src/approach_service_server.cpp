#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_srvs/srv/empty.hpp"
#include "attach_shelf/srv/go_to_loading.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <cmath>
#include <vector>

struct Point2D { double x; double y; };

class ApproachServiceServer : public rclcpp::Node {
public:
    ApproachServiceServer() : Node("approach_service_server") {
        // Multi-threading setup to allow service blocking while receiving sensor data
        cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        rclcpp::SubscriptionOptions sub_opts;
        sub_opts.callback_group = cb_group_;

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) { last_scan_ = msg; }, sub_opts);

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/diffbot_base_controller/cmd_vel_unstamped", 10);
        elevator_client_ = this->create_client<std_srvs::srv::Empty>("/elevator_up");
        
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        service_ = this->create_service<attach_shelf::srv::GoToLoading>(
            "/approach_shelf",
            std::bind(&ApproachServiceServer::handle_approach, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default, cb_group_);

        RCLCPP_INFO(this->get_logger(), "Approach Service Server Ready.");
    }

private:
    void handle_approach(const std::shared_ptr<attach_shelf::srv::GoToLoading::Request> request,
                         std::shared_ptr<attach_shelf::srv::GoToLoading::Response> response) {
        if (!last_scan_) {
            RCLCPP_ERROR(this->get_logger(), "No laser scan data available yet.");
            response->complete = false;
            return;
        }

        // 1. Detect Legs
        std::vector<Point2D> high_intensity_points;
        for (size_t i = 0; i < last_scan_->ranges.size(); ++i) {
            if (last_scan_->intensities[i] >= 8000.0) {
                double angle = last_scan_->angle_min + (i * last_scan_->angle_increment);
                high_intensity_points.push_back({last_scan_->ranges[i] * std::cos(angle), 
                                                 last_scan_->ranges[i] * std::sin(angle)});
            }
        }

        std::vector<Point2D> leg1, leg2;
        if (!high_intensity_points.empty()) leg1.push_back(high_intensity_points[0]);
        for (size_t i = 1; i < high_intensity_points.size(); ++i) {
            double dist = std::hypot(high_intensity_points[i].x - high_intensity_points[i-1].x, 
                                     high_intensity_points[i].y - high_intensity_points[i-1].y);
            if (dist > 0.1) leg2.push_back(high_intensity_points[i]);
            else if (leg2.empty()) leg1.push_back(high_intensity_points[i]);
            else leg2.push_back(high_intensity_points[i]);
        }

        if (leg1.empty() || leg2.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to detect both legs.");
            response->complete = false;
            return;
        }

        // 2. Publish TF
        auto calc_centroid = [](const std::vector<Point2D>& pts) {
            Point2D c = {0, 0};
            for (const auto& p : pts) { c.x += p.x; c.y += p.y; }
            return Point2D{c.x / pts.size(), c.y / pts.size()};
        };
        Point2D mid = {(calc_centroid(leg1).x + calc_centroid(leg2).x) / 2.0, 
                       (calc_centroid(leg1).y + calc_centroid(leg2).y) / 2.0};

        broadcast_tf(mid.x, mid.y);

        if (!request->attach_to_shelf) {
            response->complete = false;
            return;
        }

        // 3. Move towards TF (Simplified Kinematics loop)
        rclcpp::Rate rate(10);
        while (rclcpp::ok()) {
            try {
                auto t = tf_buffer_->lookupTransform("robot_base_link", "cart_frame", tf2::TimePointZero);
                double err_x = t.transform.translation.x;
                double err_y = t.transform.translation.y;
                
                if (err_x < 0.1) break; // Reached underneath

                auto twist = geometry_msgs::msg::Twist();
                twist.linear.x = std::min(0.2, 0.5 * err_x);
                twist.angular.z = 1.0 * err_y; 
                cmd_pub_->publish(twist);
            } catch (const tf2::TransformException & ex) {
                RCLCPP_WARN(this->get_logger(), "TF Error: %s", ex.what());
            }
            rate.sleep();
        }

        // 4. Move extra 30cm and Lift
        auto twist = geometry_msgs::msg::Twist();
        twist.linear.x = 0.2;
        cmd_pub_->publish(twist);
        rclcpp::sleep_for(std::chrono::milliseconds(1500)); // 0.2 m/s * 1.5s = 0.3m
        twist.linear.x = 0.0;
        cmd_pub_->publish(twist);

        auto el_req = std::make_shared<std_srvs::srv::Empty::Request>();
        elevator_client_->async_send_request(el_req);
        
        response->complete = true;
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

    rclcpp::CallbackGroup::SharedPtr cb_group_;
    sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Service<attach_shelf::srv::GoToLoading>::SharedPtr service_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr elevator_client_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    // Crucial: Multi-threaded executor required for blocking service callbacks
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<ApproachServiceServer>();
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}