#include "my_components/attach_client_component.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;

namespace my_components
{
AttachClient::AttachClient(const rclcpp::NodeOptions & options)
: Node("attach_client", options)
{
  client_ = this->create_client<attach_shelf::srv::GoToLoading>("/approach_shelf");

  // Timer fires every 500ms to check for the service without blocking the container
  timer_ = this->create_wall_timer(
    500ms, std::bind(&AttachClient::check_service_and_send_request, this));

  RCLCPP_INFO(this->get_logger(), "AttachClient Component Initialized.");
}

void AttachClient::check_service_and_send_request()
{
  // 1. Check if the server is online yet
  if (!client_->service_is_ready()) {
    RCLCPP_INFO(this->get_logger(), "Waiting for /approach_shelf service to be available...");
    return; // Try again in 500ms
  }

  // 2. Server is online! Cancel the timer so we only send ONE request
  timer_->cancel();
  RCLCPP_INFO(this->get_logger(), "Service found! Sending approach request...");

  // 3. Prepare the request
  auto request = std::make_shared<attach_shelf::srv::GoToLoading::Request>();
  request->attach_to_shelf = true; 

  // 4. Send the request asynchronously
  auto result_future = client_->async_send_request(request, 
    [this](rclcpp::Client<attach_shelf::srv::GoToLoading>::SharedFuture future) {
      auto response = future.get();
      if (response->complete) {
        RCLCPP_INFO(this->get_logger(), "SUCCESS: The robot successfully attached to the shelf.");
      } else {
        RCLCPP_ERROR(this->get_logger(), "FAILURE: The robot failed to attach to the shelf.");
      }
    });
}
} // namespace my_components

// Register the component with the ROS 2 system
RCLCPP_COMPONENTS_REGISTER_NODE(my_components::AttachClient)