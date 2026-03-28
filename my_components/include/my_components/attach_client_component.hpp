#ifndef MY_COMPONENTS__ATTACH_CLIENT_COMPONENT_HPP_
#define MY_COMPONENTS__ATTACH_CLIENT_COMPONENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "attach_shelf/srv/go_to_loading.hpp"
#include <chrono>

namespace my_components
{
class AttachClient : public rclcpp::Node
{
public:
  explicit AttachClient(const rclcpp::NodeOptions & options);

private:
  void check_service_and_send_request();

  rclcpp::Client<attach_shelf::srv::GoToLoading>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
};
} // namespace my_components

#endif  // MY_COMPONENTS__ATTACH_CLIENT_COMPONENT_HPP_