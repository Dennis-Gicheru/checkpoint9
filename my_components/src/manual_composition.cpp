#include "rclcpp/rclcpp.hpp"
#include "my_components/pre_approach_component.hpp"
#include "my_components/attach_server_component.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // 1. Create a MultiThreadedExecutor (essential for services/callbacks to run in parallel)
  rclcpp::executors::MultiThreadedExecutor executor;

  // 2. Instantiate the nodes using NodeOptions
  auto options = rclcpp::NodeOptions();
  
  auto pre_approach_node = std::make_shared<my_components::PreApproach>(options);
  auto attach_server_node = std::make_shared<my_components::AttachServer>(options);

  // 3. Add them to the same process executor
  executor.add_node(pre_approach_node);
  executor.add_node(attach_server_node);

  RCLCPP_INFO(rclcpp::get_logger("manual_composition"), "Manual Composition: PreApproach and AttachServer nodes started.");

  // 4. Spin the process
  executor.spin();

  rclcpp::shutdown();
  return 0;
}