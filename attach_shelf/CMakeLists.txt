cmake_minimum_required(VERSION 3.8)
project(attach_shelf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Find all required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 2. Generate the Service Interface
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GoToLoading.srv"
)

# Extract the C++ typesupport target into a CMake variable
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# 3. Add Executables and Link Dependencies
# Pre-Approach V2
add_executable(pre_approach_v2_node src/pre_approach_v2.cpp)
ament_target_dependencies(pre_approach_v2_node rclcpp sensor_msgs geometry_msgs)
target_link_libraries(pre_approach_v2_node "${cpp_typesupport_target}")

# Approach Service Server
add_executable(approach_service_server_node src/approach_service_server.cpp)
ament_target_dependencies(approach_service_server_node rclcpp sensor_msgs geometry_msgs tf2_ros std_srvs)
target_link_libraries(approach_service_server_node "${cpp_typesupport_target}")

# 4. Install Directives
install(TARGETS
  pre_approach_v2_node
  approach_service_server_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()