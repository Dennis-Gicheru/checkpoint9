cmake_minimum_required(VERSION 3.8)
project(my_components)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Find standard ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# 2. Find your custom service package from Task 1
find_package(attach_shelf REQUIRED)

# ==========================================
# COMPONENT 1: PreApproach
# ==========================================
add_library(pre_approach_component SHARED src/pre_approach_component.cpp)
target_include_directories(pre_approach_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(pre_approach_component rclcpp rclcpp_components sensor_msgs geometry_msgs)
rclcpp_components_register_nodes(pre_approach_component "my_components::PreApproach")

# ==========================================
# COMPONENT 2: AttachServer
# ==========================================
add_library(attach_server_component SHARED src/attach_server_component.cpp)
target_include_directories(attach_server_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(attach_server_component rclcpp rclcpp_components sensor_msgs nav_msgs geometry_msgs std_msgs tf2_ros attach_shelf)
rclcpp_components_register_nodes(attach_server_component "my_components::AttachServer")

# ==========================================
# COMPONENT 3: AttachClient
# ==========================================
add_library(attach_client_component SHARED src/attach_client_component.cpp)
target_include_directories(attach_client_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(attach_client_component rclcpp rclcpp_components attach_shelf)
rclcpp_components_register_nodes(attach_client_component "my_components::AttachClient")

# ==========================================
# EXECUTABLE: Manual Composition Wrapper
# ==========================================
add_executable(manual_composition src/manual_composition.cpp)
# We must link the executable to the libraries so it knows what classes to instantiate
target_link_libraries(manual_composition pre_approach_component attach_server_component)
ament_target_dependencies(manual_composition rclcpp rclcpp_components)

# ==========================================
# INSTALL DIRECTIVES
# ==========================================
install(TARGETS 
  pre_approach_component
  attach_server_component
  attach_client_component
  manual_composition
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_package()