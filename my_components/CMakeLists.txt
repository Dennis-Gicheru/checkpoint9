cmake_minimum_required(VERSION 3.8)
project(my_components)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Create the shared library for the component
add_library(pre_approach_component SHARED
  src/pre_approach_component.cpp
)

target_include_directories(pre_approach_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(pre_approach_component
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
)

# Register the component
rclcpp_components_register_nodes(pre_approach_component
  "my_components::Preapproach"
)

install(TARGETS pre_approach_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

ament_package()