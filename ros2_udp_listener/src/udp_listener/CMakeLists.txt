cmake_minimum_required(VERSION 3.8)
project(udp_listener)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(udp_listener_node src/udp_listener_node.cpp)

ament_target_dependencies(udp_listener_node rclcpp std_msgs)

install(TARGETS
  udp_listener_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
