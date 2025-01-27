cmake_minimum_required(VERSION 3.8)
project(udp_sender)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
add_executable(udp_sender_node src/udp_sender_node.cpp)

ament_target_dependencies(udp_sender_node rclcpp std_msgs geometry_msgs)

install(TARGETS
  udp_sender_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
