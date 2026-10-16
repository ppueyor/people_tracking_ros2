cmake_minimum_required(VERSION 3.8)
project(localino)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(range_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(localino_node src/localino_node.cpp src/localino.hpp)

ament_target_dependencies(localino_node
  rclcpp
  range_msgs
  visualization_msgs
)

install(
  TARGETS localino_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
