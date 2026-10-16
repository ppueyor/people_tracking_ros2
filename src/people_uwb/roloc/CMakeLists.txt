cmake_minimum_required(VERSION 3.8)
project(roloc)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(range_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(people_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(roloc_node src/roloc_node.cpp)

target_include_directories(roloc_node PUBLIC ${EIGEN3_INCLUDE_DIRS})

ament_target_dependencies(roloc_node
  rclcpp
  range_msgs
  visualization_msgs
  people_msgs
)

install(
  TARGETS roloc_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
