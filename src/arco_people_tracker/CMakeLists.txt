cmake_minimum_required(VERSION 3.8)
project(arco_people_tracker)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(people_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(people_tracker_node src/people_tracker_node.cpp)

target_include_directories(people_tracker_node PUBLIC ${EIGEN3_INCLUDE_DIRS})

ament_target_dependencies(people_tracker_node
  rclcpp
  people_msgs
  visualization_msgs
)

install(
  TARGETS people_tracker_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
