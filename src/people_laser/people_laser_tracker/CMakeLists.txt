cmake_minimum_required(VERSION 3.8)
project(people_laser_tracker)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(people_laser_detector REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(people_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(laser_tracker_node src/laser_tracker_node.cpp src/person_kf.hpp)

target_include_directories(laser_tracker_node PUBLIC
  ${people_laser_detector_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(laser_tracker_node
  ${people_laser_detector_LIBRARIES}
)

ament_target_dependencies(laser_tracker_node
  rclcpp
  std_msgs
  people_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(
  TARGETS laser_tracker_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch training_data config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
