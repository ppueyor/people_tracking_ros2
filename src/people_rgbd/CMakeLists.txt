cmake_minimum_required(VERSION 3.8)
project(people_rgbd)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(people_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(people_rgbd_node src/people_rgbd_node.cpp)

target_include_directories(people_rgbd_node PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(people_rgbd_node ${OpenCV_LIBRARIES})

ament_target_dependencies(people_rgbd_node
  rclcpp
  cv_bridge
  image_transport
  people_msgs
  sensor_msgs
  visualization_msgs
  message_filters
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
)

install(
  TARGETS people_rgbd_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
