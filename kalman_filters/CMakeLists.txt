cmake_minimum_required(VERSION 3.8)
project(kalman_filters)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(ekf_node
  src/ekf_node.cpp
  src/kalman_filter.cpp
)

target_include_directories(ekf_node PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  include
)

ament_target_dependencies(ekf_node
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
)

install(TARGETS
  ekf_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()