cmake_minimum_required(VERSION 3.8)
project(moving_point)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Nodo del punto móvil
add_executable(random_point_node
  src/random_point_node.cpp
)
ament_target_dependencies(random_point_node
  rclcpp
  visualization_msgs
  nav_msgs
  sensor_msgs
)

add_executable(static_map_node
  src/static_map_node.cpp
)
ament_target_dependencies(static_map_node
  rclcpp
  visualization_msgs
)

# Instalación de los ejecutables
install(TARGETS
  random_point_node 
  static_map_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}/config
)

# Exportar dependencias
ament_export_dependencies(
  rclcpp
  visualization_msgs
)

ament_package()