cmake_minimum_required(VERSION 3.8)
project(ucus_mode_manager)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ucus_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(mode_manager_node src/mode_manager_node.cpp)
ament_target_dependencies(mode_manager_node
  rclcpp
  ucus_msgs
  std_msgs
  nav_msgs
)

add_executable(safety_monitor_node src/safety_monitor_node.cpp)
ament_target_dependencies(safety_monitor_node
  rclcpp
  ucus_msgs
  std_msgs
  nav_msgs
  sensor_msgs
)

install(TARGETS
  mode_manager_node
  safety_monitor_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY params
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
