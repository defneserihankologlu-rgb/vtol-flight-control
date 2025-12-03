cmake_minimum_required(VERSION 3.8)
project(ucus_durum_tahmin)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(ekf_node src/ekf_node.cpp)
ament_target_dependencies(ekf_node 
  rclcpp 
  sensor_msgs 
  nav_msgs 
  geometry_msgs
)

install(TARGETS ekf_node DESTINATION lib/${PROJECT_NAME})

ament_package()
