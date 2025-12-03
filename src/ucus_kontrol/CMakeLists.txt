cmake_minimum_required(VERSION 3.8)
project(ucus_kontrol)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ucus_msgs REQUIRED)


add_executable(vtol_controller_node src/vtol_controller_node.cpp)
ament_target_dependencies(vtol_controller_node
  rclcpp nav_msgs geometry_msgs std_msgs ucus_msgs
)

install(TARGETS vtol_controller_node DESTINATION lib/${PROJECT_NAME})

# Install parameter files
install(DIRECTORY params
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
