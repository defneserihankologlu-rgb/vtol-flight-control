cmake_minimum_required(VERSION 3.8)
project(ucus_sensorleri)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# IMU node
add_executable(fake_imu_node src/fake_imu_node.cpp)
ament_target_dependencies(fake_imu_node rclcpp sensor_msgs)

# GPS node
add_executable(fake_gps_node src/fake_gps_node.cpp)
ament_target_dependencies(fake_gps_node rclcpp sensor_msgs)

# Barometer node
add_executable(fake_barometer_node src/fake_barometer_node.cpp)
ament_target_dependencies(fake_barometer_node rclcpp sensor_msgs)

# Magnetometer node
add_executable(fake_magnetometer_node src/fake_magnetometer_node.cpp)
ament_target_dependencies(fake_magnetometer_node rclcpp sensor_msgs)

install(TARGETS
  fake_imu_node
  fake_gps_node
  fake_barometer_node
  fake_magnetometer_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
