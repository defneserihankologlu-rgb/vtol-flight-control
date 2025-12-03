cmake_minimum_required(VERSION 3.8)
project(ucus_bringup)

find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY launch params config
  DESTINATION share/${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  src/telemetry_logger_node.py
  src/vtol_gcs_monitor.py
  src/flight_path_publisher.py
  src/flight_mode_marker_publisher.py
  src/odom_tf_broadcaster.py
  src/vtol_model_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
