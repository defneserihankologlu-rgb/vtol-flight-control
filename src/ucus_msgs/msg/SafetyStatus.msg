# Safety status message
std_msgs/Header header

# Pre-flight checks
bool imu_calibrated
bool gps_locked
bool gps_accuracy_ok
bool battery_ok
bool sensors_healthy
bool all_checks_passed

# Battery status
float32 battery_voltage
float32 battery_percentage
bool low_battery_warning
bool critical_battery

# GPS status
float32 gps_hdop  # Horizontal Dilution of Precision
uint8 gps_satellites
float32 gps_accuracy_m  # Position accuracy in meters

# Sensor health
bool imu_healthy
bool gps_healthy
bool barometer_healthy
bool magnetometer_healthy

# Link status
bool rc_link_ok
float32 rc_link_timeout  # Seconds since last RC message
bool telemetry_link_ok
float32 telemetry_link_timeout  # Seconds since last telemetry

# Geofence status
bool geofence_ok
float32 altitude_m
float32 distance_from_home_m  # Distance from home position

# Arming status
bool armed
bool can_arm  # Whether arming is allowed based on checks

# Overall safety status
bool safe_to_fly

