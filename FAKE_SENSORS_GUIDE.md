# 📡 Fake Sensors Guide

## Overview

The `ucus_sensorleri` package now includes **4 fake sensor nodes** for testing and simulation:

1. ✅ **fake_imu_node** - 6 DOF IMU (accelerometer + gyroscope)
2. ✅ **fake_gps_node** - GPS position sensor
3. ✅ **fake_barometer_node** - Barometric pressure sensor
4. ✅ **fake_magnetometer_node** - Magnetometer (compass)

All sensors publish to the topics expected by the EKF node.

---

## Topics Published

| Node | Topic | Message Type | Description |
|------|-------|--------------|-------------|
| `fake_imu_node` | `/imu/data` | `sensor_msgs/Imu` | IMU data (accel, gyro, orientation) |
| `fake_gps_node` | `/gps/fix` | `sensor_msgs/NavSatFix` | GPS position (lat, lon, alt) |
| `fake_barometer_node` | `/barometer/pressure` | `sensor_msgs/FluidPressure` | Barometric pressure |
| `fake_magnetometer_node` | `/magnetometer/data` | `sensor_msgs/MagneticField` | Magnetic field (compass) |

---

## Usage

### Launch All Sensors

The sensors are automatically included in the main launch file:

```bash
cd ~/ucus_kontrol_ws
source install/setup.bash
ros2 launch ucus_bringup vtol_demo.launch.py
```

This will launch:
- All 4 fake sensor nodes
- EKF node (uses all sensors)
- Mode manager
- Controller
- Safety monitor

### Run Individual Sensors

You can also run sensors individually:

```bash
# Terminal 1: IMU
ros2 run ucus_sensorleri fake_imu_node

# Terminal 2: GPS
ros2 run ucus_sensorleri fake_gps_node

# Terminal 3: Barometer
ros2 run ucus_sensorleri fake_barometer_node

# Terminal 4: Magnetometer
ros2 run ucus_sensorleri fake_magnetometer_node
```

---

## Parameters

### fake_imu_node

```bash
ros2 run ucus_sensorleri fake_imu_node \
  --ros-args \
  -p publish_rate_hz:=100.0 \
  -p noise_level:=0.01
```

- `publish_rate_hz` (default: 100.0) - Publication rate
- `noise_level` (default: 0.01) - Noise level for all measurements

### fake_gps_node

```bash
ros2 run ucus_sensorleri fake_gps_node \
  --ros-args \
  -p publish_rate_hz:=10.0 \
  -p noise_level:=1.5 \
  -p altitude_noise:=3.0 \
  -p home_latitude:=41.0082 \
  -p home_longitude:=28.9784 \
  -p home_altitude:=0.0 \
  -p position_offset_x:=0.0 \
  -p position_offset_y:=0.0 \
  -p position_offset_z:=0.0
```

- `publish_rate_hz` (default: 10.0) - Publication rate
- `noise_level` (default: 1.5) - Horizontal position noise in meters
- `altitude_noise` (default: 3.0) - Altitude noise in meters
- `home_latitude` (default: 41.0082) - Home latitude (Istanbul)
- `home_longitude` (default: 28.9784) - Home longitude (Istanbul)
- `home_altitude` (default: 0.0) - Home altitude in meters
- `position_offset_x/y/z` (default: 0.0) - Position offset in meters (for testing movement)

### fake_barometer_node

```bash
ros2 run ucus_sensorleri fake_barometer_node \
  --ros-args \
  -p publish_rate_hz:=50.0 \
  -p noise_level:=100.0 \
  -p reference_pressure:=101325.0 \
  -p current_altitude:=0.0
```

- `publish_rate_hz` (default: 50.0) - Publication rate
- `noise_level` (default: 100.0) - Pressure noise in Pascals
- `reference_pressure` (default: 101325.0) - Sea level pressure in Pa
- `current_altitude` (default: 0.0) - Current altitude in meters (affects pressure)

### fake_magnetometer_node

```bash
ros2 run ucus_sensorleri fake_magnetometer_node \
  --ros-args \
  -p publish_rate_hz:=50.0 \
  -p noise_level:=0.01 \
  -p magnetic_field_strength:=0.00005 \
  -p declination:=0.1047 \
  -p inclination:=1.0472
```

- `publish_rate_hz` (default: 50.0) - Publication rate
- `noise_level` (default: 0.01) - Magnetic field noise in Tesla
- `magnetic_field_strength` (default: 0.00005) - Earth's field strength in Tesla (~50,000 nT)
- `declination` (default: 6° = 0.1047 rad) - Magnetic declination in radians
- `inclination` (default: 60° = 1.0472 rad) - Magnetic inclination in radians

---

## Testing

### Check Topics

```bash
# List all sensor topics
ros2 topic list | grep -E "(imu|gps|barometer|magnetometer)"

# Echo GPS data
ros2 topic echo /gps/fix

# Echo barometer data
ros2 topic echo /barometer/pressure

# Echo magnetometer data
ros2 topic echo /magnetometer/data

# Echo IMU data
ros2 topic echo /imu/data
```

### Check Publication Rates

```bash
# Check GPS rate
ros2 topic hz /gps/fix

# Check barometer rate
ros2 topic hz /barometer/pressure

# Check magnetometer rate
ros2 topic hz /magnetometer/data

# Check IMU rate
ros2 topic hz /imu/data
```

### Verify EKF Integration

```bash
# Check if EKF is receiving all sensors
ros2 topic echo /odom

# Check EKF parameters
ros2 param list /ekf
ros2 param get /ekf use_gps
ros2 param get /ekf use_barometer
ros2 param get /ekf use_magnetometer
```

---

## Sensor Details

### GPS Node (fake_gps_node)

- **Publishes:** GPS position with realistic noise
- **Default location:** Istanbul, Turkey (41.0082°N, 28.9784°E)
- **Features:**
  - Configurable home position
  - Position offset for testing movement
  - Realistic GPS noise (horizontal and vertical)
  - Proper covariance matrices

### Barometer Node (fake_barometer_node)

- **Publishes:** Barometric pressure
- **Features:**
  - Barometric formula for altitude calculation
  - Configurable reference pressure (sea level)
  - Altitude-based pressure calculation
  - Realistic noise

### Magnetometer Node (fake_magnetometer_node)

- **Publishes:** Earth's magnetic field
- **Features:**
  - Realistic magnetic field strength
  - Declination and inclination support
  - NED frame (North-East-Down)
  - Proper covariance matrices

### IMU Node (fake_imu_node)

- **Publishes:** IMU data (already existed)
- **Features:**
  - 6 DOF (accelerometer + gyroscope)
  - Orientation quaternion
  - Realistic noise

---

## Integration with EKF

The EKF node (`ekf_node`) automatically uses all sensors:

1. **IMU** - Primary sensor for orientation and velocity
2. **GPS** - Position correction (if enabled)
3. **Barometer** - Altitude correction (if enabled)
4. **Magnetometer** - Yaw correction (if enabled)

You can enable/disable sensors via EKF parameters:

```bash
# Disable GPS
ros2 param set /ekf use_gps false

# Disable barometer
ros2 param set /ekf use_barometer false

# Disable magnetometer
ros2 param set /ekf use_magnetometer false
```

---

## Example: Simulate Altitude Change

To test altitude changes with the barometer:

```bash
# Terminal 1: Launch system
ros2 launch ucus_bringup vtol_demo.launch.py

# Terminal 2: Change barometer altitude
ros2 param set /fake_barometer current_altitude 50.0  # 50 meters
ros2 param set /fake_barometer current_altitude 100.0  # 100 meters
ros2 param set /fake_barometer current_altitude 0.0  # Back to ground
```

---

## Example: Simulate GPS Movement

To test GPS position changes:

```bash
# Terminal 1: Launch system
ros2 launch ucus_bringup vtol_demo.launch.py

# Terminal 2: Change GPS position offset
ros2 param set /fake_gps position_offset_x 10.0  # 10 meters East
ros2 param set /fake_gps position_offset_y 5.0   # 5 meters North
ros2 param set /fake_gps position_offset_z 20.0  # 20 meters altitude
```

---

## Troubleshooting

### Sensors Not Publishing

```bash
# Check if nodes are running
ros2 node list | grep fake

# Check node info
ros2 node info /fake_gps
ros2 node info /fake_barometer
ros2 node info /fake_magnetometer
```

### EKF Not Using Sensors

```bash
# Check EKF parameters
ros2 param get /ekf use_gps
ros2 param get /ekf use_barometer
ros2 param get /ekf use_magnetometer

# Check if topics are being published
ros2 topic hz /gps/fix
ros2 topic hz /barometer/pressure
ros2 topic hz /magnetometer/data
```

### Build Errors

```bash
# Rebuild the package
cd ~/ucus_kontrol_ws
colcon build --packages-select ucus_sensorleri
source install/setup.bash
```

---

## Next Steps

1. ✅ All sensors are now available
2. ✅ EKF can use all sensors
3. ⏭️ Test with real hardware (replace fake nodes with real sensor drivers)
4. ⏭️ Tune sensor noise parameters for your hardware
5. ⏭️ Calibrate sensors (magnetometer, barometer)

---

**Last Updated:** After adding GPS, barometer, and magnetometer nodes

