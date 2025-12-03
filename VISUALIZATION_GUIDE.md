# 📊 VTOL Visualization & Monitoring Guide

This guide explains how to use the visualization and monitoring tools for the VTOL system.

---

## 🎯 Available Tools

### 1. **RViz2 3D Visualization**
Interactive 3D visualization of the VTOL system with:
- Vehicle position and orientation
- Flight path history
- Flight mode indicators
- IMU data visualization
- Safety status

### 2. **GCS Monitor (Text Interface)**
Real-time text-based ground control station showing:
- Current flight mode
- Position, velocity, and attitude
- Safety and arming status
- Actuator commands
- IMU data

### 3. **Telemetry Logger**
Records flight data for post-flight analysis (optional)

---

## 🚀 Quick Start

### Launch with Visualization

**Option 1: Launch with RViz2 (Recommended)**
```bash
cd ~/ucus_kontrol_ws
source install/setup.bash
ros2 launch ucus_bringup vtol_with_viz.launch.py use_rviz:=true
```

**Option 2: Launch with GCS Monitor**
```bash
ros2 launch ucus_bringup vtol_with_viz.launch.py use_gcs:=true
```

**Option 3: Launch with Both**
```bash
ros2 launch ucus_bringup vtol_with_viz.launch.py use_rviz:=true use_gcs:=true
```

**Option 4: Launch with Logging**
```bash
ros2 launch ucus_bringup vtol_with_viz.launch.py use_logging:=true
```

### Launch Arguments

- `use_rviz` (default: `true`) - Launch RViz2 visualization
- `use_gcs` (default: `false`) - Launch GCS monitor interface
- `use_logging` (default: `false`) - Enable telemetry logging
- `controller_params` (default: `moderate`) - Controller parameter preset

---

## 📺 Using RViz2

### Starting RViz2

RViz2 will automatically launch with the visualization configuration when you use:
```bash
ros2 launch ucus_bringup vtol_with_viz.launch.py
```

Or manually:
```bash
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix ucus_bringup)/share/ucus_bringup/config/vtol_visualization.rviz
```

### RViz2 Displays

The configuration includes:

1. **Grid** - Reference grid on the ground plane
2. **TF (Transform)** - Coordinate frame visualization
3. **Odometry** - Vehicle position and orientation (orange arrow)
4. **Flight Path** - Historical flight path (green line)
5. **Flight Mode Marker** - Text marker showing current mode above vehicle
6. **IMU** - IMU acceleration and angular velocity vectors

### View Controls

- **Orbit View**: Click and drag to rotate around the vehicle
- **Pan**: Middle mouse button or Shift+Left click
- **Zoom**: Mouse wheel or Ctrl+Left click
- **Focus**: Double-click on an object to focus on it

### Customizing RViz2

1. Open RViz2
2. Add/remove displays using the "Add" button
3. Configure display properties in the left panel
4. Save configuration: `File > Save Config As...`

---

## 🖥️ Using GCS Monitor

### Starting GCS Monitor

**Option 1: Via Launch File**
```bash
ros2 launch ucus_bringup vtol_with_viz.launch.py use_gcs:=true
```

**Option 2: Standalone**
```bash
ros2 run ucus_bringup vtol_gcs_monitor.py
```

### GCS Monitor Display

The GCS monitor shows a real-time text interface with:

```
================================================================================
                         VTOL GROUND CONTROL STATION
================================================================================

📊 FLIGHT MODE
--------------------------------------------------------------------------------
  Mode:     MC                    Reason: MANUAL

📍 POSITION & ATTITUDE
--------------------------------------------------------------------------------
  Position:  X:    0.00 m  Y:    0.00 m  Z:    0.00 m
  Velocity:  Vx:   0.00 m/s  Vy:   0.00 m/s  Vz:   0.00 m/s
  Attitude:  Roll:    0.0°  Pitch:    0.0°  Yaw:    0.0°

🛡️  SAFETY & ARMING
--------------------------------------------------------------------------------
  Status:    DISARMED
  Can Arm:   YES
  Reason:    All pre-flight checks passed
  IMU OK:    YES
  GPS OK:    YES
  Battery:   12.50 V
  Link OK:   YES

⚙️  ACTUATOR COMMANDS
--------------------------------------------------------------------------------
  Motors:    M1: 0.000  M2: 0.000  M3: 0.000  M4: 0.000

📡 IMU DATA
--------------------------------------------------------------------------------
  Accel:     X:   0.000  Y:   0.000  Z:   9.810 m/s²
  Gyro:      X:   0.000  Y:   0.000  Z:   0.000 rad/s

================================================================================
```

The display updates at 2 Hz (twice per second).

**Note**: The screen clears and redraws continuously. Press `Ctrl+C` to exit.

---

## 📝 Telemetry Logging

### Automatic Logging (Recommended)

The telemetry logger node automatically records all important topics using rosbag2:

```bash
# Launch system with logging enabled
ros2 launch ucus_bringup vtol_with_viz.launch.py use_logging:=true
```

The logger will automatically start recording when the node starts (if `auto_start:=true`).

### Manual Control

You can also control logging via services:

```bash
# Start logging manually
ros2 service call /telemetry_logger/start std_srvs/srv/SetBool "{data: true}"

# Stop logging
ros2 service call /telemetry_logger/stop std_srvs/srv/SetBool "{data: true}"
```

### Configuration

The logger can be configured via parameters:

```bash
ros2 run ucus_bringup telemetry_logger_node.py \
  --ros-args \
  -p log_directory:=~/ucus_kontrol_ws/logs \
  -p log_prefix:=vtol_flight \
  -p auto_start:=true \
  -p max_bag_size:=1073741824
```

**Parameters:**
- `log_directory` (default: `~/ucus_kontrol_ws/logs`) - Directory for bag files
- `log_prefix` (default: `vtol_flight`) - Prefix for bag filenames
- `auto_start` (default: `true`) - Automatically start logging on node startup
- `max_bag_size` (default: `1073741824` = 1GB) - Maximum bag file size
- `topics_to_log` - List of topics to record (default: all important topics)

### Logged Topics

By default, the logger records:
- `/odom` - Vehicle odometry
- `/flight_mode` - Current flight mode
- `/actuator_cmd` - Actuator commands
- `/safety/status` - Safety status
- `/safety/arming_status` - Arming status
- `/armed` - Armed/disarmed status
- `/imu/data` - IMU data
- `/setpoint/pos` - Position setpoints

### Log File Location

Logs are saved to: `~/ucus_kontrol_ws/logs/vtol_flight_YYYYMMDD_HHMMSS/`

### Playing Back Logs

```bash
# List available bags
ros2 bag info <bag_directory>

# Play back a bag
ros2 bag play <bag_directory>

# Play back with rate control
ros2 bag play <bag_directory> --rate 0.5  # Half speed
```

### Analyzing Logs

Use **PlotJuggler** for data analysis:
```bash
# Install PlotJuggler
sudo apt install ros-humble-plotjuggler-ros

# Launch PlotJuggler
ros2 run plotjuggler plotjuggler
```

Then load your bag file and visualize signals.

---

## 🔧 Manual Tool Usage

### Flight Path Publisher

Publishes the flight path as a `nav_msgs/Path` for visualization:

```bash
ros2 run ucus_bringup flight_path_publisher.py
```

**Parameters:**
- `max_path_length` (default: 1000) - Maximum poses in path
- `update_rate_hz` (default: 10.0) - Update rate
- `min_distance` (default: 0.1) - Minimum distance between path points (m)

### Flight Mode Marker Publisher

Publishes visual markers showing current flight mode:

```bash
ros2 run ucus_bringup flight_mode_marker_publisher.py
```

**Parameters:**
- `marker_scale` (default: 0.5) - Marker size
- `marker_lifetime` (default: 1.0) - Marker lifetime (seconds)

---

## 📋 Available Topics for Visualization

| Topic | Type | Description |
|-------|------|-------------|
| `/odom` | `nav_msgs/Odometry` | Vehicle position, velocity, orientation |
| `/flight_mode` | `ucus_msgs/FlightMode` | Current flight mode and reason |
| `/actuator_cmd` | `ucus_msgs/ActuatorCmd` | Actuator commands (motors, surfaces) |
| `/safety/status` | `ucus_msgs/SafetyStatus` | Comprehensive safety status |
| `/safety/arming_status` | `ucus_msgs/ArmingStatus` | Arming status with reason |
| `/armed` | `std_msgs/Bool` | Armed/disarmed status |
| `/imu/data` | `sensor_msgs/Imu` | IMU data (accel, gyro) |
| `/flight_path` | `nav_msgs/Path` | Historical flight path |
| `/flight_mode_marker` | `visualization_msgs/Marker` | Flight mode visual marker |

---

## 🎨 Customizing Visualization

### Changing RViz2 Colors

1. Open RViz2
2. Select a display (e.g., "Odometry")
3. Expand "Color" property
4. Adjust RGB values or use color picker
5. Save configuration

### Adding New Displays

1. Click "Add" button in Displays panel
2. Select display type (e.g., "Marker", "PointCloud2")
3. Configure topic and properties
4. Save configuration

### Creating Custom RViz2 Config

1. Configure RViz2 as desired
2. Save: `File > Save Config As...`
3. Save to: `src/ucus_bringup/config/my_config.rviz`
4. Update launch file to use your config

---

## 🐛 Troubleshooting

### RViz2 Not Showing Data

1. **Check topics are publishing:**
   ```bash
   ros2 topic list
   ros2 topic echo /odom
   ```

2. **Check frame_id:**
   - Ensure `/odom` uses frame_id `odom`
   - Check TF tree: `ros2 run tf2_tools view_frames`

3. **Check RViz2 fixed frame:**
   - Set Fixed Frame to `odom` in Global Options

### GCS Monitor Shows "NO DATA"

1. **Check topics are publishing:**
   ```bash
   ros2 topic list
   ros2 topic hz /odom
   ```

2. **Check node is running:**
   ```bash
   ros2 node list | grep gcs
   ```

3. **Check for errors:**
   ```bash
   ros2 node info /vtol_gcs_monitor
   ```

### Flight Path Not Appearing

1. **Check path publisher is running:**
   ```bash
   ros2 node list | grep flight_path
   ```

2. **Check path topic:**
   ```bash
   ros2 topic echo /flight_path
   ```

3. **Enable path display in RViz2:**
   - Add "Path" display
   - Set topic to `/flight_path`

---

## 📚 Additional Resources

- **RViz2 Documentation**: https://github.com/ros2/rviz
- **PlotJuggler**: https://github.com/facontidavide/PlotJuggler
- **rosbag2**: https://github.com/ros2/rosbag2

---

## 💡 Tips

1. **Performance**: Reduce RViz2 update rate if experiencing lag
2. **Recording**: Use rosbag2 for flight logs - it's more efficient than custom loggers
3. **Multiple Views**: You can run multiple RViz2 instances with different configs
4. **GCS Monitor**: Best for quick status checks without GUI overhead
5. **Logging**: Always record flights for post-analysis and debugging

---

**Last Updated**: Based on current visualization implementation

