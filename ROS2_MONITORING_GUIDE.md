# 📊 ROS 2 Monitoring and Plotting Guide

Complete guide for running nodes, monitoring topics, and plotting data in ROS 2.

---

## 🚀 Running Nodes

### Method 1: Using Launch Files (Recommended)

**IMPORTANT:** Always source the workspace first!

```bash
cd ~/ucus_kontrol_ws
source install/setup.bash
```

**Launch the full VTOL system:**
```bash
ros2 launch ucus_bringup vtol_demo.launch.py
```

**Launch with visualization:**
```bash
# Launch with RViz2 only
ros2 launch ucus_bringup vtol_with_viz.launch.py use_rviz:=true

# Launch with GCS monitor only
ros2 launch ucus_bringup vtol_with_viz.launch.py use_gcs:=true

# Launch with both RViz2 and GCS monitor
ros2 launch ucus_bringup vtol_with_viz.launch.py use_rviz:=true use_gcs:=true

# Launch with telemetry logging
ros2 launch ucus_bringup vtol_with_viz.launch.py use_logging:=true

# Launch with everything
ros2 launch ucus_bringup vtol_with_viz.launch.py use_rviz:=true use_gcs:=true use_logging:=true
```

**Or use a single command:**
```bash
cd ~/ucus_kontrol_ws && source install/setup.bash && ros2 launch ucus_bringup vtol_demo.launch.py
```

### Method 2: Running Individual Nodes

**Run nodes separately:**
```bash
# Terminal 1: Mode Manager
ros2 run ucus_mode_manager mode_manager_node

# Terminal 2: Controller
ros2 run ucus_kontrol vtol_controller_node

# Terminal 3: EKF
ros2 run ucus_durum_tahmin ekf_node

# Terminal 4: Fake IMU
ros2 run ucus_sensorleri fake_imu_node

# Terminal 5: Safety Monitor
ros2 run ucus_mode_manager safety_monitor_node
```

**With custom parameters:**
```bash
ros2 run ucus_kontrol vtol_controller_node \
  --ros-args --params-file src/ucus_kontrol/params/controller_params_conservative.yaml
```

---

## 📡 Monitoring Topics

### List All Topics

```bash
# List all available topics
ros2 topic list

# List topics with their types
ros2 topic list -t

# List topics in a specific namespace
ros2 topic list | grep /safety
```

### View Topic Information

```bash
# Get topic type
ros2 topic type /flight_mode

# Get topic info (publishers, subscribers, rate)
ros2 topic info /flight_mode

# Get topic hz (publication rate)
ros2 topic hz /flight_mode

# Get topic bw (bandwidth)
ros2 topic bw /flight_mode
```

### Echo Topics (View Data)

**Basic echo:**
```bash
# Echo a topic (continuous output)
ros2 topic echo /flight_mode

# Echo once
ros2 topic echo /flight_mode --once

# Echo with specific field
ros2 topic echo /flight_mode mode

# Echo with no array data (for large arrays)
ros2 topic echo /actuator_cmd --no-arr
```

**Echo multiple topics:**
```bash
# Terminal 1
ros2 topic echo /flight_mode

# Terminal 2
ros2 topic echo /odom

# Terminal 3
ros2 topic echo /actuator_cmd
```

### Publish to Topics (Testing)

```bash
# Publish mode command
ros2 topic pub --once /mode_cmd std_msgs/msg/UInt8 "{data: 2}"

# Publish repeatedly (10 Hz)
ros2 topic pub --rate 10 /mode_cmd std_msgs/msg/UInt8 "{data: 2}"

# Publish position setpoint
ros2 topic pub --once /setpoint/pos geometry_msgs/msg/PoseStamped "
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: 'map'
pose:
  position:
    x: 5.0
    y: 3.0
    z: 10.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
"

# Publish arm command
ros2 topic pub --once /arm_cmd std_msgs/msg/Bool "{data: true}"
```

---

## 📊 Plotting Data

### Method 1: Using PlotJuggler (Recommended)

**PlotJuggler** is the best tool for plotting ROS 2 data.

#### Installation:
```bash
# Install PlotJuggler
sudo apt install ros-humble-plotjuggler-ros

# Or download from: https://github.com/facontidavide/PlotJuggler
```

#### Usage:

**1. Record data to rosbag:**
```bash
# Record all topics
ros2 bag record -a

# Record specific topics
ros2 bag record /odom /flight_mode /actuator_cmd /safety/status

# Record with custom name
ros2 bag record -o my_flight_test -a
```

**2. Play back and plot:**
```bash
# Launch PlotJuggler
ros2 run plotjuggler plotjuggler

# In PlotJuggler:
# - File → Load Data → Select your .db3 bag file
# - Drag topics/fields to plot area
# - Use time series plots, XY plots, etc.
```

**3. Live plotting (streaming):**
```bash
# Terminal 1: Launch your system
ros2 launch ucus_bringup vtol_demo.launch.py

# Terminal 2: Launch PlotJuggler with ROS 2 streaming
ros2 run plotjuggler plotjuggler --ros2
```

**In PlotJuggler:**
- Click "Streaming" tab
- Select topics to stream
- Drag fields to plot area
- Real-time plotting!

---

### Method 2: Using rqt_plot (Built-in)

**rqt_plot** is good for quick plots, but may need to be installed.

#### Installation:
```bash
# Install rqt_plot for ROS 2 Humble
sudo apt update
sudo apt install -y ros-humble-rqt-plot ros-humble-rqt-gui ros-humble-rqt-gui-py

# Or install all rqt tools
sudo apt install -y ros-humble-rqt*
```

#### Usage:
```bash
# Launch rqt_plot (correct ROS 2 way)
ros2 run rqt_plot rqt_plot

# Or specify topic directly
ros2 run rqt_plot rqt_plot /odom/pose/pose/position/z

# Plot multiple fields
ros2 run rqt_plot rqt_plot /odom/pose/pose/position/z /odom/twist/twist/linear/z
```

**Note:** In ROS 2, use `ros2 run rqt_plot rqt_plot` instead of just `rqt_plot`. Alternatively, use `rqt` GUI (see Method 3).

**Usage:**
1. Launch `rqt_plot`
2. Click "+" to add topic
3. Type topic path (e.g., `/odom/pose/pose/position/z`)
4. Click "OK"
5. Data plots in real-time

**Example plots with PlotJuggler:**
- Altitude: `/odom/pose/pose/position/z`
- Velocity: `/odom/twist/twist/linear/x`, `/odom/twist/twist/linear/y`, `/odom/twist/twist/linear/z`
- Actuator commands: `/actuator_cmd/outputs[0]` through `outputs[3]`
- Battery voltage: `/battery/voltage/data`

---

### Method 2: Using rqt (Multi-tool GUI)

**rqt** provides multiple tools in one interface.

```bash
# Launch rqt
rqt

# Or launch specific plugins
rqt --force-discover
```

**Useful rqt plugins:**
- **Topic Monitor**: View all topics and their data
- **Topic Publisher**: Publish to topics
- **Node Graph**: Visualize node/topic connections
- **Plot**: Real-time plotting (same as rqt_plot)

**Access plugins:**
- Plugins → Topics → Topic Monitor
- Plugins → Topics → Topic Publisher
- Plugins → Visualization → Plot

---

### Method 3: Using Python Scripts (Custom Plotting)

Create custom plotting scripts with matplotlib.

**Example: `plot_topics.py`**
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
import numpy as np
from nav_msgs.msg import Odometry
from collections import deque

class TopicPlotter(Node):
    def __init__(self):
        super().__init__('topic_plotter')
        
        # Data buffers
        self.time_data = deque(maxlen=1000)
        self.altitude_data = deque(maxlen=1000)
        self.start_time = None
        
        # Subscribe to topics
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        
        # Setup plot
        plt.ion()  # Interactive mode
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [])
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Altitude (m)')
        self.ax.set_title('Vehicle Altitude')
        plt.show()
        
        # Update plot periodically
        self.timer = self.create_timer(0.1, self.update_plot)
    
    def odom_callback(self, msg):
        if self.start_time is None:
            self.start_time = self.get_clock().now().nanoseconds / 1e9
        
        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed = current_time - self.start_time
        
        altitude = msg.pose.pose.position.z
        
        self.time_data.append(elapsed)
        self.altitude_data.append(altitude)
    
    def update_plot(self):
        if len(self.time_data) > 0:
            self.line.set_data(list(self.time_data), list(self.altitude_data))
            self.ax.relim()
            self.ax.autoscale_view()
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()

def main():
    rclpy.init()
    node = TopicPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Run the script:**
```bash
chmod +x plot_topics.py
python3 plot_topics.py
```

---

## 🔍 Useful Monitoring Commands

### Monitor System Health

```bash
# Check node status
ros2 node list

# Get node info
ros2 node info /vtol_controller_node

# Check service list
ros2 service list

# Check parameter list
ros2 param list

# Get parameter value
ros2 param get /vtol_controller_node hover_thrust

# Set parameter
ros2 param set /vtol_controller_node hover_thrust 0.5
```

### Monitor Topic Rates

```bash
# Check publication rate
ros2 topic hz /odom

# Check multiple topics
ros2 topic hz /odom /flight_mode /actuator_cmd

# Monitor bandwidth
ros2 topic bw /odom
```

### View Node Graph

```bash
# Launch rqt node graph (correct ROS 2 way)
ros2 run rqt_graph rqt_graph

# Or use rqt GUI
rqt
# Then: Plugins → Introspection → Node Graph
```

---

## 📝 Practical Examples

### Example 1: Monitor Full System

```bash
# Terminal 1: Launch system
cd ~/ucus_kontrol_ws
source install/setup.bash
ros2 launch ucus_bringup vtol_demo.launch.py

# Terminal 2: Monitor flight mode
ros2 topic echo /flight_mode

# Terminal 3: Monitor odometry
ros2 topic echo /odom

# Terminal 4: Monitor actuator commands
ros2 topic echo /actuator_cmd

# Terminal 5: Monitor safety status
ros2 topic echo /safety/status
```

### Example 2: Plot Altitude and Velocity

```bash
# Terminal 1: Launch system
ros2 launch ucus_bringup vtol_demo.launch.py

# Terminal 2: Plot altitude
ros2 run rqt_plot rqt_plot /odom/pose/pose/position/z

# Terminal 3: Plot velocity
ros2 run rqt_plot rqt_plot /odom/twist/twist/linear/x /odom/twist/twist/linear/y /odom/twist/twist/linear/z
```

### Example 3: Record and Analyze Flight

```bash
# Terminal 1: Launch system
ros2 launch ucus_bringup vtol_demo.launch.py

# Terminal 2: Record data
ros2 bag record -o flight_test_001 /odom /flight_mode /actuator_cmd /safety/status

# After flight, analyze with PlotJuggler
ros2 run plotjuggler plotjuggler
# Load: flight_test_001_*.db3
```

### Example 4: Monitor with rqt (All-in-One)

```bash
# Terminal 1: Launch system
ros2 launch ucus_bringup vtol_demo.launch.py

# Terminal 2: Launch rqt
rqt

# In rqt:
# - Plugins → Topics → Topic Monitor (view all topics)
# - Plugins → Visualization → Plot (plot data)
# - Plugins → Introspection → Node Graph (see connections)
```

---

## 🎯 Quick Reference

### Most Common Commands

```bash
# List topics
ros2 topic list

# Echo topic
ros2 topic echo /topic_name

# Plot topic (correct ROS 2 way)
ros2 run rqt_plot rqt_plot /topic_name/field/path

# Record bag
ros2 bag record -a

# List nodes
ros2 node list

# Node info
ros2 node info /node_name

# View node graph
ros2 run rqt_graph rqt_graph
```

### Topic Paths for VTOL System

**System Topics:**
- `/flight_mode` - Current flight mode
- `/odom` - Vehicle odometry
- `/actuator_cmd` - Actuator commands
- `/safety/status` - Safety status
- `/safety/arming_status` - Arming status
- `/armed` - Armed status (bool)

**Sensor Topics:**
- `/imu/data` - IMU data
- `/gps/fix` - GPS position
- `/barometer/pressure` - Barometer data
- `/magnetometer/data` - Magnetometer data
- `/battery/voltage` - Battery voltage

**Command Topics:**
- `/mode_cmd` - Mode command (UInt8)
- `/arm_cmd` - Arm/disarm command (Bool)
- `/setpoint/pos` - Position setpoint

---

## 🛠️ Troubleshooting

### Topic Not Found
```bash
# Check if topic exists
ros2 topic list | grep topic_name

# Check if node is running
ros2 node list

# Check topic info
ros2 topic info /topic_name
```

### No Data on Topic
```bash
# Check publication rate
ros2 topic hz /topic_name

# Check if publishers exist
ros2 topic info /topic_name
```

### Plot Not Updating
```bash
# Restart rqt_plot
ros2 run rqt_plot rqt_plot

# Or use PlotJuggler for better performance
ros2 run plotjuggler plotjuggler --ros2
```

---

## 📚 Additional Resources

- **ROS 2 Documentation**: https://docs.ros.org/en/humble/
- **PlotJuggler**: https://github.com/facontidavide/PlotJuggler
- **rqt Documentation**: https://github.com/ros-visualization/rqt

---

**Last Updated:** Based on ROS 2 Humble

