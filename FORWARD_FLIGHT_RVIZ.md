# 🛩️ How to See Forward Flight in RViz

## Quick Start

### Terminal 1: Launch System with RViz
```bash
cd ~/ucus_kontrol_ws
source install/setup.bash
ros2 launch ucus_bringup vtol_with_viz.launch.py use_rviz:=true
```

### Terminal 2: Set Altitude (Required for FW Mode)
FW mode requires altitude >= 20m. You can simulate this by publishing odometry:

```bash
cd ~/ucus_kontrol_ws
source install/setup.bash

# Publish odometry with altitude 25m (above 20m threshold)
ros2 topic pub /odom nav_msgs/msg/Odometry "
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: 'odom'
child_frame_id: 'base_link'
pose:
  pose:
    position:
      x: 0.0
      y: 0.0
      z: 25.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
twist:
  twist:
    linear:
      x: 0.0
      y: 0.0
      z: 0.0
"
```

### Terminal 3: Switch to Forward Flight Mode
```bash
cd ~/ucus_kontrol_ws
source install/setup.bash

# Switch to FW mode (mode 4)
ros2 topic pub --once /mode_cmd std_msgs/msg/UInt8 "{data: 4}"
```

### Terminal 4: Monitor Flight Mode (Optional)
```bash
cd ~/ucus_kontrol_ws
source install/setup.bash
ros2 topic echo /flight_mode
```

---

## What You'll See in RViz

- **Vehicle model** at position (0, 0, 25m)
- **Flight mode marker** showing "FW" above vehicle
- **Orange odometry arrow** showing vehicle orientation
- **Green flight path** showing movement

---

## Make Vehicle Move Forward

To see forward flight movement, send position setpoints:

```bash
# Terminal 5: Send forward position command
cd ~/ucus_kontrol_ws
source install/setup.bash

# Move forward 10 meters
ros2 topic pub --once /setpoint/pos geometry_msgs/msg/PoseStamped "
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: 'map'
pose:
  position:
    x: 10.0
    y: 0.0
    z: 25.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
"
```

---

## Complete Workflow

**Terminal 1:**
```bash
cd ~/ucus_kontrol_ws && source install/setup.bash
ros2 launch ucus_bringup vtol_with_viz.launch.py use_rviz:=true
```

**Terminal 2:**
```bash
cd ~/ucus_kontrol_ws && source install/setup.bash

# Set altitude to 25m
ros2 topic pub /odom nav_msgs/msg/Odometry "
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: 'odom'
child_frame_id: 'base_link'
pose:
  pose:
    position:
      x: 0.0
      y: 0.0
      z: 25.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
twist:
  twist:
    linear:
      x: 0.0
      y: 0.0
      z: 0.0
"

# Switch to FW mode
ros2 topic pub --once /mode_cmd std_msgs/msg/UInt8 "{data: 4}"

# Monitor mode
ros2 topic echo /flight_mode
```

---

## Flight Mode Numbers

- `0` = MC (Multicopter)
- `1` = VTOL_TAKEOFF
- `2` = HOVER
- `3` = TRANSITION_MC_TO_FW
- `4` = FW (Forward Flight) ← **Use this**
- `5` = TRANSITION_FW_TO_MC
- `6` = LAND
- `7` = EMERGENCY

---

## Troubleshooting

**Problem: Can't switch to FW mode**
- Check altitude: `ros2 topic echo /odom` (must be >= 20m)
- Check mode: `ros2 topic echo /flight_mode`

**Problem: Vehicle not moving in RViz**
- Check if controller is running: `ros2 node list | grep vtol_ctrl`
- Check actuator commands: `ros2 topic echo /actuator_cmd`
- Send position setpoint: `ros2 topic pub /setpoint/pos ...`

**Problem: RViz not showing vehicle**
- Check if odometry is publishing: `ros2 topic hz /odom`
- Check TF frames: `ros2 run tf2_tools view_frames`

---

## Notes

- FW mode requires altitude >= 20m (default `min_fw_alt` parameter)
- Vehicle must be armed for controller to respond
- Position commands are in `/setpoint/pos` topic
- RViz shows real-time visualization of vehicle state

