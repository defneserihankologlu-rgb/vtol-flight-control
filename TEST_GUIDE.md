# Mode Manager Testing Guide

## Quick Start

### Terminal 1: Start Mode Manager
```bash
cd /home/asus/ucus_kontrol_ws
source install/setup.bash
ros2 run ucus_mode_manager mode_manager_node
```

### Terminal 2: Run Test Script
```bash
cd /home/asus/ucus_kontrol_ws
source install/setup.bash
python3 test_mode_manager.py
```

## Manual Testing with ROS 2 CLI

### Monitor Flight Mode
```bash
ros2 topic echo /flight_mode
```

### Send Mode Commands
```bash
# MC mode
ros2 topic pub --once /mode_cmd std_msgs/msg/UInt8 "{data: 0}"

# VTOL_TAKEOFF
ros2 topic pub --once /mode_cmd std_msgs/msg/UInt8 "{data: 1}"

# HOVER
ros2 topic pub --once /mode_cmd std_msgs/msg/UInt8 "{data: 2}"

# TRANSITION_MC_TO_FW
ros2 topic pub --once /mode_cmd std_msgs/msg/UInt8 "{data: 3}"

# FW mode
ros2 topic pub --once /mode_cmd std_msgs/msg/UInt8 "{data: 4}"

# TRANSITION_FW_TO_MC
ros2 topic pub --once /mode_cmd std_msgs/msg/UInt8 "{data: 5}"

# LAND
ros2 topic pub --once /mode_cmd std_msgs/msg/UInt8 "{data: 6}"

# EMERGENCY
ros2 topic pub --once /mode_cmd std_msgs/msg/UInt8 "{data: 7}"
```

### Publish Simulated Odometry
```bash
# Example: 25m altitude (above min_fw_alt threshold)
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

## Test Scenarios

### 1. Basic Mode Switching
1. Start mode manager
2. Monitor `/flight_mode` - should show MC mode initially
3. Try switching to HOVER (mode 2)
4. Try switching to FW (mode 4) - should fail if altitude < 20m
5. Publish odometry with altitude > 20m
6. Try switching to FW again - should succeed

### 2. Takeoff Sequence
1. Start at MC mode
2. Switch to VTOL_TAKEOFF (mode 1)
3. Publish odometry with increasing altitude
4. Watch for automatic transition to HOVER when altitude reaches ~30m

### 3. Transition MC→FW
1. Start in HOVER mode at altitude > 20m
2. Switch to TRANSITION_MC_TO_FW (mode 3)
3. Publish odometry with increasing airspeed (>12 m/s) and stable attitude
4. Watch for automatic transition to FW mode

### 4. Emergency Mode
1. In any mode, switch to EMERGENCY (mode 7)
2. Try switching to another mode - should be blocked
3. Emergency should override all other modes

### 5. Odometry Timeout
1. Start mode manager
2. Stop publishing odometry
3. After 0.5 seconds, should automatically enter EMERGENCY mode

## Expected Behaviors

- **MC → VTOL_TAKEOFF**: Always allowed
- **VTOL_TAKEOFF → HOVER**: Automatic when altitude >= 29m
- **HOVER → TRANSITION_MC_TO_FW**: Requires altitude >= 20m
- **TRANSITION_MC_TO_FW → FW**: Automatic when airspeed >= 12 m/s and attitude stable
- **FW → TRANSITION_FW_TO_MC**: Always allowed
- **TRANSITION_FW_TO_MC → HOVER**: Automatic when airspeed < 2 m/s
- **Any mode → EMERGENCY**: Always allowed, overrides everything
- **EMERGENCY → Any mode**: Blocked (must exit emergency manually)

