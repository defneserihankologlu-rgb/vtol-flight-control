# 🚀 Quick Start Guide

## Building the Workspace

```bash
cd ~/ucus_kontrol_ws
colcon build
source install/setup.bash
```

## Launching the System

**IMPORTANT:** Always source the workspace before launching:

```bash
cd ~/ucus_kontrol_ws
source install/setup.bash
ros2 launch ucus_bringup vtol_demo.launch.py
```

Or use a single command:
```bash
cd ~/ucus_kontrol_ws && source install/setup.bash && ros2 launch ucus_bringup vtol_demo.launch.py
```

## If You Get "Package Not Found" Error

This means the workspace isn't sourced. Solutions:

1. **Source manually:**
   ```bash
   cd ~/ucus_kontrol_ws
   source install/setup.bash
   ros2 launch ucus_bringup vtol_demo.launch.py
   ```

2. **Add to your `.bashrc` (recommended):**
   ```bash
   echo "source ~/ucus_kontrol_ws/install/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

3. **Rebuild if needed:**
   ```bash
   cd ~/ucus_kontrol_ws
   colcon build
   source install/setup.bash
   ```

## Testing the Safety Monitor

**IMPORTANT:** The safety monitor requires GPS, battery, and RC link data. In simulation, you need to publish fake data first.

### Step 1: Launch System
```bash
# Terminal 1: Launch system
cd ~/ucus_kontrol_ws
source install/setup.bash
ros2 launch ucus_bringup vtol_demo.launch.py
```

### Step 2: Publish Fake Sensor Data (Required for Testing)
```bash
# Terminal 2: Publish fake GPS, battery, and RC link data
cd ~/ucus_kontrol_ws
source install/setup.bash
python3 test_safety_sensors.py
```

### Step 3: Monitor Safety Status
```bash
# Terminal 3: Monitor safety status
cd ~/ucus_kontrol_ws
source install/setup.bash
ros2 topic echo /safety/status
```

### Step 4: Check Arming Status
```bash
# Terminal 4: Check arming status
cd ~/ucus_kontrol_ws
source install/setup.bash
ros2 topic echo /safety/arming_status
```

### Step 5: Try to Arm
```bash
# Terminal 5: Try to arm (should succeed if all checks pass)
cd ~/ucus_kontrol_ws
source install/setup.bash
ros2 topic pub --once /arm_cmd std_msgs/msg/Bool "{data: true}"
```

**What to expect:**
- Without `test_safety_sensors.py`: `can_arm: false`, reason: "Pre-flight checks failed: GPS, Sensors, RC Link"
- With `test_safety_sensors.py`: `can_arm: true`, reason: "All pre-flight checks passed"

## Available Topics

**Safety Monitor Topics:**
- `/safety/status` - Complete safety status
- `/safety/arming_status` - Arming status with reason
- `/armed` - Boolean armed status
- `/arm_cmd` - Command to arm/disarm (subscribe to arm)

**Required Input Topics (for safety checks):**
- `/gps/fix` - GPS position data (use `test_safety_sensors.py` for testing)
- `/battery/voltage` - Battery voltage (use `test_safety_sensors.py` for testing)
- `/rc/link_ok` - RC link status (use `test_safety_sensors.py` for testing)
- `/imu/data` - IMU data (published by fake_imu_node)
- `/odom` - Odometry data (published by ekf_node)

**System Topics:**
- `/flight_mode` - Current flight mode
- `/actuator_cmd` - Actuator commands

