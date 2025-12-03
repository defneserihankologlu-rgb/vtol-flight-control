# 🔧 Troubleshooting Guide

## Problem: `ros2 topic echo /flight_mode` doesn't work

### Possible Causes:

1. **Launch file not running**
   - The mode manager node must be running to publish `/flight_mode`
   - Solution: Start the launch file in a terminal

2. **Workspace not sourced**
   - You need to source the workspace in the terminal where you run commands
   - Solution: Run `source ~/ucus_kontrol_ws/install/setup.bash`

3. **ROS 2 domain mismatch**
   - All nodes must be in the same ROS 2 domain
   - Solution: Make sure `ROS_DOMAIN_ID` is the same (or not set) in all terminals

### Step-by-Step Fix:

**Terminal 1 - Start the system:**
```bash
cd ~/ucus_kontrol_ws
source install/setup.bash
ros2 launch ucus_bringup vtol_demo.launch.py
```

**Terminal 2 - Test topics:**
```bash
cd ~/ucus_kontrol_ws
source install/setup.bash

# Check if topics exist
ros2 topic list

# Should see: /flight_mode, /odom, /imu/data, etc.

# Echo the flight mode
ros2 topic echo /flight_mode

# Or get one message
ros2 topic echo /flight_mode --once
```

### Quick Test:

Run the test script:
```bash
cd ~/ucus_kontrol_ws
./test_system.sh
```

### Verify Nodes Are Running:

```bash
ros2 node list
```

Should see:
- `/fake_imu`
- `/ekf`
- `/mode_mgr`
- `/vtol_ctrl`

### Verify Topics Are Published:

```bash
ros2 topic list
```

Should see:
- `/flight_mode`
- `/odom`
- `/imu/data`
- `/actuator_cmd`

### Check Topic Info:

```bash
ros2 topic info /flight_mode
```

Should show:
- Type: `ucus_msgs/msg/FlightMode`
- Publisher count: 1
- Subscription count: 1 (or more)

### Common Issues:

**Issue: "Topic not found"**
- Launch file is not running
- Workspace not sourced
- Wrong terminal (different ROS domain)

**Issue: "No messages received"**
- Node crashed (check launch file output)
- Topic name mismatch
- Rate too slow (wait a bit)

**Issue: "Permission denied"**
- Need to source workspace: `source install/setup.bash`

### Debug Commands:

```bash
# Check all topics
ros2 topic list

# Check topic rate
ros2 topic hz /flight_mode

# Check topic type
ros2 topic type /flight_mode

# Check node info
ros2 node info /mode_mgr

# Check if messages are being published
ros2 topic echo /flight_mode --once
```

