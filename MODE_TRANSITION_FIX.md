# 🔧 Mode Transition Issue - Fixed!

## The Problem

You're trying to go directly from **MC (mode 0)** to **FW (mode 4)**, but the state machine doesn't allow this!

Looking at line 425-428 in `mode_manager_node.cpp`:
```cpp
case ucus_msgs::msg::FlightMode::MC:
  return (to_mode == ucus_msgs::msg::FlightMode::VTOL_TAKEOFF ||
          to_mode == ucus_msgs::msg::FlightMode::HOVER ||
          to_mode == ucus_msgs::msg::FlightMode::TRANSITION_MC_TO_FW);
```

**From MC mode, you can ONLY go to:**
- Mode 1 (VTOL_TAKEOFF)
- Mode 2 (HOVER)
- Mode 3 (TRANSITION_MC_TO_FW)

**NOT directly to Mode 4 (FW)!**

---

## Solution: Use Transition Mode

### Option 1: MC → TRANSITION_MC_TO_FW → FW (Automatic)

```bash
# Terminal 1: Launch system
cd ~/ucus_kontrol_ws
source install/setup.bash
ros2 launch ucus_bringup vtol_demo.launch.py
```

```bash
# Terminal 2: Set altitude to 25m (required)
ros2 topic pub --rate 10 /odom nav_msgs/msg/Odometry "
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

```bash
# Terminal 3: Switch to TRANSITION_MC_TO_FW (mode 3)
ros2 topic pub --once /mode_cmd std_msgs/msg/UInt8 "{data: 3}"
```

The system will **automatically** transition to FW mode when conditions are met (airspeed >= 12 m/s).

### Option 2: MC → HOVER → TRANSITION_MC_TO_FW → FW

```bash
# Step 1: Go to HOVER (mode 2)
ros2 topic pub --once /mode_cmd std_msgs/msg/UInt8 "{data: 2}"

# Step 2: Then go to TRANSITION (mode 3)
ros2 topic pub --once /mode_cmd std_msgs/msg/UInt8 "{data: 3}"

# Step 3: System automatically goes to FW when transition completes
```

---

## Valid Mode Transitions

### From MC (0):
- ✅ → VTOL_TAKEOFF (1)
- ✅ → HOVER (2)
- ✅ → TRANSITION_MC_TO_FW (3)
- ❌ → FW (4) **NOT ALLOWED DIRECTLY**

### From HOVER (2):
- ✅ → TRANSITION_MC_TO_FW (3)
- ✅ → LAND (6)
- ✅ → MC (0)

### From TRANSITION_MC_TO_FW (3):
- ✅ → FW (4) **AUTOMATIC when conditions met**
- ✅ → MC (0) **If transition fails**
- ✅ → HOVER (2)

### From FW (4):
- ✅ → TRANSITION_FW_TO_MC (5)
- ✅ → LAND (6)

---

## Why This Design?

The state machine enforces **safe transitions**:
1. You can't jump directly to FW mode
2. You must go through transition mode first
3. Transition mode validates conditions (altitude, airspeed, attitude)
4. Only completes transition when safe

---

## Quick Test

```bash
# 1. Set altitude
ros2 topic pub --rate 10 /odom nav_msgs/msg/Odometry "..." # (with z: 25.0)

# 2. Go to transition mode
ros2 topic pub --once /mode_cmd std_msgs/msg/UInt8 "{data: 3}"

# 3. Monitor mode changes
ros2 topic echo /flight_mode
```

You should see:
- Mode changes to 3 (TRANSITION_MC_TO_FW)
- Then automatically to 4 (FW) when transition completes

---

## Summary

**The issue:** You can't go directly from MC (0) to FW (4)

**The fix:** Use TRANSITION_MC_TO_FW (3) first, then it automatically goes to FW (4)

**Command:**
```bash
ros2 topic pub --once /mode_cmd std_msgs/msg/UInt8 "{data: 3}"
```

This is by design - the state machine enforces safe transitions!

