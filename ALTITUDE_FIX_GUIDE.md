# 🔧 Altitude Fix for Mode Switching

## The Problem

You're trying to switch to mode 3 (TRANSITION_MC_TO_FW) or mode 4 (FW), but the mode manager is rejecting it because **altitude is too low**.

**Current altitude:** -0.92m (below ground!)  
**Required altitude:** >= 20m

---

## Quick Fix

### Option 1: Use Keyboard Node (Easiest)

1. **In keyboard node terminal, press `r`** → Resets position to (0, 0, 25m)
2. **Then press `3`** → Switch to TRANSITION mode
3. **Monitor mode:** `ros2 topic echo /flight_mode`

### Option 2: Publish Odometry Manually

**Terminal 2:**
```bash
cd ~/ucus_kontrol_ws
source install/setup.bash

# Set altitude to 25m
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

**Terminal 3: Switch to mode 3**
```bash
ros2 topic pub --once /mode_cmd std_msgs/msg/UInt8 "{data: 3}"
```

---

## Step-by-Step Solution

### 1. Check Current Altitude
```bash
ros2 topic echo /odom --once | grep -A 3 "position:"
```

### 2. Set Altitude to 25m

**Using keyboard node:**
- Press `r` (reset to 25m)

**Or manually:**
```bash
ros2 topic pub --rate 10 /odom nav_msgs/msg/Odometry "..." # (with z: 25.0)
```

### 3. Verify Altitude
```bash
ros2 topic echo /odom --once | grep "z:"
```
Should show `z: 25.0` (or close to it)

### 4. Switch to Mode 3
```bash
# Using keyboard node
Press: 3

# Or manually
ros2 topic pub --once /mode_cmd std_msgs/msg/UInt8 "{data: 3}"
```

### 5. Check Mode
```bash
ros2 topic echo /flight_mode
```
Should show `mode: 3`

---

## Why This Happens

The EKF is integrating IMU data, which causes drift. The altitude can drift negative (below ground) due to:
- IMU noise
- Gravity compensation errors
- Double integration of acceleration

**Solution:** Set altitude explicitly via odometry topic, or use keyboard node's reset (`r` key).

---

## Keyboard Node Commands for Altitude

| Key | Action |
|-----|--------|
| `q` | Increase altitude by 5m |
| `e` | Decrease altitude by 5m |
| `r` | Reset to (0, 0, 25m) ← **Use this!** |

**Quick workflow:**
1. Press `r` → Reset to 25m
2. Press `3` → Switch to TRANSITION mode
3. System automatically transitions to FW when ready

---

## Troubleshooting

### Still showing mode 0 after setting altitude?

1. **Check altitude is actually set:**
   ```bash
   ros2 topic echo /odom --once | grep "z:"
   ```
   Should be >= 20.0

2. **Check mode manager is receiving odometry:**
   ```bash
   ros2 topic hz /odom
   ```
   Should show messages being published

3. **Check mode manager logs:**
   Look for warnings like:
   ```
   "FW mode requires altitude >= 20.0 m (current: X.X m)"
   ```

4. **Try mode 2 (HOVER) first:**
   ```bash
   # Press 2 in keyboard node
   # Then press 3
   ```

---

## Summary

**Problem:** Altitude is negative (-0.92m), below 20m requirement

**Solution:** 
1. Press `r` in keyboard node (resets to 25m)
2. Press `3` to switch to TRANSITION mode
3. System will automatically go to FW mode when transition completes

**Remember:** Mode 3 and 4 require altitude >= 20m!

