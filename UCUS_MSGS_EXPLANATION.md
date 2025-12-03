# 📨 ucus_msgs Package Explanation

## Do You Need ucus_msgs? **YES - It's Essential!**

Even if you **don't use automatic commands**, you **absolutely need** `ucus_msgs` because it contains the core messages your system uses for:

1. **Flight mode management** (manual or automatic)
2. **Actuator commands** (sending motor/control surface commands)
3. **Safety monitoring** (arming, pre-flight checks)
4. **System status** (battery, GPS, sensors)

---

## Messages in ucus_msgs

### 1. **FlightMode.msg** ✅ **ESSENTIAL**

**What it does:**
- Defines current flight mode (MC, HOVER, FW, EMERGENCY, etc.)
- Includes reason code (MANUAL, MISSION, FAILSAFE)

**Used by:**
- `mode_manager_node` - Publishes current mode
- `vtol_controller_node` - Reads mode to select controller
- Ground station - Displays current mode

**Do you need it?** **YES** - Even for manual control, you need to know which mode the vehicle is in.

**About REASON_MISSION:**
- This is just one value in the `reason` field
- If you don't use automatic commands, you'll just use `REASON_MANUAL` (value 0)
- You can ignore `REASON_MISSION` - it won't hurt to have it defined

**Example usage:**
```cpp
// Controller reads flight mode
case ucus_msgs::msg::FlightMode::MC:
  // Run multicopter controller
  break;
  
case ucus_msgs::msg::FlightMode::EMERGENCY:
  // Run emergency controller
  break;
```

---

### 2. **ActuatorCmd.msg** ✅ **ESSENTIAL**

**What it does:**
- Contains array of actuator outputs (motors, control surfaces)
- Published by controller, consumed by hardware driver

**Used by:**
- `vtol_controller_node` - Publishes motor/control surface commands
- Hardware driver (future) - Reads commands and sends to actuators

**Do you need it?** **YES** - This is how your controller sends commands to motors!

**Structure:**
```msg
float32[] outputs  # Array of normalized values (0.0-1.0)
```

**Example:**
- `outputs[0]` = Motor 1 thrust
- `outputs[1]` = Motor 2 thrust
- `outputs[2]` = Motor 3 thrust
- `outputs[3]` = Motor 4 thrust
- `outputs[4]` = Aileron position
- `outputs[5]` = Elevator position
- `outputs[6]` = Rudder position

---

### 3. **SafetyStatus.msg** ✅ **ESSENTIAL**

**What it does:**
- Comprehensive safety status (battery, GPS, sensors, geofence, etc.)
- Published by `safety_monitor_node`

**Used by:**
- `safety_monitor_node` - Publishes safety status
- Ground station - Displays safety information
- Mode manager - Can trigger emergency mode

**Do you need it?** **YES** - Critical for safety monitoring!

**Contains:**
- Pre-flight checks (IMU, GPS, battery, sensors)
- Battery status (voltage, percentage, warnings)
- GPS status (HDOP, satellites, accuracy)
- Link status (RC link, telemetry)
- Geofence status
- Arming status

---

### 4. **ArmingStatus.msg** ✅ **ESSENTIAL**

**What it does:**
- Arming/disarming status with reason
- Published by `safety_monitor_node`

**Used by:**
- `safety_monitor_node` - Publishes arming status
- Ground station - Shows why arming is allowed/denied
- Controller - Checks if vehicle is armed before sending commands

**Do you need it?** **YES** - Needed for arming system!

**Contains:**
- `armed` - Whether vehicle is armed
- `can_arm` - Whether arming is allowed
- `reason` - Text explanation (e.g., "All pre-flight checks passed")

---

## What About Automatic Commands?

**Question:** "I won't use automatic commands, do I still need ucus_msgs?"

**Answer:** **YES!** Here's why:

### REASON_MISSION is Optional

The `REASON_MISSION` field in `FlightMode.msg` is just one value:
- `REASON_MANUAL=0` - You'll use this for manual commands
- `REASON_MISSION=1` - You can ignore this (it's just defined, not required)
- `REASON_FAILSAFE=2` - You'll use this for safety triggers

**Even without automatic commands, you still need:**
1. ✅ **FlightMode** - To know which mode you're in (MC, HOVER, FW, etc.)
2. ✅ **ActuatorCmd** - To send commands to motors
3. ✅ **SafetyStatus** - To monitor safety
4. ✅ **ArmingStatus** - To arm/disarm the vehicle

---

## Can You Remove REASON_MISSION?

**Technically yes, but not recommended:**

### Option 1: Keep it (Recommended)
- It's just a constant definition
- Doesn't affect functionality if unused
- Keeps message compatible with future features
- No performance impact

### Option 2: Remove it (Not Recommended)
If you really want to remove it:

1. Edit `FlightMode.msg`:
```msg
uint8 reason
uint8 REASON_MANUAL=0         # Operator requested mode change
# uint8 REASON_MISSION=1     # Removed - not using automatic commands
uint8 REASON_FAILSAFE=2       # Sensor, link, or safety-triggered change
```

2. Update code that uses `REASON_MISSION`:
   - Check `src/ucus_mode_manager/src/mode_manager_node.cpp` (4 occurrences)
   - Replace with `REASON_MANUAL` or `REASON_FAILSAFE`

**Why not recommended:**
- More work for minimal benefit
- Breaks compatibility if you add automatic features later
- The constant doesn't hurt anything

---

## Summary

### ✅ **You MUST keep ucus_msgs because:**

1. **FlightMode.msg** - Essential for mode management
2. **ActuatorCmd.msg** - Essential for sending commands to hardware
3. **SafetyStatus.msg** - Essential for safety monitoring
4. **ArmingStatus.msg** - Essential for arming system

### ❓ **About REASON_MISSION:**

- It's just one optional value
- You can ignore it if you don't use automatic commands
- You'll use `REASON_MANUAL` for manual commands
- Keeping it doesn't hurt anything

### 🎯 **Bottom Line:**

**YES, you absolutely need ucus_msgs!** Even without automatic commands, these messages are the foundation of your system. The `REASON_MISSION` field is just one value you can ignore - it doesn't make the whole package unnecessary.

---

## Current Usage in Your System

**FlightMode.msg:**
- Used by: `mode_manager_node`, `vtol_controller_node`, visualization tools
- Topic: `/flight_mode`

**ActuatorCmd.msg:**
- Used by: `vtol_controller_node` (publishes), hardware driver (will subscribe)
- Topic: `/actuator_cmd`

**SafetyStatus.msg:**
- Used by: `safety_monitor_node` (publishes), ground station (subscribes)
- Topic: `/safety/status`

**ArmingStatus.msg:**
- Used by: `safety_monitor_node` (publishes), ground station (subscribes)
- Topic: `/safety/arming_status`

---

**Conclusion:** Keep `ucus_msgs` - it's essential for your system to work, even with only manual commands!

