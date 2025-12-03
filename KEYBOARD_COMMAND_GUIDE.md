# ⌨️ Keyboard Command Node Guide

A ROS2 node that reads keyboard input and publishes commands as if from a joystick/GCS. Perfect for testing flight mode switching and system behavior without hardware.

---

## 🚀 Quick Start

### Terminal 1: Launch System
```bash
cd ~/ucus_kontrol_ws
source install/setup.bash
ros2 launch ucus_bringup vtol_demo.launch.py
```

### Terminal 2: Run Keyboard Command Node
```bash
cd ~/ucus_kontrol_ws
source install/setup.bash
ros2 run ucus_bringup keyboard_command_node.py
```

**Important:** Make sure Terminal 2 has focus (click on it) for keyboard input to work!

---

## ⌨️ Keyboard Commands

### Flight Mode Commands

| Key | Mode | Description |
|-----|------|-------------|
| `0` | MC | Multicopter mode |
| `1` | VTOL_TAKEOFF | Vertical takeoff |
| `2` | HOVER | Hover/position hold |
| `3` | TRANSITION_MC_TO_FW | Transition to forward flight |
| `4` | FW | Forward flight (use mode 3 first!) |
| `5` | TRANSITION_FW_TO_MC | Transition back to multicopter |
| `6` | LAND | Landing mode |
| `7` | EMERGENCY | Emergency/failsafe mode |

### Position Setpoint Commands

| Key | Action | Description |
|-----|--------|-------------|
| `w` | Forward | Move forward (+X, 5m) |
| `s` | Backward | Move backward (-X, 5m) |
| `a` | Left | Move left (+Y, 5m) |
| `d` | Right | Move right (-Y, 5m) |
| `q` | Up | Increase altitude (+Z, 5m) |
| `e` | Down | Decrease altitude (-Z, 5m) |
| `r` | Reset | Reset to (0, 0, 25m) |

### Arming Commands

| Key | Action |
|-----|-------|
| `space` | Toggle arm/disarm |

### Other Commands

| Key | Action |
|-----|-------|
| `h` | Show help |
| `x` | Exit node |

---

## 📋 Example Usage

### Test Flight Mode Switching

1. **Launch system:**
   ```bash
   ros2 launch ucus_bringup vtol_demo.launch.py
   ```

2. **Start keyboard node:**
   ```bash
   ros2 run ucus_bringup keyboard_command_node.py
   ```

3. **Switch modes:**
   - Press `2` → Switch to HOVER mode
   - Press `3` → Switch to TRANSITION_MC_TO_FW
   - System automatically transitions to FW when conditions met
   - Press `5` → Transition back to MC
   - Press `6` → Landing mode
   - Press `7` → Emergency mode

4. **Monitor mode changes:**
   ```bash
   # In another terminal
   ros2 topic echo /flight_mode
   ```

### Test Position Control

1. **Set initial position:**
   - Press `r` → Reset to (0, 0, 25m)

2. **Move vehicle:**
   - Press `w` → Move forward 5m
   - Press `a` → Move left 5m
   - Press `q` → Increase altitude 5m
   - Press `d` → Move right 5m
   - Press `s` → Move backward 5m
   - Press `e` → Decrease altitude 5m

3. **Watch in RViz:**
   ```bash
   ros2 launch ucus_bringup vtol_with_viz.launch.py use_rviz:=true
   ```

---

## 🎯 Testing Scenarios

### Scenario 1: Basic Mode Switching

```bash
# Terminal 1: Launch system
ros2 launch ucus_bringup vtol_demo.launch.py

# Terminal 2: Keyboard commands
ros2 run ucus_bringup keyboard_command_node.py
# Press: 0 → 2 → 3 → (wait for auto transition) → 5 → 0

# Terminal 3: Monitor
ros2 topic echo /flight_mode
```

### Scenario 2: Position Control Test

```bash
# Terminal 1: Launch with RViz
ros2 launch ucus_bringup vtol_with_viz.launch.py use_rviz:=true

# Terminal 2: Keyboard commands
ros2 run ucus_bringup keyboard_command_node.py
# Press: r (reset) → w → w → a → q → q
# Watch vehicle move in RViz
```

### Scenario 3: Emergency Mode Test

```bash
# Terminal 1: Launch system
ros2 launch ucus_bringup vtol_demo.launch.py

# Terminal 2: Keyboard commands
ros2 run ucus_bringup keyboard_command_node.py
# Press: 2 (hover) → 7 (emergency)
# System should enter emergency mode immediately
```

---

## 🔧 How It Works

The keyboard node:
1. **Reads keyboard input** in real-time (non-blocking)
2. **Publishes commands** to ROS2 topics:
   - `/mode_cmd` - Flight mode commands
   - `/arm_cmd` - Arming commands
   - `/setpoint/pos` - Position setpoints
3. **Provides feedback** - Shows what command was sent

### Topics Published

- `/mode_cmd` (std_msgs/UInt8) - Flight mode commands
- `/arm_cmd` (std_msgs/Bool) - Arm/disarm commands
- `/setpoint/pos` (geometry_msgs/PoseStamped) - Position setpoints

---

## ⚠️ Important Notes

1. **Terminal Focus:** The keyboard node must have terminal focus to receive input
2. **Mode Transitions:** You can't go directly from MC (0) to FW (4). Use TRANSITION (3) first.
3. **Altitude Requirement:** FW mode requires altitude >= 20m. Use `q` to increase altitude first.
4. **Position Setpoints:** Position commands are relative. Press `r` to reset to origin.

---

## 🐛 Troubleshooting

### Keyboard input not working
- Make sure terminal has focus (click on it)
- Check if node is running: `ros2 node list | grep keyboard`
- Try typing in the terminal to verify it's active

### Commands not being received
- Check if system is running: `ros2 node list`
- Verify topics exist: `ros2 topic list | grep -E "(mode_cmd|arm_cmd|setpoint)"`
- Check topic info: `ros2 topic info /mode_cmd`

### Mode not changing
- Check altitude: FW mode requires >= 20m
- Check valid transitions: Can't go directly MC → FW
- Monitor mode manager logs for warnings

---

## 📝 Example Session

```
$ ros2 run ucus_bringup keyboard_command_node.py
======================================================================
Keyboard Command Node Started
======================================================================

FLIGHT MODE COMMANDS:
  0  - MC (Multicopter)
  1  - VTOL_TAKEOFF
  2  - HOVER
  3  - TRANSITION_MC_TO_FW
  4  - FW (Forward Flight) - Note: Use mode 3 first!
  5  - TRANSITION_FW_TO_MC
  6  - LAND
  7  - EMERGENCY

[... help text ...]

======================================================================

[INFO] [keyboard_command_node]: 📡 Sent mode command: 2 (HOVER)
[INFO] [keyboard_command_node]: 📡 Sent mode command: 3 (TRANSITION_MC_TO_FW)
[INFO] [keyboard_command_node]: 📍 Position setpoint: X=5.0m, Y=0.0m, Z=25.0m
[INFO] [keyboard_command_node]: 📡 Sent mode command: 5 (TRANSITION_FW_TO_MC)
```

---

## 🎓 Tips

1. **Start with simple modes:** Test MC (0) and HOVER (2) first
2. **Use RViz:** Launch with visualization to see vehicle response
3. **Monitor topics:** Use `ros2 topic echo` to see what's happening
4. **Check logs:** Mode manager logs show why transitions fail
5. **Reset position:** Press `r` to reset if position gets confusing

---

**Enjoy testing your VTOL system!** 🚁✈️

