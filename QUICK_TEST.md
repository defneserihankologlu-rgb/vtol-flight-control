# Quick Test Guide for Mode Manager

## Setup (Terminal 1)
```bash
cd /home/asus/ucus_kontrol_ws
source install/setup.bash
ros2 run ucus_mode_manager mode_manager_node
```

## Interactive Test (Terminal 2)
```bash
cd /home/asus/ucus_kontrol_ws
source install/setup.bash
python3 test_with_odom.py
```

This script will:
- Continuously publish odometry (25m altitude) so emergency mode doesn't trigger
- Allow you to test mode transitions interactively
- Show mode changes in real-time

## Test Sequence

1. **Start with MC mode** (default)
   - Enter: `0`
   - Should stay in MC mode

2. **Test VTOL_TAKEOFF**
   - Enter: `1`
   - Should transition to VTOL_TAKEOFF
   - If you modify the test script to increase altitude, it should auto-transition to HOVER

3. **Test HOVER**
   - Enter: `2`
   - Should transition to HOVER mode

4. **Test FW mode** (requires altitude >= 20m)
   - Enter: `4`
   - Should transition to FW (we're at 25m, so it should work)

5. **Test TRANSITION_MC_TO_FW**
   - First go to HOVER: `2`
   - Then enter: `3`
   - Should transition to TRANSITION_MC_TO_FW
   - If you modify script to increase airspeed, it should auto-transition to FW

6. **Test EMERGENCY**
   - Enter: `7`
   - Should immediately go to EMERGENCY
   - Try entering another mode - should be blocked

## Monitor Mode Changes

In Terminal 3:
```bash
cd /home/asus/ucus_kontrol_ws
source install/setup.bash
ros2 topic echo /flight_mode
```

## Check Logs

The mode manager logs will show:
- Mode transitions
- Invalid transition warnings
- Precondition failures
- Emergency triggers

