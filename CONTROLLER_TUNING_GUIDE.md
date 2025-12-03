# 🎛️ VTOL Controller Tuning Guide

This guide provides step-by-step instructions for tuning your VTOL flight controller parameters.

## 📋 Table of Contents

1. [Overview](#overview)
2. [Parameter Presets](#parameter-presets)
3. [Tuning Methodology](#tuning-methodology)
4. [Step-by-Step Tuning Procedure](#step-by-step-tuning-procedure)
5. [Parameter Descriptions](#parameter-descriptions)
6. [Safety Guidelines](#safety-guidelines)
7. [Troubleshooting](#troubleshooting)

---

## Overview

Controller tuning is critical for stable flight. The goal is to find gains that provide:
- **Fast response** to disturbances
- **Stable behavior** without oscillations
- **Smooth transitions** between flight modes

### Key Principles

1. **Start Conservative**: Begin with lower gains for safety
2. **Tune Incrementally**: Change one parameter at a time
3. **Test Thoroughly**: Test each change before proceeding
4. **Back Off 20%**: When oscillations appear, reduce gains by 20%
5. **Document Everything**: Keep notes on what works

---

## Parameter Presets

Three parameter presets are provided for systematic tuning:

### 1. Conservative (`controller_params_conservative.yaml`)
- **Use for**: Initial testing, first flights, safety validation
- **Characteristics**: Lower gains, slower response, very stable
- **When to use**: 
  - First time testing the system
  - After hardware changes
  - When experiencing instability

### 2. Moderate (`controller_params_moderate.yaml`) ⭐ **RECOMMENDED START**
- **Use for**: Standard operation, most vehicles
- **Characteristics**: Balanced gains, good performance
- **When to use**: 
  - Default starting point
  - After conservative preset works well
  - Most flight scenarios

### 3. Tuned (`controller_params_tuned.yaml`)
- **Use for**: Optimized performance after initial tuning
- **Characteristics**: Higher gains, faster response
- **When to use**: 
  - After completing tuning procedure
  - When you need maximum performance
  - Only after thorough testing

---

## Tuning Methodology

### The "Oscillation Method"

1. **Start with conservative gains**
2. **Gradually increase** the parameter being tuned
3. **Test** at each increment
4. **Identify oscillation point**: When oscillations appear
5. **Back off 20%**: Reduce gain by 20% from oscillation point
6. **Verify stability**: Test thoroughly at final value

### Example: Tuning `kp_z`

```
Step 1: Start at 0.5 (conservative)
Step 2: Test - stable but slow
Step 3: Increase to 0.8 - still stable
Step 4: Increase to 1.2 - stable, good response
Step 5: Increase to 1.6 - slight oscillations appear
Step 6: Back off 20%: 1.6 * 0.8 = 1.28
Step 7: Set to 1.2 (rounded) - optimal value
```

---

## Step-by-Step Tuning Procedure

### Phase 1: Hover Thrust Tuning ⚠️ **CRITICAL FIRST STEP**

**Goal**: Find the correct `hover_thrust` value for your vehicle

**Procedure**:
1. Start with `hover_thrust: 0.4`
2. Arm the vehicle (in simulation or on bench with props removed)
3. Enter HOVER mode
4. Observe altitude behavior:
   - **Descending**: Increase `hover_thrust` by 0.05
   - **Ascending**: Decrease `hover_thrust` by 0.05
   - **Stable**: You've found the right value!
5. Fine-tune in 0.01 increments
6. **Document the value** - this is vehicle-specific!

**Typical Values**:
- Light quadcopter: 0.35 - 0.45
- Medium quadcopter: 0.45 - 0.55
- Heavy quadcopter: 0.55 - 0.65

---

### Phase 2: MC Altitude Control (`kp_z`)

**Goal**: Tune vertical position control

**Procedure**:
1. Start with `kp_z: 0.5` (conservative)
2. Test altitude step response:
   - Command altitude change (e.g., 5m → 10m)
   - Observe response time and overshoot
3. Increase by 0.2 increments:
   - `0.5 → 0.7 → 0.9 → 1.1 → 1.3`
4. Watch for:
   - **Too low**: Slow response, takes long to reach setpoint
   - **Too high**: Oscillations, overshoot, instability
5. When oscillations appear, back off 20%
6. Final value should provide fast response without overshoot

**Signs of Good Tuning**:
- ✅ Reaches setpoint in 2-3 seconds
- ✅ Minimal overshoot (<10%)
- ✅ No oscillations
- ✅ Smooth response

---

### Phase 3: MC Horizontal Control (`kp_xy`)

**Goal**: Tune horizontal position control

**Procedure**:
1. Start with `kp_xy: 0.3` (conservative)
2. Test horizontal step response:
   - Command position change (e.g., x: 0m → 5m)
   - Observe response
3. Increase by 0.1-0.2 increments
4. Similar methodology to `kp_z`
5. Note: This may require attitude control implementation first

**Current Status**: Basic implementation exists, may need roll/pitch control

---

### Phase 4: MC Yaw Control (`kp_yaw`)

**Goal**: Tune heading control

**Procedure**:
1. Start with `kp_yaw: 0.2`
2. Test yaw step response:
   - Command heading change (e.g., 0° → 90°)
3. Increase gradually
4. Usually less critical than altitude
5. Watch for yaw oscillations

---

### Phase 5: FW Airspeed Control (`kp_airspeed`)

**Goal**: Tune forward flight airspeed control

**Procedure**:
1. Start with `kp_airspeed: 0.08` (conservative)
2. Enter FW mode (after transition)
3. Test airspeed response:
   - Command airspeed change
   - Observe throttle response
4. Increase by 0.02 increments
5. Watch for:
   - **Too low**: Slow to reach target airspeed
   - **Too high**: Airspeed oscillations, throttle hunting

**Typical Range**: 0.08 - 0.2

---

### Phase 6: FW Altitude Control (`kp_altitude_fw`)

**Goal**: Tune altitude control in forward flight (via elevator)

**Procedure**:
1. Start with `kp_altitude_fw: 0.03`
2. Test altitude response in FW mode
3. Increase by 0.01 increments
4. Watch for elevator oscillations
5. Balance between responsiveness and stability

**Typical Range**: 0.03 - 0.1

---

### Phase 7: Transition Parameters

**Goal**: Tune MC ↔ FW transition behavior

#### `transition_duration`
- **What**: How long transition takes (seconds)
- **Start**: 8.0 seconds (conservative)
- **Tuning**: 
  - Longer = smoother but slower
  - Shorter = faster but may be jerky
  - Typical: 5-8 seconds

#### `transition_airspeed_threshold`
- **What**: Minimum airspeed before completing MC→FW transition
- **Start**: 12.0 m/s
- **Tuning**: 
  - Too low: Transition completes before sufficient speed
  - Too high: Transition never completes
  - Adjust based on your vehicle's stall speed

---

### Phase 8: Safety Limits

**Goal**: Set appropriate safety constraints

#### `thrust_min` and `thrust_max`
- **thrust_min**: Usually 0.0 (can be higher for safety)
- **thrust_max**: Keep below 1.0 for safety margin
  - Conservative: 0.85
  - Moderate: 0.9
  - Tuned: 0.95

#### `control_surface_max`
- **What**: Maximum control surface deflection
- **Start**: 0.4 (conservative)
- **Tuning**: Increase if more authority needed
- **Typical**: 0.4 - 0.7

---

## Parameter Descriptions

### MC (Multicopter) Parameters

| Parameter | Description | Range | Default | Unit |
|-----------|-------------|-------|---------|------|
| `kp_z` | Altitude control gain | 0.3 - 2.0 | 0.8 | - |
| `kp_xy` | Horizontal position gain | 0.2 - 1.5 | 0.5 | - |
| `kp_yaw` | Yaw control gain | 0.1 - 1.0 | 0.3 | - |
| `hover_thrust` | Base hover thrust | 0.3 - 0.7 | 0.5 | normalized (0-1) |

### FW (Fixed-Wing) Parameters

| Parameter | Description | Range | Default | Unit |
|-----------|-------------|-------|---------|------|
| `kp_airspeed` | Airspeed control gain | 0.05 - 0.3 | 0.1 | - |
| `kp_altitude_fw` | FW altitude gain | 0.02 - 0.15 | 0.05 | - |
| `cruise_airspeed` | Target cruise speed | 12 - 20 | 15.0 | m/s |
| `cruise_altitude` | Target cruise altitude | 20 - 50 | 30.0 | m |

### Transition Parameters

| Parameter | Description | Range | Default | Unit |
|-----------|-------------|-------|---------|------|
| `transition_duration` | Transition time | 4 - 10 | 6.0 | seconds |
| `transition_airspeed_threshold` | Min airspeed for transition | 10 - 16 | 12.0 | m/s |
| `transition_blend_start` | Blend start point | 0.0 | 0.0 | normalized |
| `transition_blend_end` | Blend end point | 1.0 | 1.0 | normalized |

### Safety Parameters

| Parameter | Description | Range | Default | Unit |
|-----------|-------------|-------|---------|------|
| `thrust_min` | Minimum thrust | 0.0 - 0.2 | 0.0 | normalized |
| `thrust_max` | Maximum thrust | 0.8 - 1.0 | 0.9 | normalized |
| `control_surface_max` | Max control surface | 0.3 - 0.8 | 0.6 | normalized |
| `odom_timeout` | Odometry timeout | 0.3 - 1.0 | 0.5 | seconds |
| `pub_rate_hz` | Control loop rate | 50 - 100 | 50 | Hz |

---

## Safety Guidelines

### ⚠️ CRITICAL SAFETY RULES

1. **Never tune on real hardware first**
   - Always test in simulation
   - Use hardware-in-the-loop (HITL) if available

2. **Test on bench before flight**
   - Remove props for initial testing
   - Verify actuator responses
   - Check for oscillations

3. **One parameter at a time**
   - Change only one parameter per test
   - Document each change
   - Test thoroughly before next change

4. **Have emergency stop ready**
   - Always have manual override
   - Test emergency procedures
   - Know how to disarm quickly

5. **Start with conservative values**
   - Better to be too slow than unstable
   - Gradually increase gains
   - Stop if instability appears

6. **Test incrementally**
   - Small altitude changes first
   - Then larger maneuvers
   - Test all flight modes

---

## Troubleshooting

### Problem: Vehicle oscillates in hover

**Possible Causes**:
- `kp_z` too high
- `hover_thrust` incorrect
- Control loop rate too high

**Solutions**:
1. Reduce `kp_z` by 20-30%
2. Re-tune `hover_thrust`
3. Check `pub_rate_hz` (should be 50-100 Hz)

---

### Problem: Slow response to commands

**Possible Causes**:
- Gains too low
- Control loop rate too low

**Solutions**:
1. Gradually increase relevant gain
2. Increase `pub_rate_hz` (if CPU allows)

---

### Problem: Transition is jerky

**Possible Causes**:
- `transition_duration` too short
- Insufficient airspeed before transition

**Solutions**:
1. Increase `transition_duration`
2. Increase `transition_airspeed_threshold`
3. Check transition blending logic

---

### Problem: Vehicle doesn't maintain altitude in FW mode

**Possible Causes**:
- `kp_altitude_fw` too low
- `cruise_altitude` incorrect
- Elevator authority insufficient

**Solutions**:
1. Increase `kp_altitude_fw`
2. Check `control_surface_max`
3. Verify elevator is working

---

### Problem: Airspeed control unstable

**Possible Causes**:
- `kp_airspeed` too high
- Throttle response too aggressive

**Solutions**:
1. Reduce `kp_airspeed` by 20%
2. Check throttle limits

---

## Testing Checklist

Before considering tuning complete:

- [ ] Hover thrust tuned and stable
- [ ] Altitude control responsive without oscillations
- [ ] Horizontal control works (if implemented)
- [ ] Yaw control stable
- [ ] MC→FW transition smooth
- [ ] FW mode stable
- [ ] FW→MC transition smooth
- [ ] Emergency mode works
- [ ] All safety limits set appropriately
- [ ] Tested in simulation
- [ ] Tested on bench (no props)
- [ ] Ready for flight test (with safety measures)

---

## Quick Reference: Parameter File Selection

### Using Launch File

```bash
# Use moderate preset (recommended)
ros2 launch ucus_bringup vtol_demo.launch.py

# Or specify parameter file directly
ros2 run ucus_kontrol vtol_controller_node --ros-args \
  --params-file src/ucus_kontrol/params/controller_params_moderate.yaml
```

### Manual Parameter Override

```bash
# Override single parameter
ros2 param set /vtol_ctrl kp_z 1.0

# View current parameters
ros2 param list /vtol_ctrl
ros2 param get /vtol_ctrl kp_z
```

---

## Next Steps

After completing tuning:

1. **Document your final values** in a vehicle-specific config file
2. **Test all flight modes** thoroughly
3. **Create backup parameter files** for different conditions
4. **Consider advanced tuning**: PID controllers, feedforward, etc.
5. **Implement logging** to analyze performance

---

## Additional Resources

- See `NEXT_STEPS.md` for system roadmap
- See `README.md` for system overview
- Parameter files location: `src/ucus_kontrol/params/`

---

**Last Updated**: Controller tuning setup complete
**Status**: Ready for systematic tuning procedure

