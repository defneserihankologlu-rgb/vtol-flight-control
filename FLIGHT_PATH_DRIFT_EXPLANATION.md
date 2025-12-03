# 🔍 Why Flight Path Goes Up and Down (Z Drift)

## The Problem

Your flight path in RViz2 shows the vehicle going up and down, sometimes even going negative (below ground). This is **IMU integration drift** - a common issue with dead-reckoning.

---

## Why This Happens

### 1. **IMU Acceleration Integration**

The EKF integrates IMU acceleration twice to get position:
```
Acceleration → Velocity → Position
```

**The process:**
1. IMU measures acceleration (with noise)
2. EKF removes gravity: `accel_z = imu_z - 9.81 m/s²`
3. Integrates to velocity: `velocity += acceleration * dt`
4. Integrates to position: `position += velocity * dt`

### 2. **Noise Accumulation**

Even small noise in acceleration gets **doubly integrated**:
- Noise in acceleration → small velocity error
- Velocity error → position drift
- Over time, drift accumulates

**Example:**
- IMU noise: ±0.01 m/s²
- After 1 second: velocity error ≈ 0.01 m/s
- After 10 seconds: position drift ≈ 0.1 m
- After 100 seconds: position drift ≈ 10 m

### 3. **Gravity Compensation Issues**

The EKF tries to remove gravity (line 191):
```cpp
accel_body_z = msg->linear_acceleration.z - 9.81; // Remove gravity
```

But if:
- IMU orientation is slightly off
- Gravity vector isn't perfectly aligned
- There's noise in the measurement

Then gravity isn't fully removed, causing vertical drift.

### 4. **Barometer Fusion**

The barometer should correct this, but:
- Barometer has noise (±0.8 m by default)
- Reference pressure might be slightly off
- Fusion weight is only 30% (line 278: `baro_weight = 0.3`)

So barometer helps but doesn't completely stop drift.

---

## Why Negative Z Values?

**Z = 0 is ground level** in your coordinate system. When the EKF drifts:
- If it thinks the vehicle is accelerating down (due to noise)
- It integrates this to negative velocity
- Which integrates to negative position (below ground)

This is **normal for simulation** - the vehicle isn't actually moving, but sensor noise makes the EKF think it is.

---

## Solutions

### Quick Fix: Increase Velocity Damping

The EKF already has velocity damping (line 204-207):
```cpp
double vel_damping = 0.95;  // Reduces velocity by 5% each step
```

You can make it stronger to reduce drift:
- Change to `0.98` or `0.99` for less damping (more responsive)
- Change to `0.90` for more damping (less drift, but slower response)

### Better Fix: Improve Barometer Fusion

Increase barometer weight in EKF parameters:
```yaml
# In ekf_params.yaml
baro_weight: 0.5  # Instead of 0.3 (trust barometer more)
```

### Best Fix: Initialize at Positive Altitude

Start the vehicle at a positive altitude (e.g., 10 m) so drift doesn't go negative:
```cpp
// In ekf_node.cpp, line 44
position_z_ = 10.0;  // Start at 10 meters
```

Or set barometer reference altitude:
```yaml
# In ekf_params.yaml
reference_altitude: 10.0  # Start at 10 meters
```

---

## Is This Normal?

**Yes!** This is expected behavior for:
- ✅ Simulation with fake sensors
- ✅ IMU-only dead-reckoning
- ✅ Systems without GPS/barometer correction

**In real flight:**
- GPS will correct horizontal position
- Barometer will correct altitude
- Drift will be much smaller

---

## Current Behavior

**What you're seeing:**
- Vehicle starts at Z = 0 (ground)
- IMU noise causes small accelerations
- These integrate to velocity drift
- Velocity integrates to position drift
- Position oscillates up/down due to noise
- Sometimes goes negative (below ground)

**This is normal** - the vehicle isn't actually moving, but the EKF thinks it is due to sensor noise.

---

## How to Verify

Check the actual odometry:
```bash
ros2 topic echo /odom
```

You'll see:
- Position Z oscillating around 0
- Small velocity values (due to noise)
- Covariance increasing over time (uncertainty grows)

---

## Summary

**Why it drifts:**
1. IMU noise → acceleration errors
2. Double integration → position drift
3. Gravity compensation not perfect
4. Barometer helps but doesn't stop it completely

**Why negative Z:**
- Z=0 is ground level
- Downward drift → negative Z
- Normal for simulation

**Solutions:**
- Increase velocity damping
- Increase barometer weight
- Initialize at positive altitude
- Accept it (it's normal for simulation)

---

**Bottom line:** This is expected behavior for a simulation with noisy sensors. In real flight with GPS and barometer, drift will be much smaller.

