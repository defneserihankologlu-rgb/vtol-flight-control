# 🔧 Pre-Hardware Deployment Checklist

**CRITICAL:** Complete these items before connecting to real hardware. This checklist ensures your system is safe, tested, and ready for flight.

---

## ✅ Phase 1: Software Validation (MUST COMPLETE)

### 1.1 System Integration Testing ⚡ **HIGH PRIORITY**

**Goal:** Verify all software components work together correctly

- [ ] **Test full system launch**
  ```bash
  cd ~/ucus_kontrol_ws
  source install/setup.bash
  ros2 launch ucus_bringup vtol_demo.launch.py
  ```
  - Verify all nodes start without errors
  - Check that all topics are being published
  - Confirm no crashes or warnings

- [ ] **Verify data flow**
  - [ ] Fake IMU → EKF → Odometry (check `/odom` topic)
  - [ ] Mode Manager receives odometry and publishes flight mode
  - [ ] Controller receives flight mode and odometry
  - [ ] Controller publishes actuator commands (`/actuator_cmd`)

- [ ] **Test mode transitions**
  - [ ] MC mode works
  - [ ] Mode switching via `/mode_cmd` works
  - [ ] Emergency mode triggers correctly
  - [ ] Safety monitor integration works

- [ ] **Test safety system**
  ```bash
  # Terminal 1: Launch system
  ros2 launch ucus_bringup vtol_demo.launch.py
  
  # Terminal 2: Publish fake sensor data
  python3 test_safety_sensors.py
  
  # Terminal 3: Test arming
  ros2 topic pub --once /arm_cmd std_msgs/msg/Bool "{data: true}"
  ros2 topic echo /safety/status
  ```
  - [ ] Pre-flight checks work
  - [ ] Arming/disarming works
  - [ ] Safety limits are enforced

**How to verify:**
```bash
# Monitor all critical topics
ros2 topic echo /odom
ros2 topic echo /flight_mode
ros2 topic echo /actuator_cmd
ros2 topic echo /safety/status
ros2 topic list  # Should show all expected topics
```

---

### 1.2 Controller Parameter Tuning ⚡ **HIGH PRIORITY**

**Goal:** Set reasonable control gains for your vehicle

**Status:** ✅ Tuning framework is ready, but parameters need vehicle-specific tuning

- [ ] **Set hover thrust** (CRITICAL FIRST STEP)
  - [ ] Determine hover thrust value for your vehicle (typically 0.4-0.6)
  - [ ] Update `hover_thrust` in parameter file
  - [ ] Test in simulation if possible

- [ ] **Tune MC controller gains**
  - [ ] Start with `controller_params_moderate.yaml` (recommended)
  - [ ] Tune `kp_z`, `kp_xy`, `kp_yaw` for multicopter mode
  - [ ] Follow `CONTROLLER_TUNING_GUIDE.md` step-by-step
  - [ ] Test each gain incrementally

- [ ] **Tune FW controller gains** (if using fixed-wing mode)
  - [ ] Tune `kp_airspeed`, `kp_altitude_fw`
  - [ ] Adjust transition parameters

- [ ] **Set safety limits**
  - [ ] Configure `thrust_min` and `thrust_max` based on your hardware
  - [ ] Set `control_surface_max` for control surfaces
  - [ ] Verify limits are appropriate for your vehicle

**Parameter files:**
- `src/ucus_kontrol/params/controller_params_conservative.yaml` - Start here for first tests
- `src/ucus_kontrol/params/controller_params_moderate.yaml` - Recommended default
- `src/ucus_kontrol/params/controller_params.yaml` - Current default

**See:** `CONTROLLER_TUNING_GUIDE.md` for detailed procedure

---

### 1.3 Safety Parameter Configuration ⚡ **HIGH PRIORITY**

**Goal:** Configure safety limits for your vehicle

- [ ] **Battery parameters** (`safety_params.yaml`)
  - [ ] Set `battery_nominal_voltage` for your battery
  - [ ] Set `battery_low_voltage` threshold
  - [ ] Set `battery_min_voltage` (critical threshold)
  - [ ] Test battery monitoring

- [ ] **GPS parameters**
  - [ ] Set `gps_min_satellites` (typically 6-8)
  - [ ] Set `gps_max_hdop` (typically 2.0-3.0)
  - [ ] Set `gps_max_accuracy_m` (typically 3.0-5.0)

- [ ] **Geofence parameters**
  - [ ] Set `max_altitude_m` (maximum flight altitude)
  - [ ] Set `max_distance_from_home_m` (maximum range)
  - [ ] Test geofence enforcement

- [ ] **Link loss parameters**
  - [ ] Set `rc_link_timeout_s` (typically 0.5-1.0 seconds)
  - [ ] Test link loss detection

**File:** `src/ucus_mode_manager/params/safety_params.yaml`

---

### 1.4 Emergency Procedures ⚡ **HIGH PRIORITY**

**Goal:** Know how to stop the system in emergencies

- [ ] **Test emergency stop**
  - [ ] Verify emergency mode command works
  - [ ] Test automatic emergency triggers (odometry timeout, etc.)
  - [ ] Verify motors stop in emergency mode

- [ ] **Test disarm command**
  ```bash
  ros2 topic pub --once /arm_cmd std_msgs/msg/Bool "{data: false}"
  ```
  - [ ] Verify disarm immediately stops all outputs
  - [ ] Test disarm from any mode

- [ ] **Create emergency procedure document**
  - [ ] Write down emergency stop commands
  - [ ] Document manual override procedures
  - [ ] Plan for hardware kill switch (if available)

---

## ✅ Phase 2: Hardware Preparation (BEFORE FIRST BENCH TEST)

### 2.1 Hardware Interface Identification

**Goal:** Identify what hardware interfaces you need

- [ ] **Actuator interfaces**
  - [ ] Identify: PWM, SBUS, CAN, or other?
  - [ ] Document pin assignments
  - [ ] Verify signal voltage levels (3.3V vs 5V)

- [ ] **Sensor interfaces**
  - [ ] IMU: I2C, SPI, or serial?
  - [ ] GPS: Serial (UART) - baud rate?
  - [ ] Barometer: I2C, SPI?
  - [ ] Magnetometer: I2C, SPI?
  - [ ] Battery monitor: ADC, I2C, or other?

- [ ] **RC receiver interface**
  - [ ] SBUS, PPM, PWM, or other?
  - [ ] Document channel mapping

- [ ] **Hardware platform**
  - [ ] Identify: Raspberry Pi, Jetson, Pixhawk, or custom?
  - [ ] Document available GPIO, I2C, SPI, UART ports

---

### 2.2 Hardware Abstraction Layer

**Goal:** Create interface between ROS 2 and hardware

**Options:**

**Option A: Use ros2_control (Recommended)**
- [ ] Install `ros2_control` and `ros2_controllers`
- [ ] Create hardware interface for actuators
- [ ] Create hardware interface for sensors
- [ ] Configure control loop

**Option B: Use mavros (If using Pixhawk/PX4)**
- [ ] Install `mavros` package
- [ ] Configure MAVLink connection
- [ ] Map topics to MAVLink messages

**Option C: Custom drivers**
- [ ] Create actuator driver node
- [ ] Create sensor driver nodes
- [ ] Implement hardware-specific interfaces

**Current status:** ❌ **NOT STARTED** - This is required before hardware connection

---

### 2.3 Actuator Driver Implementation

**Goal:** Convert `/actuator_cmd` messages to hardware signals

- [ ] **Create actuator driver node**
  - [ ] Subscribe to `/actuator_cmd`
  - [ ] Convert normalized values (0.0-1.0) to PWM/SBUS/etc.
  - [ ] Implement safety checks (disarm = zero output)
  - [ ] Add output rate limiting

- [ ] **Test actuator mapping**
  - [ ] Verify motor order matches vehicle configuration
  - [ ] Test control surface mapping (if applicable)
  - [ ] Verify direction (forward/reverse) is correct

- [ ] **Implement failsafes**
  - [ ] Zero outputs on odometry timeout
  - [ ] Zero outputs when disarmed
  - [ ] Zero outputs in emergency mode

**Example structure:**
```
src/ucus_hardware/
  ├── src/
  │   └── actuator_driver_node.cpp
  ├── CMakeLists.txt
  └── package.xml
```

---

### 2.4 Sensor Driver Implementation

**Goal:** Publish sensor data to ROS 2 topics

- [ ] **IMU driver**
  - [ ] Publish to `/imu/data` (sensor_msgs/Imu)
  - [ ] Implement calibration
  - [ ] Handle sensor errors

- [ ] **GPS driver**
  - [ ] Publish to `/gps/fix` (sensor_msgs/NavSatFix)
  - [ ] Parse NMEA or UBX messages
  - [ ] Handle GPS lock status

- [ ] **Barometer driver**
  - [ ] Publish to `/barometer/pressure` (sensor_msgs/FluidPressure)
  - [ ] Convert pressure to altitude if needed

- [ ] **Magnetometer driver**
  - [ ] Publish to `/magnetometer/data` (sensor_msgs/MagneticField)
  - [ ] Implement calibration

- [ ] **Battery monitor driver**
  - [ ] Publish to `/battery/voltage` (std_msgs/Float64)
  - [ ] Implement voltage divider calibration if needed

- [ ] **RC receiver driver**
  - [ ] Publish to `/rc/link_ok` (std_msgs/Bool)
  - [ ] Parse RC channels if needed

---

## ✅ Phase 3: Bench Testing (NO PROPS!)

### 3.1 Pre-Bench Test Checklist

**Goal:** Ensure system is safe for bench testing

- [ ] **Remove all propellers** ⚠️ **CRITICAL**
- [ ] **Secure vehicle** to prevent movement
- [ ] **Verify emergency stop** procedure works
- [ ] **Test disarm command** multiple times
- [ ] **Have fire extinguisher** nearby (if using LiPo batteries)

---

### 3.2 Bench Test Procedures

**Goal:** Verify hardware integration without flight

- [ ] **Test actuator outputs**
  ```bash
  # Monitor actuator commands
  ros2 topic echo /actuator_cmd
  
  # Send test commands (start with very low values!)
  ros2 topic pub /setpoint/pos geometry_msgs/msg/PoseStamped ...
  ```
  - [ ] Verify motors respond to commands
  - [ ] Verify direction is correct
  - [ ] Check for oscillations or unexpected behavior
  - [ ] Test all motors individually

- [ ] **Test sensor data**
  - [ ] Verify IMU data is reasonable
  - [ ] Verify GPS publishes (if available indoors, may not lock)
  - [ ] Check barometer readings
  - [ ] Verify all sensors publish at expected rates

- [ ] **Test safety system**
  - [ ] Test arming/disarming
  - [ ] Test emergency mode
  - [ ] Verify motors stop when disarmed
  - [ ] Test link loss detection (disconnect RC)

- [ ] **Test mode switching**
  - [ ] Switch between modes
  - [ ] Verify controller responds correctly
  - [ ] Check for smooth transitions

- [ ] **Test odometry**
  - [ ] Verify EKF publishes odometry
  - [ ] Check that odometry values are reasonable
  - [ ] Test with vehicle movement (gently!)

---

### 3.3 Parameter Tuning on Bench

**Goal:** Fine-tune parameters with real hardware

- [ ] **Calibrate hover thrust**
  - [ ] Gradually increase thrust until vehicle lifts (with props off!)
  - [ ] Record the value
  - [ ] Set `hover_thrust` to this value

- [ ] **Test control response**
  - [ ] Send small position commands
  - [ ] Observe actuator response
  - [ ] Adjust gains if needed

- [ ] **Verify safety limits**
  - [ ] Test that `thrust_max` limits are enforced
  - [ ] Verify geofence works (if GPS available)

---

## ✅ Phase 4: Pre-Flight Validation

### 4.1 Final Pre-Flight Checklist

**Goal:** Final checks before first flight

- [ ] **All software tests passed**
- [ ] **All bench tests passed**
- [ ] **Parameters tuned** (at least roughly)
- [ ] **Safety limits configured**
- [ ] **Emergency procedures documented and tested**
- [ ] **Hardware secure** (all connections tight, props balanced)
- [ ] **Battery fully charged**
- [ ] **GPS lock acquired** (if required)
- [ ] **Pre-flight checks pass** (safety monitor)
- [ ] **Test area clear** of people and obstacles
- [ ] **Weather conditions suitable**
- [ ] **Have spotter/observer** present

---

### 4.2 First Flight Plan

**Goal:** Safe, incremental flight testing

**Recommended sequence:**

1. **Hover test (MC mode)**
   - [ ] Arm system
   - [ ] Take off to 1-2 meters
   - [ ] Hover for 10-30 seconds
   - [ ] Land immediately
   - [ ] Analyze logs

2. **Position hold test**
   - [ ] Hover and test position hold
   - [ ] Small position commands
   - [ ] Verify stability

3. **Transition test** (if applicable)
   - [ ] Test MC → FW transition (at safe altitude)
   - [ ] Test FW → MC transition
   - [ ] Verify smooth transitions

4. **Forward flight test** (if applicable)
   - [ ] Test fixed-wing mode
   - [ ] Verify stability

**After each test:**
- [ ] Review telemetry logs
- [ ] Check for anomalies
- [ ] Adjust parameters if needed
- [ ] Document results

---

## 📊 Current Status Summary

### ✅ Completed
- Flight mode system
- Controller framework
- Safety monitor with pre-flight checks
- Enhanced state estimation (EKF)
- Visualization tools
- Tuning framework

### ⚠️ Needs Work (Before Hardware)
- [ ] **System integration testing** - Verify all components work together
- [ ] **Controller parameter tuning** - Set gains for your vehicle
- [ ] **Safety parameter configuration** - Set limits for your hardware
- [ ] **Hardware abstraction layer** - Connect ROS 2 to hardware
- [ ] **Actuator drivers** - Convert commands to hardware signals
- [ ] **Sensor drivers** - Read real sensor data
- [ ] **Bench testing** - Test without props

### ❌ Not Started
- Hardware integration
- Real sensor drivers
- Real actuator drivers

---

## 🚨 Critical Warnings

1. **NEVER test with propellers attached** until you've completed bench testing
2. **ALWAYS have emergency stop** procedure ready
3. **ALWAYS test disarm** command before arming
4. **START with conservative parameters** - increase gradually
5. **TEST incrementally** - don't try everything at once
6. **LOG everything** - use rosbag2 to record all tests
7. **HAVE A SPOTTER** for first flights

---

## 📚 Reference Documents

- `NEXT_STEPS.md` - Overall roadmap
- `CONTROLLER_TUNING_GUIDE.md` - Detailed tuning procedure
- `QUICK_START.md` - How to run the system
- `TEST_GUIDE.md` - Testing procedures
- `TROUBLESHOOTING.md` - Common issues and solutions

---

## 🎯 Quick Priority List

**Do these FIRST (in order):**

1. ✅ **System Integration Testing** - Verify software works
2. ✅ **Controller Tuning** - Set basic gains
3. ✅ **Safety Configuration** - Set safety limits
4. ✅ **Hardware Abstraction** - Create hardware interface
5. ✅ **Bench Testing** - Test without props
6. ✅ **First Flight** - Start with hover test

---

**Last Updated:** Based on current codebase state
**Status:** Ready for Phase 1 (Software Validation)

