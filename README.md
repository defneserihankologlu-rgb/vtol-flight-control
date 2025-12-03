# 🛩️ VTOL Flight Mode System — Overview

This document summarizes the current state of the **VTOL Flight Mode Architecture** and outlines the additional modes planned for the VTOL system.

---

## Current Flight Mode Message (`FlightMode.msg`)

We have implemented an extended `FlightMode` message that supports multi-stage VTOL operations, safety logic, and telemetry clarity.

### Current Fields

```msg
uint8 mode
uint8 MC=0                    # Multicopter hover mode
uint8 TRANSITION_MC_TO_FW=1   # Transition (Multicopter → Fixed-Wing)
uint8 TRANSITION_FW_TO_MC=2   # Transition (Fixed-Wing → Multicopter)
uint8 FW=3                    # Fixed-wing forward flight
uint8 LAND=4                  # Vertical or gliding landing
uint8 EMERGENCY=5             # Failsafe / emergency mode

uint8 reason
uint8 REASON_MANUAL=0         # Operator requested mode change
uint8 REASON_MISSIN=1         # Automatic change (reserved for future use)
uint8 REASON_FAILSAFE=2       # Sensor, link, or safety-triggered change
```

**Location:** `src/ucus_msgs/msg/FlightMode.msg`

This message is published continuously by the `mode_manager` node and consumed by the controller and ground station.

---

## 🧠 Current Mode Manager Capabilities

The `mode_manager` node operates as the central **state machine** for the VTOL.
It currently provides:

### ✔ Mode Command Interface

Receives mode requests through:

```
/mode_cmd  (std_msgs/UInt8)
```

### ✔ Basic Safety Conditions

* **Altitude check** for entering **FW (Fixed-Wing)** mode
  * Configurable minimum altitude via parameter `min_fw_alt` (default: 20.0 m)
  * Validates odometry altitude before allowing FW mode transition
* **Direct override behavior** for **EMERGENCY** mode
  * Always overrides other states regardless of conditions
  * Automatically sets reason to `REASON_FAILSAFE`
* **Manual transition** between MC ↔ FW
  * MC mode can be entered at any time
  * FW mode requires altitude validation
* **Reason code assignment** for telemetry logging
  * Tracks whether mode changes are manual, automatic, or failsafe-triggered

### ✔ Continuous Mode Broadcast

Publishes the current mode and reason at a fixed rate on:

```
/flight_mode  (ucus_msgs/FlightMode)
```

**Publication rate:** Configurable via parameter `pub_rate_hz` (default: 10 Hz)

### Current Implementation Details

**File:** `src/ucus_mode_manager/src/mode_manager_node.cpp`

**Subscriptions:**
- `/mode_cmd` - Mode change requests
- `/odom` - Odometry for altitude validation

**Publications:**
- `/flight_mode` - Current flight mode and reason

**Parameters:**
- `min_fw_alt` (double, default: 20.0) - Minimum altitude [m] required for FW mode
- `pub_rate_hz` (int, default: 10) - Flight mode publication rate

**Current State Machine:**
- Supports MC, FW, and EMERGENCY modes
- TRANSITION and LAND modes are defined in the message but not yet implemented in the state machine
- Initial mode: MC with REASON_MANUAL

---

## 🎮 Current Controller Implementation

The `vtol_controller` node provides basic multicopter control and adapts behavior based on the current flight mode.

### ✔ Current Capabilities

**File:** `src/ucus_kontrol/src/vtol_controller_node.cpp`

**Subscriptions:**
- `/odom` - Vehicle odometry (position, velocity, orientation)
- `/flight_mode` - Current flight mode from mode_manager
- `/armed` - Arm/disarm status
- `/setpoint/pos` - Position setpoint (PoseStamped)

**Publications:**
- `/actuator_cmd` - Actuator commands (4 motor outputs)

**Current Control Logic:**
- **MC Mode:** Simple P-controller for altitude control
  - Uses `hover_thrust` as baseline
  - Applies proportional gain `kp_z` to altitude error
  - Clamps thrust output between `thrust_min` and `thrust_max`
  - Currently applies same thrust to all 4 motors

**Safety Features:**
- **Odometry freshness check:** Disables motors if odometry is older than 200 ms
- **Arm status check:** Zeroes all outputs when disarmed
- **Output clamping:** Ensures actuator commands stay within valid range

**Parameters:**
- `kp_z` (double, default: 0.8) - Altitude proportional gain
- `hover_thrust` (double, default: 0.4) - Baseline hover thrust
- `thrust_min` (double, default: 0.0) - Minimum thrust output
- `thrust_max` (double, default: 1.0) - Maximum thrust output
- `pub_rate_hz` (int, default: 50) - Control loop rate

**Current Limitations:**
- Only implements MC mode control
- No attitude control (roll/pitch/yaw)
- No position control (x/y)
- No transition control logic
- No fixed-wing control logic
- No emergency mode specific behavior

---

## 🚀 Current Integrated Behavior

* The controller listens to `/flight_mode` and adapts high-level behavior
* The system supports manual mode switching via `/mode_cmd`
* Emergency mode always overrides other states
* FW mode transition requires altitude validation (minimum 20 m by default)
* Telemetry includes both the mode and the reason (important for GCS/UI)
* Controller currently only implements MC mode; other modes need implementation

---

# 🔜 Planned VTOL Flight Modes (Requested)

To complete a *full VTOL flight envelope*, the following modes will be added:

### 1️⃣ **Vertical Takeoff (VTOL_TAKEOFF)**

* Climb to target altitude
* Maintain stable MC thrust profile
* Transition trigger when altitude is reached
* Safety checks: IMU validity, GPS lock, battery level

### 2️⃣ **Hover Flight Mode (HOVER)**

* Maintain fixed position and altitude
* Used for station keeping, takeoff staging, or landing alignment
* Full 6-DOF position control (x, y, z, roll, pitch, yaw)
* Wind disturbance rejection

### 3️⃣ **Transition to Forward Flight (TRANSITION_MC_TO_FW)**

* Blend MC motors & control surfaces
* Gradually shift to aerodynamic lift
* Monitor airspeed, angle of attack, and attitude
* Switch to FW mode when airspeed/attitude criteria are met
* Safety abort back to MC if transition fails

### 4️⃣ **Forward Flight Mode (FW)**

* Aileron/elevator/rudder based control
* Higher efficiency cruise
* Throttle-based altitude management
* Airspeed control

### 5️⃣ **Transition Back to Hover (TRANSITION_FW_TO_MC)**

* Reduce airspeed gradually
* Engage MC motors progressively
* Maintain attitude during transition
* Achieve stable hover before landing or inspection mode
* Safety abort to emergency landing if transition fails

### 6️⃣ **Vertical Landing (VTOL_LAND)**

* Controlled descent with position hold
* Precision landing behavior
* Safety checks (IMU validity, odom freshness, thrust saturation)
* Landing gear deployment
* Final touchdown detection

---

# 🧩 Next Steps (Implementation Plan)

## 1. **Expand State Machine** (`mode_manager_node.cpp`)

Add logic branches for:

* `VTOL_TAKEOFF` - Takeoff sequence with altitude target
* `HOVER` - Position hold mode
* `TRANSITION_MC_TO_FW` - Active transition control
* `FORWARD_FLIGHT` - Fixed-wing cruise mode
* `TRANSITION_FW_TO_MC` - Back-transition control
* `VTOL_LAND` - Landing sequence

**State Machine Enhancements:**
- Add transition conditions (altitude, velocity, attitude thresholds)
- Add transition timeout handling
- Implement abort conditions for each mode

## 2. **Controller Specialization** (`vtol_controller_node.cpp`)

Create specialized handlers:

* `run_mc_controller()` - Full multicopter control (position, attitude, altitude)
* `run_fw_controller()` - Fixed-wing control (airspeed, altitude, heading)
* `run_transition_controller()` - Transition blending logic
* `run_emergency_controller()` - Failsafe behavior (hover or land)
* `run_land_controller()` - Precision landing control

**Control Enhancements:**
- Implement full 6-DOF position control for MC mode
- Add attitude control (roll, pitch, yaw)
- Implement fixed-wing control surfaces (aileron, elevator, rudder)
- Add transition blending between MC and FW control
- Implement airspeed control for FW mode

## 3. **Integrate Automatic Safety Logic** (Optional)

Add automatic switching triggers based on:

* **Altitude** - Mode transitions at specific altitudes
* **Velocity** - Airspeed thresholds for transitions
* **Attitude threshold** - Pitch/roll limits for safety
* **Battery** - Low battery triggers landing sequence
* **GPS accuracy** - Mode restrictions based on navigation quality

## 4. **Telemetry Extensions**

Provide human-readable descriptions and ground station UI integration:

* Mode name strings for display
* Transition progress indicators
* Safety status flags
* Control surface positions
* Motor RPM / thrust values
* Airspeed and altitude telemetry

## 5. **Safety Enhancements**

* Implement comprehensive pre-flight checks
* Add transition abort conditions
* Implement emergency landing procedures
* Add battery monitoring and low-power modes
* GPS-denied operation fallback
* Link loss handling

---

## 📁 Project Structure

```
ucus_kontrol_ws/
├── src/
│   ├── ucus_msgs/              # Message definitions
│   │   └── msg/
│   │       └── FlightMode.msg  # Flight mode message
│   ├── ucus_mode_manager/      # Mode manager node
│   │   └── src/
│   │       └── mode_manager_node.cpp
│   └── ucus_kontrol/           # VTOL controller node
│       └── src/
│           └── vtol_controller_node.cpp
└── README.md                   # This file
```

---

## 🔧 Building and Running

### Build the workspace:
```bash
cd /home/asus/ucus_kontrol_ws
colcon build
source install/setup.bash
```

### Run the mode manager:
```bash
ros2 run ucus_mode_manager mode_manager_node
```

### Run the controller:
```bash
ros2 run ucus_kontrol vtol_controller_node
```

---

## 📝 Notes

- The current implementation provides a solid foundation for VTOL operations
- MC mode is functional with basic altitude control
- FW mode is defined but requires controller implementation
- Transition modes are defined in the message but not yet implemented
- Emergency mode is supported at the state machine level
- The system is designed to be extensible for various flight operations

---

**Last Updated:** Based on current codebase state as of implementation review.

