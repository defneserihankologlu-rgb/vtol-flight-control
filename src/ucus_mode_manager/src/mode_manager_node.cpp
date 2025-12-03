#include "rclcpp/rclcpp.hpp"
#include "ucus_msgs/msg/flight_mode.hpp"
#include "ucus_msgs/msg/safety_status.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <cmath>

class ModeManager : public rclcpp::Node
{
public:
  ModeManager() : Node("mode_manager")
  {
    // ========== Parameters ==========
    // Altitude thresholds
    min_fw_alt_ = this->declare_parameter<double>("min_fw_alt", 20.0);           // [m]
    takeoff_target_alt_ = this->declare_parameter<double>("takeoff_target_alt", 30.0);  // [m]
    landing_alt_threshold_ = this->declare_parameter<double>("landing_alt_threshold", 2.0); // [m]
    
    // Velocity thresholds
    min_transition_airspeed_ = this->declare_parameter<double>("min_transition_airspeed", 12.0); // [m/s]
    max_transition_airspeed_ = this->declare_parameter<double>("max_transition_airspeed", 25.0); // [m/s]
    hover_max_velocity_ = this->declare_parameter<double>("hover_max_velocity", 2.0); // [m/s]
    
    // Attitude thresholds
    max_transition_pitch_ = this->declare_parameter<double>("max_transition_pitch", 0.35); // [rad] ~20 deg
    max_transition_roll_ = this->declare_parameter<double>("max_transition_roll", 0.35);   // [rad] ~20 deg
    
    // Timing
    transition_timeout_ = this->declare_parameter<double>("transition_timeout", 30.0); // [s]
    takeoff_timeout_ = this->declare_parameter<double>("takeoff_timeout", 60.0);        // [s]
    landing_timeout_ = this->declare_parameter<double>("landing_timeout", 120.0);       // [s]
    
    // Safety
    odom_timeout_ = this->declare_parameter<double>("odom_timeout", 1.0); // [s] - increased from 0.5
    enable_auto_transitions_ = this->declare_parameter<bool>("enable_auto_transitions", true);
    
    pub_rate_hz_ = this->declare_parameter<int>("pub_rate_hz", 10);
    state_machine_rate_hz_ = this->declare_parameter<int>("state_machine_rate_hz", 20);

    // ========== Initial State ==========
    current_mode_ = ucus_msgs::msg::FlightMode::MC;
    current_reason_ = ucus_msgs::msg::FlightMode::REASON_MANUAL;
    transition_start_time_ = this->now();
    takeoff_start_time_ = this->now();
    startup_time_ = this->now();

    // ========== Subscriptions ==========
    mode_cmd_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
      "/mode_cmd", 10,
      std::bind(&ModeManager::modeCmdCb, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&ModeManager::odomCb, this, std::placeholders::_1));

    safety_status_sub_ = this->create_subscription<ucus_msgs::msg::SafetyStatus>(
      "/safety/status", 10,
      std::bind(&ModeManager::safetyStatusCb, this, std::placeholders::_1));

    // ========== Publishers ==========
    mode_pub_ = this->create_publisher<ucus_msgs::msg::FlightMode>("/flight_mode", 10);

    // ========== Timers ==========
    // Mode publication timer
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000 / pub_rate_hz_),
      std::bind(&ModeManager::publishMode, this));
    
    // State machine update timer
    state_machine_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000 / state_machine_rate_hz_),
      std::bind(&ModeManager::updateStateMachine, this));
  }

private:
  // ========== Callbacks ==========
  void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    last_odom_ = *msg;
    have_odom_ = true;
    last_odom_stamp_ = this->now();
    
    // Extract velocity
    velocity_x_ = msg->twist.twist.linear.x;
    velocity_y_ = msg->twist.twist.linear.y;
    velocity_z_ = msg->twist.twist.linear.z;
    airspeed_ = std::sqrt(velocity_x_ * velocity_x_ + velocity_y_ * velocity_y_);
    
    // Extract attitude
    extractAttitude(msg->pose.pose.orientation);
    
    // Debug: log odometry reception (throttled)
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "Odometry received: alt=%.1fm, airspeed=%.1fm/s", 
      msg->pose.pose.position.z, airspeed_);
  }

  void modeCmdCb(const std_msgs::msg::UInt8::SharedPtr msg)
  {
    uint8_t requested = msg->data;
    handleModeRequest(requested, ucus_msgs::msg::FlightMode::REASON_MANUAL);
  }

  void safetyStatusCb(const ucus_msgs::msg::SafetyStatus::SharedPtr msg)
  {
    last_safety_status_ = *msg;
    have_safety_status_ = true;
    
    // Check for critical safety issues that should trigger emergency
    if (!msg->safe_to_fly && current_mode_ != ucus_msgs::msg::FlightMode::EMERGENCY) {
      // Critical battery or other unsafe conditions
      if (msg->critical_battery || !msg->sensors_healthy) {
        RCLCPP_ERROR(this->get_logger(), 
          "Critical safety issue detected! Entering emergency mode.");
        changeMode(ucus_msgs::msg::FlightMode::EMERGENCY, 
                   ucus_msgs::msg::FlightMode::REASON_FAILSAFE);
      }
    }
  }

  // ========== State Machine Logic ==========
  void updateStateMachine()
  {
    // Check for emergency conditions
    bool emergency_condition = checkEmergencyConditions();
    
    // If emergency condition detected and not already in emergency, enter emergency
    if (emergency_condition) {
      if (current_mode_ != ucus_msgs::msg::FlightMode::EMERGENCY) {
        changeMode(ucus_msgs::msg::FlightMode::EMERGENCY, 
                   ucus_msgs::msg::FlightMode::REASON_FAILSAFE);
      }
      // Stay in emergency mode - don't process other state machine logic
      return;
    }
    
    // If emergency condition cleared and we're in emergency, allow recovery
    // (user can manually exit emergency by sending a mode command)
    if (current_mode_ == ucus_msgs::msg::FlightMode::EMERGENCY && !emergency_condition) {
      RCLCPP_INFO(this->get_logger(), 
        "Emergency condition cleared. Ready for manual mode change.");
      // Don't auto-exit emergency - require explicit user command
      return;
    }

    // Don't allow automatic transitions if disabled
    if (!enable_auto_transitions_) {
      return;
    }

    // State-specific automatic progression
    switch (current_mode_) {
      case ucus_msgs::msg::FlightMode::VTOL_TAKEOFF:
        handleTakeoffState();
        break;
        
      case ucus_msgs::msg::FlightMode::HOVER:
        // Hover mode maintains position, no automatic transitions
        break;
        
      case ucus_msgs::msg::FlightMode::TRANSITION_MC_TO_FW:
        handleTransitionMcToFw();
        break;
        
      case ucus_msgs::msg::FlightMode::FW:
        // Forward flight maintains cruise, no automatic transitions
        break;
        
      case ucus_msgs::msg::FlightMode::TRANSITION_FW_TO_MC:
        handleTransitionFwToMc();
        break;
        
      case ucus_msgs::msg::FlightMode::LAND:
        handleLandingState();
        break;
        
      case ucus_msgs::msg::FlightMode::MC:
        // Basic MC mode, no automatic transitions
        break;
        
      default:
        break;
    }
  }

  // ========== Mode Request Handler ==========
  void handleModeRequest(uint8_t requested_mode, uint8_t reason)
  {
    // Emergency mode always overrides
    if (requested_mode == ucus_msgs::msg::FlightMode::EMERGENCY) {
      changeMode(ucus_msgs::msg::FlightMode::EMERGENCY, 
                 ucus_msgs::msg::FlightMode::REASON_FAILSAFE);
      return;
    }

    // Allow exit from emergency if conditions are met
    if (current_mode_ == ucus_msgs::msg::FlightMode::EMERGENCY) {
      // Check if emergency conditions are still present
      if (checkEmergencyConditions()) {
        RCLCPP_WARN(this->get_logger(), 
          "Cannot exit EMERGENCY mode: emergency conditions still present");
        return;
      }
      // Emergency conditions cleared - allow mode change
      RCLCPP_INFO(this->get_logger(), 
        "Emergency conditions cleared. Exiting emergency mode.");
    }

    // Same mode - just update reason if different
    if (requested_mode == current_mode_) {
      if (current_reason_ != reason) {
        current_reason_ = reason;
        RCLCPP_DEBUG(this->get_logger(),
          "Mode %d reason updated to %d", current_mode_, reason);
      }
      return;
    }

    // Validate mode transitions
    if (!isValidTransition(current_mode_, requested_mode)) {
      RCLCPP_WARN(this->get_logger(),
        "Invalid transition from mode %d to mode %d", current_mode_, requested_mode);
      return;
    }

    // Check preconditions for requested mode
    if (!checkModePreconditions(requested_mode)) {
      RCLCPP_WARN(this->get_logger(),
        "Preconditions not met for mode %d", requested_mode);
      return;
    }

    // Execute transition
    changeMode(requested_mode, reason);
  }

  // ========== State Handlers ==========
  void handleTakeoffState()
  {
    if (!have_odom_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Takeoff: No odometry available");
      return;
    }

    double current_alt = last_odom_.pose.pose.position.z;
    double elapsed = (this->now() - takeoff_start_time_).seconds();

    // Check timeout
    if (elapsed > takeoff_timeout_) {
      RCLCPP_WARN(this->get_logger(), 
        "Takeoff timeout after %.1f s, transitioning to HOVER", elapsed);
      changeMode(ucus_msgs::msg::FlightMode::HOVER, 
                 ucus_msgs::msg::FlightMode::REASON_MISSION);
      return;
    }

    // Check if target altitude reached
    if (current_alt >= takeoff_target_alt_ - 1.0) { // 1m tolerance
      RCLCPP_INFO(this->get_logger(), 
        "Takeoff complete: altitude %.1f m reached", current_alt);
      changeMode(ucus_msgs::msg::FlightMode::HOVER, 
                 ucus_msgs::msg::FlightMode::REASON_MISSION);
    }
  }

  void handleTransitionMcToFw()
  {
    if (!have_odom_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Transition MC->FW: No odometry available");
      return;
    }

    double elapsed = (this->now() - transition_start_time_).seconds();

    // Check timeout
    if (elapsed > transition_timeout_) {
      RCLCPP_WARN(this->get_logger(), 
        "Transition MC->FW timeout after %.1f s, aborting to MC", elapsed);
      changeMode(ucus_msgs::msg::FlightMode::MC, 
                 ucus_msgs::msg::FlightMode::REASON_FAILSAFE);
      return;
    }

    // Check abort conditions
    double alt = last_odom_.pose.pose.position.z;
    if (alt < min_fw_alt_ - 5.0) { // 5m safety margin
      RCLCPP_WARN(this->get_logger(), 
        "Transition MC->FW aborted: altitude too low (%.1f m)", alt);
      changeMode(ucus_msgs::msg::FlightMode::MC, 
                 ucus_msgs::msg::FlightMode::REASON_FAILSAFE);
      return;
    }

    // Check completion conditions
    if (airspeed_ >= min_transition_airspeed_ && 
        std::abs(pitch_) < max_transition_pitch_ &&
        std::abs(roll_) < max_transition_roll_) {
      RCLCPP_INFO(this->get_logger(), 
        "Transition MC->FW complete: airspeed %.1f m/s", airspeed_);
      changeMode(ucus_msgs::msg::FlightMode::FW, 
                 ucus_msgs::msg::FlightMode::REASON_MISSION);
    }
  }

  void handleTransitionFwToMc()
  {
    if (!have_odom_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Transition FW->MC: No odometry available");
      return;
    }

    double elapsed = (this->now() - transition_start_time_).seconds();

    // Check timeout
    if (elapsed > transition_timeout_) {
      RCLCPP_WARN(this->get_logger(), 
        "Transition FW->MC timeout after %.1f s, forcing LAND", elapsed);
      changeMode(ucus_msgs::msg::FlightMode::LAND, 
                 ucus_msgs::msg::FlightMode::REASON_FAILSAFE);
      return;
    }

    // Check completion conditions: low airspeed and stable attitude
    if (airspeed_ < hover_max_velocity_ && 
        std::abs(pitch_) < max_transition_pitch_ &&
        std::abs(roll_) < max_transition_roll_) {
      RCLCPP_INFO(this->get_logger(), 
        "Transition FW->MC complete: airspeed %.1f m/s", airspeed_);
      changeMode(ucus_msgs::msg::FlightMode::HOVER, 
                 ucus_msgs::msg::FlightMode::REASON_MISSION);
    }
  }

  void handleLandingState()
  {
    if (!have_odom_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Landing: No odometry available");
      return;
    }

    double current_alt = last_odom_.pose.pose.position.z;
    double elapsed = (this->now() - transition_start_time_).seconds();

    // Check timeout
    if (elapsed > landing_timeout_) {
      RCLCPP_WARN(this->get_logger(), 
        "Landing timeout after %.1f s, entering EMERGENCY", elapsed);
      changeMode(ucus_msgs::msg::FlightMode::EMERGENCY, 
                 ucus_msgs::msg::FlightMode::REASON_FAILSAFE);
      return;
    }

    // Check if landed (very low altitude and low velocity)
    if (current_alt < landing_alt_threshold_ && 
        airspeed_ < hover_max_velocity_ &&
        std::abs(velocity_z_) < 0.5) {
      RCLCPP_INFO(this->get_logger(), 
        "Landing complete: altitude %.2f m, velocity %.2f m/s", 
        current_alt, airspeed_);
      // Could transition to MC or stay in LAND
      // For now, stay in LAND mode
    }
  }

  // ========== Helper Functions ==========
  bool checkEmergencyConditions()
  {
    // Check safety status from safety monitor
    if (have_safety_status_) {
      // Only truly critical issues trigger emergency:
      // - Critical battery (vehicle will lose power)
      // - Missing odometry (can't control vehicle)
      // Note: Missing GPS/RC link are warnings, not emergencies for testing
      if (last_safety_status_.critical_battery) {
        return true;
      }
      // Check if sensors are unhealthy - only emergency if IMU is unhealthy
      // (odometry depends on IMU, so IMU failure is critical)
      // GPS and RC link failures are warnings, not emergencies for testing
      if (!last_safety_status_.imu_healthy) {
        return true;  // IMU failure is critical
      }
      // If sensors_healthy is false but IMU is healthy, it's likely just missing GPS
      // which is acceptable for testing - don't trigger emergency
    }
    
    // Check odometry timeout
    if (have_odom_) {
      double odom_age = (this->now() - last_odom_stamp_).seconds();
      if (odom_age > odom_timeout_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          "Emergency: Odometry timeout (%.2f s > %.2f s) - last received %.2f s ago", 
          odom_age, odom_timeout_, odom_age);
        return true;
      }
      // Odometry is fresh - no emergency
      return false;
    } else {
      // No odometry at all - but only trigger emergency if we've been waiting
      // This prevents immediate emergency on startup
      double time_since_startup = (this->now() - startup_time_).seconds();
      if (time_since_startup > 2.0) {  // Allow 2 seconds for odometry to arrive
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          "Emergency: No odometry received (startup: %.1f s ago)", time_since_startup);
        return true;
      }
      // Still in startup grace period
      return false;
    }
  }

  bool isValidTransition(uint8_t from_mode, uint8_t to_mode)
  {
    // Emergency can be entered from any mode
    if (to_mode == ucus_msgs::msg::FlightMode::EMERGENCY) {
      return true;
    }

    // Define valid transitions
    switch (from_mode) {
      case ucus_msgs::msg::FlightMode::MC:
        return (to_mode == ucus_msgs::msg::FlightMode::VTOL_TAKEOFF ||
                to_mode == ucus_msgs::msg::FlightMode::HOVER ||
                to_mode == ucus_msgs::msg::FlightMode::TRANSITION_MC_TO_FW);
        
      case ucus_msgs::msg::FlightMode::VTOL_TAKEOFF:
        return (to_mode == ucus_msgs::msg::FlightMode::HOVER ||
                to_mode == ucus_msgs::msg::FlightMode::MC ||
                to_mode == ucus_msgs::msg::FlightMode::LAND);
        
      case ucus_msgs::msg::FlightMode::HOVER:
        return (to_mode == ucus_msgs::msg::FlightMode::TRANSITION_MC_TO_FW ||
                to_mode == ucus_msgs::msg::FlightMode::LAND ||
                to_mode == ucus_msgs::msg::FlightMode::MC);
        
      case ucus_msgs::msg::FlightMode::TRANSITION_MC_TO_FW:
        return (to_mode == ucus_msgs::msg::FlightMode::FW ||
                to_mode == ucus_msgs::msg::FlightMode::MC ||
                to_mode == ucus_msgs::msg::FlightMode::HOVER);
        
      case ucus_msgs::msg::FlightMode::FW:
        return (to_mode == ucus_msgs::msg::FlightMode::TRANSITION_FW_TO_MC ||
                to_mode == ucus_msgs::msg::FlightMode::LAND);
        
      case ucus_msgs::msg::FlightMode::TRANSITION_FW_TO_MC:
        return (to_mode == ucus_msgs::msg::FlightMode::HOVER ||
                to_mode == ucus_msgs::msg::FlightMode::MC ||
                to_mode == ucus_msgs::msg::FlightMode::LAND);
        
      case ucus_msgs::msg::FlightMode::LAND:
        return (to_mode == ucus_msgs::msg::FlightMode::MC ||
                to_mode == ucus_msgs::msg::FlightMode::HOVER);
        
      default:
        return false;
    }
  }

  bool checkModePreconditions(uint8_t mode)
  {
    if (!have_odom_) {
      // Most modes require odometry
      if (mode != ucus_msgs::msg::FlightMode::MC) {
        return false;
      }
    }

    switch (mode) {
      case ucus_msgs::msg::FlightMode::FW:
      case ucus_msgs::msg::FlightMode::TRANSITION_MC_TO_FW: {
        double alt = last_odom_.pose.pose.position.z;
        if (alt < min_fw_alt_) {
          RCLCPP_WARN(this->get_logger(),
            "FW mode requires altitude >= %.1f m (current: %.1f m)", 
            min_fw_alt_, alt);
          return false;
        }
        break;
      }
      
      case ucus_msgs::msg::FlightMode::TRANSITION_FW_TO_MC: {
        // Should have some airspeed to transition from
        if (airspeed_ > max_transition_airspeed_) {
          RCLCPP_WARN(this->get_logger(),
            "Transition FW->MC: airspeed too high (%.1f m/s)", airspeed_);
          return false;
        }
        break;
      }
      
      default:
        break;
    }

    return true;
  }

  void changeMode(uint8_t new_mode, uint8_t reason)
  {
    if (new_mode == current_mode_) {
      return; // No change needed
    }

    uint8_t old_mode = current_mode_;
    current_mode_ = new_mode;
    current_reason_ = reason;

    // Update transition timing
    if (new_mode == ucus_msgs::msg::FlightMode::TRANSITION_MC_TO_FW ||
        new_mode == ucus_msgs::msg::FlightMode::TRANSITION_FW_TO_MC ||
        new_mode == ucus_msgs::msg::FlightMode::LAND) {
      transition_start_time_ = this->now();
    }

    if (new_mode == ucus_msgs::msg::FlightMode::VTOL_TAKEOFF) {
      takeoff_start_time_ = this->now();
    }

    RCLCPP_INFO(this->get_logger(), 
      "Mode changed: %d -> %d (reason: %d)", old_mode, new_mode, reason);
  }

  void extractAttitude(const geometry_msgs::msg::Quaternion& q)
  {
    // Convert quaternion to Euler angles (roll, pitch, yaw)
    // Using standard aerospace sequence: ZYX (yaw-pitch-roll)
    double qx = q.x, qy = q.y, qz = q.z, qw = q.w;
    
    // Roll (x-axis rotation)
    double sinr_cosp = 2.0 * (qw * qx + qy * qz);
    double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
    roll_ = std::atan2(sinr_cosp, cosr_cosp);
    
    // Pitch (y-axis rotation)
    double sinp = 2.0 * (qw * qy - qz * qx);
    if (std::abs(sinp) >= 1.0) {
      pitch_ = std::copysign(M_PI / 2.0, sinp); // Use 90 degrees if out of range
    } else {
      pitch_ = std::asin(sinp);
    }
    
    // Yaw (z-axis rotation)
    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    yaw_ = std::atan2(siny_cosp, cosy_cosp);
  }

  void publishMode()
  {
    ucus_msgs::msg::FlightMode msg;
    msg.mode = current_mode_;
    msg.reason = current_reason_;
    mode_pub_->publish(msg);
  }

  // ========== ROS Objects ==========
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr mode_cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<ucus_msgs::msg::SafetyStatus>::SharedPtr safety_status_sub_;
  rclcpp::Publisher<ucus_msgs::msg::FlightMode>::SharedPtr mode_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr state_machine_timer_;

  // ========== State Variables ==========
  nav_msgs::msg::Odometry last_odom_;
  rclcpp::Time last_odom_stamp_;
  bool have_odom_{false};
  
  ucus_msgs::msg::SafetyStatus last_safety_status_;
  bool have_safety_status_{false};
  
  uint8_t current_mode_;
  uint8_t current_reason_;
  
    // Timing
    rclcpp::Time transition_start_time_;
    rclcpp::Time takeoff_start_time_;
    rclcpp::Time startup_time_;
  
  // Odometry-derived state
  double velocity_x_{0.0}, velocity_y_{0.0}, velocity_z_{0.0};
  double airspeed_{0.0};
  double roll_{0.0}, pitch_{0.0}, yaw_{0.0};

  // ========== Parameters ==========
  // Altitude
  double min_fw_alt_;
  double takeoff_target_alt_;
  double landing_alt_threshold_;
  
  // Velocity
  double min_transition_airspeed_;
  double max_transition_airspeed_;
  double hover_max_velocity_;
  
  // Attitude
  double max_transition_pitch_;
  double max_transition_roll_;
  
  // Timing
  double transition_timeout_;
  double takeoff_timeout_;
  double landing_timeout_;
  double odom_timeout_;
  
  // Features
  bool enable_auto_transitions_;
  
  // Rates
  int pub_rate_hz_;
  int state_machine_rate_hz_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ModeManager>());
  rclcpp::shutdown();
  return 0;
}
