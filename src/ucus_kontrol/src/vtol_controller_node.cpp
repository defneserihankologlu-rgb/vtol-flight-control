#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ucus_msgs/msg/flight_mode.hpp"
#include "ucus_msgs/msg/actuator_cmd.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <algorithm>
#include <cmath>

class VtolController : public rclcpp::Node
{
public:
  VtolController() : Node("vtol_controller")
  {
    // ========== MC Position Control Parameters (PID) ==========
    // Altitude (Z) control
    kp_z_ = this->declare_parameter<double>("kp_z", 0.8);
    ki_z_ = this->declare_parameter<double>("ki_z", 0.05);
    kd_z_ = this->declare_parameter<double>("kd_z", 0.1);
    integral_z_max_ = this->declare_parameter<double>("integral_z_max", 0.5);
    
    // Horizontal position (X, Y) control
    kp_xy_ = this->declare_parameter<double>("kp_xy", 0.5);
    ki_xy_ = this->declare_parameter<double>("ki_xy", 0.02);
    kd_xy_ = this->declare_parameter<double>("kd_xy", 0.08);
    integral_xy_max_ = this->declare_parameter<double>("integral_xy_max", 0.3);
    
    // Velocity feedforward gains
    kv_xy_ = this->declare_parameter<double>("kv_xy", 0.3);
    kv_z_ = this->declare_parameter<double>("kv_z", 0.2);
    
    // ========== MC Attitude Control Parameters (PID) ==========
    // Roll control
    kp_roll_ = this->declare_parameter<double>("kp_roll", 0.8);
    ki_roll_ = this->declare_parameter<double>("ki_roll", 0.05);
    kd_roll_ = this->declare_parameter<double>("kd_roll", 0.1);
    
    // Pitch control
    kp_pitch_ = this->declare_parameter<double>("kp_pitch", 0.8);
    ki_pitch_ = this->declare_parameter<double>("ki_pitch", 0.05);
    kd_pitch_ = this->declare_parameter<double>("kd_pitch", 0.1);
    
    // Yaw control
    kp_yaw_ = this->declare_parameter<double>("kp_yaw", 0.3);
    ki_yaw_ = this->declare_parameter<double>("ki_yaw", 0.02);
    kd_yaw_ = this->declare_parameter<double>("kd_yaw", 0.05);
    
    // ========== Attitude Rate Control Parameters ==========
    kp_roll_rate_ = this->declare_parameter<double>("kp_roll_rate", 0.15);
    kp_pitch_rate_ = this->declare_parameter<double>("kp_pitch_rate", 0.15);
    kp_yaw_rate_ = this->declare_parameter<double>("kp_yaw_rate", 0.1);
    
    // ========== Position to Attitude Mapping ==========
    max_roll_angle_ = this->declare_parameter<double>("max_roll_angle", 0.35);  // ~20 deg
    max_pitch_angle_ = this->declare_parameter<double>("max_pitch_angle", 0.35); // ~20 deg
    
    hover_thrust_ = this->declare_parameter<double>("hover_thrust", 0.4);
    
    // ========== FW Control Parameters ==========
    kp_airspeed_ = this->declare_parameter<double>("kp_airspeed", 0.1);
    kp_altitude_fw_ = this->declare_parameter<double>("kp_altitude_fw", 0.05);
    cruise_airspeed_ = this->declare_parameter<double>("cruise_airspeed", 15.0);
    cruise_altitude_ = this->declare_parameter<double>("cruise_altitude", 30.0);
    
    // ========== Transition Parameters ==========
    transition_blend_start_ = this->declare_parameter<double>("transition_blend_start", 0.0);
    transition_blend_end_ = this->declare_parameter<double>("transition_blend_end", 1.0);
    transition_duration_ = this->declare_parameter<double>("transition_duration", 6.0);
    transition_airspeed_threshold_ = this->declare_parameter<double>("transition_airspeed_threshold", 12.0);
    
    // ========== Safety Parameters ==========
    thrust_min_ = this->declare_parameter<double>("thrust_min", 0.0);
    thrust_max_ = this->declare_parameter<double>("thrust_max", 1.0);
    control_surface_max_ = this->declare_parameter<double>("control_surface_max", 0.6);
    odom_timeout_ = this->declare_parameter<double>("odom_timeout", 0.5);
    pub_rate_hz_ = this->declare_parameter<int>("pub_rate_hz", 50);
    
    // ========== Actuator Configuration ==========
    // Assume: outputs[0-3] = MC motors, outputs[4-6] = FW surfaces (aileron, elevator, rudder), output[7] = FW throttle
    num_mc_motors_ = this->declare_parameter<int>("num_mc_motors", 4);
    num_fw_surfaces_ = this->declare_parameter<int>("num_fw_surfaces", 3);
    
    // ========== Subscriptions ==========
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&VtolController::odomCb, this, std::placeholders::_1));

    mode_sub_ = this->create_subscription<ucus_msgs::msg::FlightMode>(
      "/flight_mode", 10, std::bind(&VtolController::modeCb, this, std::placeholders::_1));

    armed_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/armed", 10, std::bind(&VtolController::armedCb, this, std::placeholders::_1));

    sp_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/setpoint/pos", 10, std::bind(&VtolController::setpointCb, this, std::placeholders::_1));

    // ========== Publishers ==========
    act_pub_ = this->create_publisher<ucus_msgs::msg::ActuatorCmd>("/actuator_cmd", 10);

    // ========== Control Loop Timer ==========
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000 / pub_rate_hz_),
      std::bind(&VtolController::controlLoop, this));
    
    // Initialize transition state
    transition_progress_ = 0.0;
    transition_start_time_ = this->now();
    
    // Initialize PID states
    resetPidStates();
    last_control_time_ = this->now();
    
    RCLCPP_INFO(this->get_logger(), "VTOL Controller initialized with PID control");
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
    
    // Extract attitude and angular velocities
    extractAttitude(msg->pose.pose.orientation);
    
    // Extract angular velocities
    angular_velocity_x_ = msg->twist.twist.angular.x;
    angular_velocity_y_ = msg->twist.twist.angular.y;
    angular_velocity_z_ = msg->twist.twist.angular.z;
  }

  void modeCb(const ucus_msgs::msg::FlightMode::SharedPtr msg) 
  { 
    uint8_t old_mode = current_mode_;
    current_mode_ = msg->mode;
    
    // Reset transition state on mode change
    if (old_mode != current_mode_) {
      transition_progress_ = 0.0;
      transition_start_time_ = this->now();
      resetPidStates();  // Reset PID integrals on mode change
      
      RCLCPP_INFO(this->get_logger(), 
        "Controller mode changed: %d -> %d", old_mode, current_mode_);
    }
  }

  void armedCb(const std_msgs::msg::Bool::SharedPtr msg) 
  { 
    armed_ = msg->data;
    if (!armed_) {
      RCLCPP_WARN(this->get_logger(), "Vehicle disarmed - zeroing all outputs");
    }
  }

  void setpointCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    sp_ = *msg;
    have_sp_ = true;
  }

  // ========== Main Control Loop ==========
  void controlLoop()
  {
    ucus_msgs::msg::ActuatorCmd cmd;
    
    // ========== Safety Checks ==========
    // Check odometry freshness
    if (!have_odom_ || (this->now() - last_odom_stamp_).seconds() > odom_timeout_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Odometry timeout - zeroing outputs");
      cmd.outputs = std::vector<float>(num_mc_motors_ + num_fw_surfaces_ + 1, 0.0f);
      act_pub_->publish(cmd);
      return;
    }

    // Check armed status
    if (!armed_) {
      cmd.outputs = std::vector<float>(num_mc_motors_ + num_fw_surfaces_ + 1, 0.0f);
      act_pub_->publish(cmd);
      return;
    }

    // ========== Mode-Specific Control ==========
    switch (current_mode_) {
      case ucus_msgs::msg::FlightMode::MC:
        cmd = runMcController();
        break;
        
      case ucus_msgs::msg::FlightMode::VTOL_TAKEOFF:
        cmd = runTakeoffController();
        break;
        
      case ucus_msgs::msg::FlightMode::HOVER:
        cmd = runHoverController();
        break;
        
      case ucus_msgs::msg::FlightMode::TRANSITION_MC_TO_FW:
        cmd = runTransitionMcToFwController();
        break;
        
      case ucus_msgs::msg::FlightMode::FW:
        cmd = runFwController();
        break;
        
      case ucus_msgs::msg::FlightMode::TRANSITION_FW_TO_MC:
        cmd = runTransitionFwToMcController();
        break;
        
      case ucus_msgs::msg::FlightMode::LAND:
        cmd = runLandingController();
        break;
        
      case ucus_msgs::msg::FlightMode::EMERGENCY:
        cmd = runEmergencyController();
        break;
        
      default:
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
          "Unknown mode %d - using emergency controller", current_mode_);
        cmd = runEmergencyController();
        break;
    }

    act_pub_->publish(cmd);
  }

  // ========== Specialized Controllers ==========
  
  ucus_msgs::msg::ActuatorCmd runMcController()
  {
    // Basic MC controller with PID altitude control
    ucus_msgs::msg::ActuatorCmd cmd;
    cmd.outputs.resize(num_mc_motors_ + num_fw_surfaces_ + 1, 0.0f);
    
    double dt = (this->now() - last_control_time_).seconds();
    dt = std::clamp(dt, 0.001, 0.1);
    last_control_time_ = this->now();
    
    double thrust = hover_thrust_;
    
    if (have_sp_) {
      double z_sp = sp_.pose.position.z;
      double z_meas = last_odom_.pose.pose.position.z;
      double e_z = z_sp - z_meas;
      
      // PID altitude control
      double u_z = computePid(e_z, dt, pid_z_, kp_z_, ki_z_, kd_z_, integral_z_max_);
      thrust = hover_thrust_ + u_z;
    }
    
    thrust = std::clamp(thrust, thrust_min_, thrust_max_);
    
    // Apply same thrust to all MC motors (no attitude control in basic MC mode)
    for (int i = 0; i < num_mc_motors_; i++) {
      cmd.outputs[i] = static_cast<float>(thrust);
    }
    
    // Zero FW surfaces and throttle
    for (size_t i = num_mc_motors_; i < cmd.outputs.size(); i++) {
      cmd.outputs[i] = 0.0f;
    }
    
    return cmd;
  }

  ucus_msgs::msg::ActuatorCmd runTakeoffController()
  {
    // Takeoff controller with PID altitude control
    ucus_msgs::msg::ActuatorCmd cmd;
    cmd.outputs.resize(num_mc_motors_ + num_fw_surfaces_ + 1, 0.0f);
    
    double dt = (this->now() - last_control_time_).seconds();
    dt = std::clamp(dt, 0.001, 0.1);
    last_control_time_ = this->now();
    
    double thrust = hover_thrust_ * 1.2; // 20% more thrust for takeoff
    
    if (have_sp_) {
      double z_sp = sp_.pose.position.z;
      double z_meas = last_odom_.pose.pose.position.z;
      double e_z = z_sp - z_meas;
      
      // PID altitude control with takeoff boost
      double u_z = computePid(e_z, dt, pid_z_, kp_z_, ki_z_, kd_z_, integral_z_max_);
      thrust = hover_thrust_ * 1.2 + u_z;
    }
    
    thrust = std::clamp(thrust, thrust_min_, thrust_max_);
    
    for (int i = 0; i < num_mc_motors_; i++) {
      cmd.outputs[i] = static_cast<float>(thrust);
    }
    
    return cmd;
  }

  ucus_msgs::msg::ActuatorCmd runHoverController()
  {
    // Full 6-DOF position and attitude control for hover
    ucus_msgs::msg::ActuatorCmd cmd;
    cmd.outputs.resize(num_mc_motors_ + num_fw_surfaces_ + 1, 0.0f);
    
    // Calculate time step
    double dt = (this->now() - last_control_time_).seconds();
    dt = std::clamp(dt, 0.001, 0.1);  // Limit dt to reasonable range
    last_control_time_ = this->now();
    
    // Default commands
    double thrust = hover_thrust_;
    double roll_cmd = 0.0;
    double pitch_cmd = 0.0;
    double yaw_cmd = 0.0;
    
    if (have_sp_) {
      // ========== Position Control (Outer Loop) ==========
      double x_sp = sp_.pose.position.x;
      double y_sp = sp_.pose.position.y;
      double z_sp = sp_.pose.position.z;
      
      double x_meas = last_odom_.pose.pose.position.x;
      double y_meas = last_odom_.pose.pose.position.y;
      double z_meas = last_odom_.pose.pose.position.z;
      
      // Position errors
      double e_x = x_sp - x_meas;
      double e_y = y_sp - y_meas;
      double e_z = z_sp - z_meas;
      
      // Velocity feedforward (if setpoint has velocity info, use it)
      double vx_sp = 0.0, vy_sp = 0.0, vz_sp = 0.0;
      
      // PID control for position
      double u_x = computePid(e_x, dt, pid_x_, kp_xy_, ki_xy_, kd_xy_, integral_xy_max_);
      double u_y = computePid(e_y, dt, pid_y_, kp_xy_, ki_xy_, kd_xy_, integral_xy_max_);
      double u_z = computePid(e_z, dt, pid_z_, kp_z_, ki_z_, kd_z_, integral_z_max_);
      
      // Add velocity feedforward
      u_x += kv_xy_ * vx_sp;
      u_y += kv_xy_ * vy_sp;
      u_z += kv_z_ * vz_sp;
      
      // Convert horizontal position commands to attitude commands
      // Limit attitude commands to prevent excessive tilting
      roll_cmd = std::clamp(-u_y, -max_roll_angle_, max_roll_angle_);   // Negative for correct direction
      pitch_cmd = std::clamp(u_x, -max_pitch_angle_, max_pitch_angle_);
      
      // Altitude control (thrust)
      thrust = hover_thrust_ + u_z;
      
      // ========== Attitude Control (Inner Loop) ==========
      // Extract desired attitude from setpoint (if available)
      double roll_sp = 0.0, pitch_sp = 0.0, yaw_sp = yaw_;
      
      // If setpoint has orientation, use it
      if (std::abs(sp_.pose.orientation.w) > 0.001 || 
          std::abs(sp_.pose.orientation.x) > 0.001 ||
          std::abs(sp_.pose.orientation.y) > 0.001 ||
          std::abs(sp_.pose.orientation.z) > 0.001) {
        // Extract setpoint attitude
        double qx = sp_.pose.orientation.x;
        double qy = sp_.pose.orientation.y;
        double qz = sp_.pose.orientation.z;
        double qw = sp_.pose.orientation.w;
        
        // Convert to Euler (simplified - just yaw for now)
        double siny_cosp = 2.0 * (qw * qz + qx * qy);
        double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
        yaw_sp = std::atan2(siny_cosp, cosy_cosp);
      }
      
      // Combine position-derived attitude with setpoint attitude
      roll_sp = roll_cmd;
      pitch_sp = pitch_cmd;
      
      // Attitude errors
      double e_roll = roll_sp - roll_;
      double e_pitch = pitch_sp - pitch_;
      double e_yaw = yaw_sp - yaw_;
      
      // Normalize yaw error to [-pi, pi]
      while (e_yaw > M_PI) e_yaw -= 2.0 * M_PI;
      while (e_yaw < -M_PI) e_yaw += 2.0 * M_PI;
      
      // PID control for attitude
      double roll_rate_cmd = computePid(e_roll, dt, pid_roll_, kp_roll_, ki_roll_, kd_roll_);
      double pitch_rate_cmd = computePid(e_pitch, dt, pid_pitch_, kp_pitch_, ki_pitch_, kd_pitch_);
      double yaw_rate_cmd = computePid(e_yaw, dt, pid_yaw_, kp_yaw_, ki_yaw_, kd_yaw_);
      
      // ========== Attitude Rate Control ==========
      double roll_rate_error = roll_rate_cmd - angular_velocity_x_;
      double pitch_rate_error = pitch_rate_cmd - angular_velocity_y_;
      double yaw_rate_error = yaw_rate_cmd - angular_velocity_z_;
      
      roll_cmd = kp_roll_rate_ * roll_rate_error;
      pitch_cmd = kp_pitch_rate_ * pitch_rate_error;
      yaw_cmd = kp_yaw_rate_ * yaw_rate_error;
    }
    
    // Clamp thrust
    thrust = std::clamp(thrust, thrust_min_, thrust_max_);
    
    // Mix motors for differential control
    mixMotors(thrust, roll_cmd, pitch_cmd, yaw_cmd, cmd.outputs);
    
    // Zero FW surfaces and throttle
    for (size_t i = num_mc_motors_; i < cmd.outputs.size(); i++) {
      cmd.outputs[i] = 0.0f;
    }
    
    return cmd;
  }

  ucus_msgs::msg::ActuatorCmd runTransitionMcToFwController()
  {
    // Blend MC motors and FW control surfaces
    ucus_msgs::msg::ActuatorCmd cmd;
    cmd.outputs.resize(num_mc_motors_ + num_fw_surfaces_ + 1, 0.0f);
    
    // Update transition progress (0.0 = MC, 1.0 = FW)
    double elapsed = (this->now() - transition_start_time_).seconds();
    transition_progress_ = std::min(1.0, elapsed / transition_duration_);
    
    // Check if we have sufficient airspeed to complete transition
    // If airspeed is too low, slow down the transition
    if (airspeed_ < transition_airspeed_threshold_ && transition_progress_ > 0.5) {
      // Slow down transition if airspeed is insufficient
      transition_progress_ = 0.5 + 0.3 * (airspeed_ / transition_airspeed_threshold_);
    }
    
    // Blend factor
    double mc_factor = 1.0 - transition_progress_;
    double fw_factor = transition_progress_;
    
    // MC motors: reduce as transition progresses
    double mc_thrust = hover_thrust_ * mc_factor;
    for (int i = 0; i < num_mc_motors_; i++) {
      cmd.outputs[i] = static_cast<float>(std::clamp(mc_thrust, thrust_min_, thrust_max_));
    }
    
    // FW surfaces: increase as transition progresses
    // Simple forward pitch for transition
    double pitch_cmd = 0.2 * fw_factor; // Pitch forward
    pitch_cmd = std::clamp(pitch_cmd, -control_surface_max_, control_surface_max_);
    cmd.outputs[num_mc_motors_] = 0.0f;                              // Aileron (not used)
    cmd.outputs[num_mc_motors_ + 1] = static_cast<float>(pitch_cmd); // Elevator
    cmd.outputs[num_mc_motors_ + 2] = 0.0f;                          // Rudder
    
    // FW throttle: increase during transition
    double fw_throttle = 0.3 * fw_factor;
    cmd.outputs[num_mc_motors_ + num_fw_surfaces_] = static_cast<float>(fw_throttle);
    
    return cmd;
  }

  ucus_msgs::msg::ActuatorCmd runFwController()
  {
    // Fixed-wing control: aileron, elevator, rudder, throttle
    ucus_msgs::msg::ActuatorCmd cmd;
    cmd.outputs.resize(num_mc_motors_ + num_fw_surfaces_ + 1, 0.0f);
    
    // Zero MC motors
    for (int i = 0; i < num_mc_motors_; i++) {
      cmd.outputs[i] = 0.0f;
    }
    
    // Airspeed control
    double airspeed_error = cruise_airspeed_ - airspeed_;
    double throttle = 0.5 + kp_airspeed_ * airspeed_error; // Base throttle 0.5
    throttle = std::clamp(throttle, 0.0, 1.0);
    
    // Altitude control via elevator
    double alt_error = 0.0;
    if (have_sp_) {
      double z_sp = sp_.pose.position.z;
      double z_meas = last_odom_.pose.pose.position.z;
      alt_error = z_sp - z_meas;
    }
    double elevator = kp_altitude_fw_ * alt_error;
    elevator = std::clamp(elevator, -control_surface_max_, control_surface_max_);
    
    // Control surfaces
    cmd.outputs[num_mc_motors_] = 0.0f;                              // Aileron
    cmd.outputs[num_mc_motors_ + 1] = static_cast<float>(elevator);  // Elevator
    cmd.outputs[num_mc_motors_ + 2] = 0.0f;                          // Rudder
    
    // Throttle
    cmd.outputs[num_mc_motors_ + num_fw_surfaces_] = static_cast<float>(throttle);
    
    return cmd;
  }

  ucus_msgs::msg::ActuatorCmd runTransitionFwToMcController()
  {
    // Reverse transition: blend from FW back to MC
    ucus_msgs::msg::ActuatorCmd cmd;
    cmd.outputs.resize(num_mc_motors_ + num_fw_surfaces_ + 1, 0.0f);
    
    // Update transition progress (0.0 = FW, 1.0 = MC)
    double elapsed = (this->now() - transition_start_time_).seconds();
    transition_progress_ = std::min(1.0, elapsed / transition_duration_);
    
    // Blend factor (inverse of MC->FW)
    double fw_factor = 1.0 - transition_progress_;
    double mc_factor = transition_progress_;
    
    // MC motors: increase as transition progresses
    double mc_thrust = hover_thrust_ * mc_factor;
    for (int i = 0; i < num_mc_motors_; i++) {
      cmd.outputs[i] = static_cast<float>(std::clamp(mc_thrust, thrust_min_, thrust_max_));
    }
    
    // FW surfaces: reduce as transition progresses
    double fw_throttle = 0.3 * fw_factor;
    cmd.outputs[num_mc_motors_ + num_fw_surfaces_] = static_cast<float>(fw_throttle);
    
    // Zero control surfaces gradually
    cmd.outputs[num_mc_motors_] = 0.0f;
    cmd.outputs[num_mc_motors_ + 1] = 0.0f;
    cmd.outputs[num_mc_motors_ + 2] = 0.0f;
    
    return cmd;
  }

  ucus_msgs::msg::ActuatorCmd runLandingController()
  {
    // Controlled descent with PID control
    ucus_msgs::msg::ActuatorCmd cmd;
    cmd.outputs.resize(num_mc_motors_ + num_fw_surfaces_ + 1, 0.0f);
    
    double dt = (this->now() - last_control_time_).seconds();
    dt = std::clamp(dt, 0.001, 0.1);
    last_control_time_ = this->now();
    
    double thrust = hover_thrust_ * 0.8; // Reduced thrust for descent
    
    if (have_sp_) {
      double z_sp = sp_.pose.position.z;
      double z_meas = last_odom_.pose.pose.position.z;
      double e_z = z_sp - z_meas;
      
      // PID control with reduced gains for gentler landing
      double u_z = computePid(e_z, dt, pid_z_, kp_z_ * 0.5, ki_z_ * 0.5, kd_z_ * 0.5, integral_z_max_);
      thrust = hover_thrust_ * 0.8 + u_z;
    }
    
    thrust = std::clamp(thrust, thrust_min_, thrust_max_);
    
    for (int i = 0; i < num_mc_motors_; i++) {
      cmd.outputs[i] = static_cast<float>(thrust);
    }
    
    return cmd;
  }

  ucus_msgs::msg::ActuatorCmd runEmergencyController()
  {
    // Emergency: try to maintain hover or safe descent
    ucus_msgs::msg::ActuatorCmd cmd;
    cmd.outputs.resize(num_mc_motors_ + num_fw_surfaces_ + 1, 0.0f);
    
    // Minimal thrust to maintain control
    double emergency_thrust = hover_thrust_ * 0.6;
    
    for (int i = 0; i < num_mc_motors_; i++) {
      cmd.outputs[i] = static_cast<float>(emergency_thrust);
    }
    
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Emergency controller active");
    
    return cmd;
  }

  // ========== PID Controller Functions ==========
  struct PidState {
    double integral{0.0};
    double last_error{0.0};
    double last_derivative{0.0};
  };
  
  double computePid(double error, double dt, PidState& state, double kp, double ki, double kd, double integral_max = 1.0)
  {
    // Proportional term
    double p_term = kp * error;
    
    // Integral term with anti-windup
    state.integral += error * dt;
    state.integral = std::clamp(state.integral, -integral_max, integral_max);
    double i_term = ki * state.integral;
    
    // Derivative term (filtered)
    double derivative = (error - state.last_error) / dt;
    // Low-pass filter on derivative to reduce noise
    double alpha = 0.7;  // Filter coefficient
    derivative = alpha * derivative + (1.0 - alpha) * state.last_derivative;
    state.last_derivative = derivative;
    double d_term = kd * derivative;
    
    state.last_error = error;
    
    return p_term + i_term + d_term;
  }
  
  void resetPidStates()
  {
    pid_x_ = PidState();
    pid_y_ = PidState();
    pid_z_ = PidState();
    pid_roll_ = PidState();
    pid_pitch_ = PidState();
    pid_yaw_ = PidState();
  }
  
  // ========== Motor Mixing Functions ==========
  // Standard quadcopter X-frame mixing
  // Motors: [0:FL, 1:FR, 2:BL, 3:BR] (Front-Left, Front-Right, Back-Left, Back-Right)
  void mixMotors(double thrust, double roll_cmd, double pitch_cmd, double yaw_cmd,
                 std::vector<float>& outputs)
  {
    // Normalize commands to [-1, 1] range
    roll_cmd = std::clamp(roll_cmd, -1.0, 1.0);
    pitch_cmd = std::clamp(pitch_cmd, -1.0, 1.0);
    yaw_cmd = std::clamp(yaw_cmd, -1.0, 1.0);
    
    // X-frame quadcopter mixing
    // Roll: +roll means tilt right (increase right motors, decrease left motors)
    // Pitch: +pitch means tilt forward (increase front motors, decrease back motors)
    // Yaw: +yaw means rotate CCW (increase CCW motors, decrease CW motors)
    
    outputs[0] = static_cast<float>(thrust + roll_cmd * 0.3 - pitch_cmd * 0.3 + yaw_cmd * 0.2);  // FL
    outputs[1] = static_cast<float>(thrust - roll_cmd * 0.3 - pitch_cmd * 0.3 - yaw_cmd * 0.2); // FR
    outputs[2] = static_cast<float>(thrust + roll_cmd * 0.3 + pitch_cmd * 0.3 - yaw_cmd * 0.2);  // BL
    outputs[3] = static_cast<float>(thrust - roll_cmd * 0.3 + pitch_cmd * 0.3 + yaw_cmd * 0.2);  // BR
    
    // Clamp outputs
    for (auto& out : outputs) {
      out = std::clamp(out, static_cast<float>(thrust_min_), static_cast<float>(thrust_max_));
    }
  }
  
  // ========== Helper Functions ==========
  void extractAttitude(const geometry_msgs::msg::Quaternion& q)
  {
    // Convert quaternion to Euler angles
    double qx = q.x, qy = q.y, qz = q.z, qw = q.w;
    
    // Roll (x-axis rotation)
    double sinr_cosp = 2.0 * (qw * qx + qy * qz);
    double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
    roll_ = std::atan2(sinr_cosp, cosr_cosp);
    
    // Pitch (y-axis rotation)
    double sinp = 2.0 * (qw * qy - qz * qx);
    if (std::abs(sinp) >= 1.0) {
      pitch_ = std::copysign(M_PI / 2.0, sinp);
    } else {
      pitch_ = std::asin(sinp);
    }
    
    // Yaw (z-axis rotation)
    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    yaw_ = std::atan2(siny_cosp, cosy_cosp);
  }

  // ========== ROS Objects ==========
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<ucus_msgs::msg::FlightMode>::SharedPtr mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr armed_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sp_sub_;
  rclcpp::Publisher<ucus_msgs::msg::ActuatorCmd>::SharedPtr act_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ========== State Variables ==========
  nav_msgs::msg::Odometry last_odom_;
  rclcpp::Time last_odom_stamp_;
  geometry_msgs::msg::PoseStamped sp_;
  bool have_odom_{false}, armed_{false}, have_sp_{false};
  uint8_t current_mode_{0};
  
  // Odometry-derived state
  double velocity_x_{0.0}, velocity_y_{0.0}, velocity_z_{0.0};
  double airspeed_{0.0};
  double roll_{0.0}, pitch_{0.0}, yaw_{0.0};
  double angular_velocity_x_{0.0}, angular_velocity_y_{0.0}, angular_velocity_z_{0.0};
  
  // PID states
  PidState pid_x_, pid_y_, pid_z_;
  PidState pid_roll_, pid_pitch_, pid_yaw_;
  
  // Control timing
  rclcpp::Time last_control_time_;
  
  // Transition state
  double transition_progress_{0.0};
  rclcpp::Time transition_start_time_;

  // ========== Parameters ==========
  // MC Position Control (PID)
  double kp_z_{0.8}, ki_z_{0.05}, kd_z_{0.1};
  double kp_xy_{0.5}, ki_xy_{0.02}, kd_xy_{0.08};
  double integral_z_max_{0.5}, integral_xy_max_{0.3};
  double kv_xy_{0.3}, kv_z_{0.2};  // Velocity feedforward
  
  // MC Attitude Control (PID)
  double kp_roll_{0.8}, ki_roll_{0.05}, kd_roll_{0.1};
  double kp_pitch_{0.8}, ki_pitch_{0.05}, kd_pitch_{0.1};
  double kp_yaw_{0.3}, ki_yaw_{0.02}, kd_yaw_{0.05};
  
  // Attitude Rate Control
  double kp_roll_rate_{0.15}, kp_pitch_rate_{0.15}, kp_yaw_rate_{0.1};
  
  // Position to Attitude Mapping
  double max_roll_angle_{0.35}, max_pitch_angle_{0.35};
  
  double hover_thrust_{0.4};
  
  // FW Control
  double kp_airspeed_{0.1}, kp_altitude_fw_{0.05};
  double cruise_airspeed_{15.0}, cruise_altitude_{30.0};
  
  // Transition
  double transition_blend_start_{0.0}, transition_blend_end_{1.0};
  double transition_duration_{6.0};
  double transition_airspeed_threshold_{12.0};
  
  // Safety
  double thrust_min_{0.0}, thrust_max_{1.0};
  double control_surface_max_{0.6};
  double odom_timeout_{0.5};
  int pub_rate_hz_{50};
  
  // Actuator config
  int num_mc_motors_{4};
  int num_fw_surfaces_{3};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VtolController>());
  rclcpp::shutdown();
  return 0;
}
