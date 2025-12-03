#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"
#include <cmath>
#include <algorithm>

class EkfNode : public rclcpp::Node {
public:
  EkfNode() : Node("ekf_node") {
    // Parameters
    publish_rate_hz_ = this->declare_parameter<int>("publish_rate_hz", 50);
    world_frame_ = this->declare_parameter<std::string>("world_frame", "map");
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
    
    // Sensor enable flags
    use_gps_ = this->declare_parameter<bool>("use_gps", true);
    use_barometer_ = this->declare_parameter<bool>("use_barometer", true);
    use_magnetometer_ = this->declare_parameter<bool>("use_magnetometer", true);
    
    // Sensor noise parameters
    imu_noise_accel_ = this->declare_parameter<double>("imu_noise_accel", 0.05);
    imu_noise_gyro_ = this->declare_parameter<double>("imu_noise_gyro", 0.01);
    gps_noise_xy_ = this->declare_parameter<double>("gps_noise_xy", 1.5);
    gps_noise_z_ = this->declare_parameter<double>("gps_noise_z", 3.0);
    baro_noise_ = this->declare_parameter<double>("baro_noise", 0.8);
    mag_noise_ = this->declare_parameter<double>("mag_noise", 0.05);
    
    // Complementary filter gains
    accel_weight_ = this->declare_parameter<double>("accel_weight", 0.02);
    mag_weight_ = this->declare_parameter<double>("mag_weight", 0.01);
    
    // Reference pressure for barometer (sea level)
    reference_pressure_ = this->declare_parameter<double>("reference_pressure", 101325.0); // Pa
    reference_altitude_ = this->declare_parameter<double>("reference_altitude", 0.0); // m
    
    // Initialize state
    position_x_ = 0.0;
    position_y_ = 0.0;
    position_z_ = 0.0;
    velocity_x_ = 0.0;
    velocity_y_ = 0.0;
    velocity_z_ = 0.0;
    orientation_w_ = 1.0;
    orientation_x_ = 0.0;
    orientation_y_ = 0.0;
    orientation_z_ = 0.0;
    
    // Initialize sensor states
    last_imu_time_ = this->now();
    have_imu_ = false;
    have_gps_ = false;
    have_barometer_ = false;
    have_magnetometer_ = false;
    
    // GPS home position (set on first fix)
    home_latitude_ = 0.0;
    home_longitude_ = 0.0;
    home_altitude_ = 0.0;
    home_set_ = false;
    
    // Earth radius for GPS conversion (WGS84)
    earth_radius_ = 6378137.0; // meters
    
    // Subscriptions
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", 50, std::bind(&EkfNode::imuCb, this, std::placeholders::_1));
    
    if (use_gps_) {
      gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/gps/fix", 10, std::bind(&EkfNode::gpsCb, this, std::placeholders::_1));
    }
    
    if (use_barometer_) {
      baro_sub_ = this->create_subscription<sensor_msgs::msg::FluidPressure>(
        "/barometer/pressure", 10, std::bind(&EkfNode::baroCb, this, std::placeholders::_1));
    }
    
    if (use_magnetometer_) {
      mag_sub_ = this->create_subscription<sensor_msgs::msg::MagneticField>(
        "/magnetometer/data", 10, std::bind(&EkfNode::magCb, this, std::placeholders::_1));
    }
    
    // Publisher
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    
    // Timer for periodic odometry publication
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000 / publish_rate_hz_),
      std::bind(&EkfNode::publishOdom, this));
    
    RCLCPP_INFO(this->get_logger(), "Enhanced EKF Node initialized (rate: %d Hz)", publish_rate_hz_);
  }

private:
  // ========== Sensor Callbacks ==========
  void imuCb(const sensor_msgs::msg::Imu::SharedPtr msg) {
    if (!have_imu_) {
      have_imu_ = true;
      last_imu_time_ = this->now();
      RCLCPP_INFO(this->get_logger(), "First IMU message received");
      
      // Initialize orientation from IMU if available
      if (msg->orientation.w != 0.0 || msg->orientation.x != 0.0 || 
          msg->orientation.y != 0.0 || msg->orientation.z != 0.0) {
        orientation_w_ = msg->orientation.w;
        orientation_x_ = msg->orientation.x;
        orientation_y_ = msg->orientation.y;
        orientation_z_ = msg->orientation.z;
        normalizeQuaternion();
      }
      return;
    }
    
    // Calculate time delta
    rclcpp::Time current_time = this->now();
    double dt = (current_time - last_imu_time_).seconds();
    if (dt <= 0.0 || dt > 0.1) {  // Skip if invalid or too large
      last_imu_time_ = current_time;
      return;
    }
    
    // ========== Quaternion Integration (Proper 3D) ==========
    // Integrate angular velocity to update orientation
    double wx = msg->angular_velocity.x;
    double wy = msg->angular_velocity.y;
    double wz = msg->angular_velocity.z;
    
    // Quaternion derivative: q_dot = 0.5 * q * [0, wx, wy, wz]
    double qw = orientation_w_;
    double qx = orientation_x_;
    double qy = orientation_y_;
    double qz = orientation_z_;
    
    // Quaternion multiplication: q * omega_quat
    double qw_dot = 0.5 * (-qx * wx - qy * wy - qz * wz);
    double qx_dot = 0.5 * (qw * wx + qy * wz - qz * wy);
    double qy_dot = 0.5 * (qw * wy - qx * wz + qz * wx);
    double qz_dot = 0.5 * (qw * wz + qx * wy - qy * wx);
    
    // Euler integration
    orientation_w_ += qw_dot * dt;
    orientation_x_ += qx_dot * dt;
    orientation_y_ += qy_dot * dt;
    orientation_z_ += qz_dot * dt;
    
    // Normalize quaternion
    normalizeQuaternion();
    
    // ========== Complementary Filter for Attitude ==========
    // Use accelerometer and magnetometer to correct drift
    if (msg->linear_acceleration.x != 0.0 || msg->linear_acceleration.y != 0.0 || 
        msg->linear_acceleration.z != 0.0) {
      // Estimate attitude from accelerometer (pitch and roll)
      double ax = msg->linear_acceleration.x;
      double ay = msg->linear_acceleration.y;
      double az = msg->linear_acceleration.z;
      
      // Normalize accelerometer
      double accel_norm = std::sqrt(ax*ax + ay*ay + az*az);
      if (accel_norm > 0.1) {
        ax /= accel_norm;
        ay /= accel_norm;
        az /= accel_norm;
        
        // Estimate roll and pitch from accelerometer
        double roll_accel = std::atan2(ay, az);
        double pitch_accel = std::asin(-ax);
        
        // Get current roll and pitch from quaternion
        double roll_current, pitch_current, yaw_current;
        quaternionToEuler(roll_current, pitch_current, yaw_current);
        
        // Complementary filter: blend gyro integration with accelerometer
        double roll_fused = (1.0 - accel_weight_) * roll_current + accel_weight_ * roll_accel;
        double pitch_fused = (1.0 - accel_weight_) * pitch_current + accel_weight_ * pitch_accel;
        
        // Convert back to quaternion (keeping yaw from gyro)
        eulerToQuaternion(roll_fused, pitch_fused, yaw_current);
      }
    }
    
    // ========== Velocity and Position Update ==========
    // Rotate acceleration from body frame to world frame
    double accel_body_x = msg->linear_acceleration.x;
    double accel_body_y = msg->linear_acceleration.y;
    double accel_body_z = msg->linear_acceleration.z - 9.81; // Remove gravity
    
    // Rotate to world frame using quaternion
    double accel_world_x, accel_world_y, accel_world_z;
    rotateVectorBodyToWorld(accel_body_x, accel_body_y, accel_body_z,
                           accel_world_x, accel_world_y, accel_world_z);
    
    // Integrate acceleration to velocity
    velocity_x_ += accel_world_x * dt;
    velocity_y_ += accel_world_y * dt;
    velocity_z_ += accel_world_z * dt;
    
    // Apply velocity damping to prevent drift
    double vel_damping = 0.95;
    velocity_x_ *= vel_damping;
    velocity_y_ *= vel_damping;
    velocity_z_ *= vel_damping;
    
    // Integrate velocity to position
    position_x_ += velocity_x_ * dt;
    position_y_ += velocity_y_ * dt;
    position_z_ += velocity_z_ * dt;
    
    // Store angular velocity
    angular_velocity_x_ = wx;
    angular_velocity_y_ = wy;
    angular_velocity_z_ = wz;
    
    last_imu_time_ = current_time;
  }
  
  void gpsCb(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    if (msg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
      return; // No valid GPS fix
    }
    
    // Set home position on first valid GPS fix
    if (!home_set_) {
      home_latitude_ = msg->latitude;
      home_longitude_ = msg->longitude;
      home_altitude_ = msg->altitude;
      home_set_ = true;
      RCLCPP_INFO(this->get_logger(), 
        "GPS home set: lat=%.6f, lon=%.6f, alt=%.1f",
        home_latitude_, home_longitude_, home_altitude_);
      return;
    }
    
    have_gps_ = true;
    
    // Convert GPS to local NED frame (relative to home)
    double lat = msg->latitude;
    double lon = msg->longitude;
    double alt = msg->altitude;
    
    // Simple flat-earth approximation (good for small distances)
    double dlat = lat - home_latitude_;
    double dlon = lon - home_longitude_;
    double dalt = alt - home_altitude_;
    
    // Convert to meters
    double gps_x = dlon * earth_radius_ * std::cos(home_latitude_ * M_PI / 180.0) * M_PI / 180.0;
    double gps_y = dlat * earth_radius_ * M_PI / 180.0;
    double gps_z = dalt;
    
    // Fuse GPS with IMU estimate using complementary filter
    double gps_weight = 0.1; // Trust GPS less (it's noisy)
    position_x_ = (1.0 - gps_weight) * position_x_ + gps_weight * gps_x;
    position_y_ = (1.0 - gps_weight) * position_y_ + gps_weight * gps_y;
    
    // Use GPS altitude if barometer not available
    if (!have_barometer_) {
      position_z_ = (1.0 - gps_weight) * position_z_ + gps_weight * gps_z;
    }
  }
  
  void baroCb(const sensor_msgs::msg::FluidPressure::SharedPtr msg) {
    have_barometer_ = true;
    
    // Convert pressure to altitude using barometric formula
    // h = 44330 * (1 - (P/P0)^0.1903)
    double pressure = msg->fluid_pressure;
    double pressure_ratio = pressure / reference_pressure_;
    double altitude_baro = 44330.0 * (1.0 - std::pow(pressure_ratio, 0.1903));
    altitude_baro += reference_altitude_;
    
    // Fuse with IMU estimate
    double baro_weight = 0.3; // Trust barometer more than GPS for altitude
    position_z_ = (1.0 - baro_weight) * position_z_ + baro_weight * altitude_baro;
  }
  
  void magCb(const sensor_msgs::msg::MagneticField::SharedPtr msg) {
    have_magnetometer_ = true;
    
    // Use magnetometer for yaw correction (if enabled)
    if (use_magnetometer_) {
      double mx = msg->magnetic_field.x;
      double my = msg->magnetic_field.y;
      double mz = msg->magnetic_field.z;
      
      // Estimate yaw from magnetometer
      double yaw_mag = std::atan2(my, mx);
      
      // Get current yaw from quaternion
      double roll, pitch, yaw_current;
      quaternionToEuler(roll, pitch, yaw_current);
      
      // Complementary filter for yaw
      double yaw_fused = (1.0 - mag_weight_) * yaw_current + mag_weight_ * yaw_mag;
      
      // Convert back to quaternion
      eulerToQuaternion(roll, pitch, yaw_fused);
    }
  }
  
  // ========== Helper Functions ==========
  void normalizeQuaternion() {
    double norm = std::sqrt(orientation_w_*orientation_w_ + 
                           orientation_x_*orientation_x_ + 
                           orientation_y_*orientation_y_ + 
                           orientation_z_*orientation_z_);
    if (norm > 0.0001) {
      orientation_w_ /= norm;
      orientation_x_ /= norm;
      orientation_y_ /= norm;
      orientation_z_ /= norm;
    }
  }
  
  void quaternionToEuler(double& roll, double& pitch, double& yaw) {
    double qw = orientation_w_;
    double qx = orientation_x_;
    double qy = orientation_y_;
    double qz = orientation_z_;
    
    // Roll (x-axis rotation)
    double sinr_cosp = 2.0 * (qw * qx + qy * qz);
    double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
    roll = std::atan2(sinr_cosp, cosr_cosp);
    
    // Pitch (y-axis rotation)
    double sinp = 2.0 * (qw * qy - qz * qx);
    if (std::abs(sinp) >= 1.0) {
      pitch = std::copysign(M_PI / 2.0, sinp);
    } else {
      pitch = std::asin(sinp);
    }
    
    // Yaw (z-axis rotation)
    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    yaw = std::atan2(siny_cosp, cosy_cosp);
  }
  
  void eulerToQuaternion(double roll, double pitch, double yaw) {
    double cr = std::cos(roll * 0.5);
    double sr = std::sin(roll * 0.5);
    double cp = std::cos(pitch * 0.5);
    double sp = std::sin(pitch * 0.5);
    double cy = std::cos(yaw * 0.5);
    double sy = std::sin(yaw * 0.5);
    
    orientation_w_ = cr * cp * cy + sr * sp * sy;
    orientation_x_ = sr * cp * cy - cr * sp * sy;
    orientation_y_ = cr * sp * cy + sr * cp * sy;
    orientation_z_ = cr * cp * sy - sr * sp * cy;
    
    normalizeQuaternion();
  }
  
  void rotateVectorBodyToWorld(double bx, double by, double bz,
                               double& wx, double& wy, double& wz) {
    double qw = orientation_w_;
    double qx = orientation_x_;
    double qy = orientation_y_;
    double qz = orientation_z_;
    
    // Rotate vector using quaternion: v_world = q * v_body * q*
    // Simplified rotation matrix from quaternion
    double t2 = qw*qx; double t3 = qw*qy; double t4 = qw*qz;
    double t5 = -qx*qx; double t6 = qx*qy; double t7 = qx*qz;
    double t8 = -qy*qy; double t9 = qy*qz; double t10 = -qz*qz;
    
    wx = 2.0 * ((t8 + t10)*bx + (t6 - t4)*by + (t3 + t7)*bz) + bx;
    wy = 2.0 * ((t4 + t6)*bx + (t5 + t10)*by + (t9 - t2)*bz) + by;
    wz = 2.0 * ((t7 - t3)*bx + (t2 + t9)*by + (t5 + t8)*bz) + bz;
  }
  
  void publishOdom() {
    if (!have_imu_) {
      return; // Don't publish until we have IMU data
    }
    
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = this->now();
    odom.header.frame_id = world_frame_;
    odom.child_frame_id = base_frame_;
    
    // Position
    odom.pose.pose.position.x = position_x_;
    odom.pose.pose.position.y = position_y_;
    odom.pose.pose.position.z = position_z_;
    
    // Orientation
    odom.pose.pose.orientation.w = orientation_w_;
    odom.pose.pose.orientation.x = orientation_x_;
    odom.pose.pose.orientation.y = orientation_y_;
    odom.pose.pose.orientation.z = orientation_z_;
    
    // Linear velocity
    odom.twist.twist.linear.x = velocity_x_;
    odom.twist.twist.linear.y = velocity_y_;
    odom.twist.twist.linear.z = velocity_z_;
    
    // Angular velocity
    odom.twist.twist.angular.x = angular_velocity_x_;
    odom.twist.twist.angular.y = angular_velocity_y_;
    odom.twist.twist.angular.z = angular_velocity_z_;
    
    // Covariance matrices (adaptive based on sensor availability)
    // Pose covariance (6x6: x, y, z, roll, pitch, yaw)
    double pos_cov = have_gps_ ? gps_noise_xy_ : 10.0; // Higher uncertainty without GPS
    double alt_cov = have_barometer_ ? baro_noise_ : (have_gps_ ? gps_noise_z_ : 10.0);
    double att_cov = 0.05;
    
    odom.pose.covariance[0] = pos_cov * pos_cov;   // x
    odom.pose.covariance[7] = pos_cov * pos_cov;    // y
    odom.pose.covariance[14] = alt_cov * alt_cov;   // z
    odom.pose.covariance[21] = att_cov * att_cov;   // roll
    odom.pose.covariance[28] = att_cov * att_cov;   // pitch
    odom.pose.covariance[35] = have_magnetometer_ ? (mag_noise_ * mag_noise_) : (att_cov * att_cov); // yaw
    
    // Twist covariance (6x6: vx, vy, vz, wx, wy, wz)
    double vel_cov = imu_noise_accel_;
    double ang_vel_cov = imu_noise_gyro_;
    
    odom.twist.covariance[0] = vel_cov * vel_cov;   // vx
    odom.twist.covariance[7] = vel_cov * vel_cov;    // vy
    odom.twist.covariance[14] = vel_cov * vel_cov;   // vz
    odom.twist.covariance[21] = ang_vel_cov * ang_vel_cov; // wx
    odom.twist.covariance[28] = ang_vel_cov * ang_vel_cov; // wy
    odom.twist.covariance[35] = ang_vel_cov * ang_vel_cov; // wz
    
    odom_pub_->publish(odom);
  }
  
  // ========== ROS Objects ==========
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr baro_sub_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  // ========== State Variables ==========
  double position_x_, position_y_, position_z_;
  double velocity_x_, velocity_y_, velocity_z_;
  double orientation_w_, orientation_x_, orientation_y_, orientation_z_;
  double angular_velocity_x_, angular_velocity_y_, angular_velocity_z_;
  
  rclcpp::Time last_imu_time_;
  bool have_imu_, have_gps_, have_barometer_, have_magnetometer_;
  
  // GPS home position
  double home_latitude_, home_longitude_, home_altitude_;
  bool home_set_;
  double earth_radius_;
  
  // ========== Parameters ==========
  int publish_rate_hz_;
  std::string world_frame_, base_frame_;
  bool use_gps_, use_barometer_, use_magnetometer_;
  double imu_noise_accel_, imu_noise_gyro_;
  double gps_noise_xy_, gps_noise_z_;
  double baro_noise_, mag_noise_;
  double accel_weight_, mag_weight_;
  double reference_pressure_, reference_altitude_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EkfNode>());
  rclcpp::shutdown();
  return 0;
}
