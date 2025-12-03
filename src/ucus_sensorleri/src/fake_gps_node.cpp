#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include <cmath>
#include <cstdlib>
using namespace std::chrono_literals;

class FakeGpsNode : public rclcpp::Node
{
public:
  FakeGpsNode() : Node("fake_gps_node")
  {
    // Parameters
    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 10.0);
    noise_level_ = this->declare_parameter<double>("noise_level", 1.5); // meters
    altitude_noise_ = this->declare_parameter<double>("altitude_noise", 3.0); // meters
    
    // Home position (can be set via parameters or defaults to Istanbul, Turkey)
    home_latitude_ = this->declare_parameter<double>("home_latitude", 41.0082); // Istanbul
    home_longitude_ = this->declare_parameter<double>("home_longitude", 28.9784);
    home_altitude_ = this->declare_parameter<double>("home_altitude", 0.0); // meters
    
    // Simulated position offset (for testing movement)
    position_offset_x_ = this->declare_parameter<double>("position_offset_x", 0.0); // meters
    position_offset_y_ = this->declare_parameter<double>("position_offset_y", 0.0); // meters
    position_offset_z_ = this->declare_parameter<double>("position_offset_z", 0.0); // meters
    
    // Earth radius for conversion (WGS84)
    earth_radius_ = 6378137.0; // meters
    
    pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/gps/fix", 10);
    
    // Timer for publishing
    int period_ms = static_cast<int>(1000.0 / publish_rate_hz_);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&FakeGpsNode::on_timer, this));
    
    // Initialize
    time_elapsed_ = 0.0;
    
    RCLCPP_INFO(this->get_logger(), 
      "Fake GPS Node initialized (rate: %.1f Hz, home: %.6f, %.6f)", 
      publish_rate_hz_, home_latitude_, home_longitude_);
  }

private:
  void on_timer()
  {
    sensor_msgs::msg::NavSatFix msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "gps";
    
    // Update time
    double dt = 1.0 / publish_rate_hz_;
    time_elapsed_ += dt;
    
    // Convert position offset (meters) to lat/lon
    // Simple flat-earth approximation (good for small distances)
    double lat_offset = position_offset_y_ / earth_radius_ * 180.0 / M_PI;
    double lon_offset = position_offset_x_ / (earth_radius_ * std::cos(home_latitude_ * M_PI / 180.0)) * 180.0 / M_PI;
    
    // Add noise
    double lat_noise = (rand() % 100 - 50) / 50.0 * noise_level_ / 111320.0; // ~111320 m per degree latitude
    double lon_noise = (rand() % 100 - 50) / 50.0 * noise_level_ / (111320.0 * std::cos(home_latitude_ * M_PI / 180.0));
    double alt_noise = (rand() % 100 - 50) / 50.0 * altitude_noise_;
    
    // Calculate position
    msg.latitude = home_latitude_ + lat_offset + lat_noise;
    msg.longitude = home_longitude_ + lon_offset + lon_noise;
    msg.altitude = home_altitude_ + position_offset_z_ + alt_noise;
    
    // Status: Good fix
    msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    
    // Position covariance (3x3 matrix, row-major)
    // [xx, xy, xz, yx, yy, yz, zx, zy, zz]
    double pos_std = noise_level_;
    double alt_std = altitude_noise_;
    msg.position_covariance[0] = pos_std * pos_std; // xx
    msg.position_covariance[4] = pos_std * pos_std; // yy
    msg.position_covariance[8] = alt_std * alt_std; // zz
    msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    
    pub_->publish(msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  // State
  double time_elapsed_;
  double publish_rate_hz_;
  double noise_level_;
  double altitude_noise_;
  double home_latitude_;
  double home_longitude_;
  double home_altitude_;
  double position_offset_x_;
  double position_offset_y_;
  double position_offset_z_;
  double earth_radius_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeGpsNode>());
  rclcpp::shutdown();
  return 0;
}

