#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include <cmath>
#include <cstdlib>
using namespace std::chrono_literals;

class FakeMagnetometerNode : public rclcpp::Node
{
public:
  FakeMagnetometerNode() : Node("fake_magnetometer_node")
  {
    // Parameters
    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 50.0);
    noise_level_ = this->declare_parameter<double>("noise_level", 0.01); // Tesla
    
    // Earth's magnetic field strength (typical values in Tesla)
    // At Istanbul, Turkey: ~50,000 nT = 0.00005 T
    magnetic_field_strength_ = this->declare_parameter<double>("magnetic_field_strength", 0.00005); // Tesla (50,000 nT)
    
    // Magnetic field direction (declination and inclination)
    // Istanbul: declination ~6°, inclination ~60°
    declination_ = this->declare_parameter<double>("declination", 6.0 * M_PI / 180.0); // radians
    inclination_ = this->declare_parameter<double>("inclination", 60.0 * M_PI / 180.0); // radians
    
    pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>("/magnetometer/data", 10);
    
    // Timer for publishing
    int period_ms = static_cast<int>(1000.0 / publish_rate_hz_);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&FakeMagnetometerNode::on_timer, this));
    
    // Initialize
    time_elapsed_ = 0.0;
    
    RCLCPP_INFO(this->get_logger(), 
      "Fake Magnetometer Node initialized (rate: %.1f Hz, field: %.6f T)", 
      publish_rate_hz_, magnetic_field_strength_);
  }

private:
  void on_timer()
  {
    sensor_msgs::msg::MagneticField msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "magnetometer";
    
    // Update time
    double dt = 1.0 / publish_rate_hz_;
    time_elapsed_ += dt;
    
    // Calculate Earth's magnetic field components
    // In NED frame (North-East-Down):
    // X = North component = H * cos(inclination)
    // Y = East component = H * sin(declination) * cos(inclination)
    // Z = Down component = H * sin(inclination)
    double H = magnetic_field_strength_;
    
    // North component (X)
    double B_north = H * std::cos(inclination_);
    
    // East component (Y) - includes declination
    double B_east = H * std::sin(declination_) * std::cos(inclination_);
    
    // Down component (Z)
    double B_down = H * std::sin(inclination_);
    
    // Add noise
    double noise_x = (rand() % 100 - 50) / 50.0 * noise_level_;
    double noise_y = (rand() % 100 - 50) / 50.0 * noise_level_;
    double noise_z = (rand() % 100 - 50) / 50.0 * noise_level_;
    
    msg.magnetic_field.x = B_north + noise_x;
    msg.magnetic_field.y = B_east + noise_y;
    msg.magnetic_field.z = B_down + noise_z;
    
    // Magnetic field covariance (3x3 matrix, row-major)
    double field_std = noise_level_;
    msg.magnetic_field_covariance[0] = field_std * field_std; // xx
    msg.magnetic_field_covariance[4] = field_std * field_std; // yy
    msg.magnetic_field_covariance[8] = field_std * field_std; // zz
    // Other elements remain 0 (unknown)
    
    pub_->publish(msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  // State
  double time_elapsed_;
  double publish_rate_hz_;
  double noise_level_;
  double magnetic_field_strength_;
  double declination_;
  double inclination_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeMagnetometerNode>());
  rclcpp::shutdown();
  return 0;
}

