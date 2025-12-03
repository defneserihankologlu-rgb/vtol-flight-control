#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include <cmath>
#include <cstdlib>
using namespace std::chrono_literals;

class FakeBarometerNode : public rclcpp::Node
{
public:
  FakeBarometerNode() : Node("fake_barometer_node")
  {
    // Parameters
    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 50.0);
    noise_level_ = this->declare_parameter<double>("noise_level", 100.0); // Pascals
    
    // Reference pressure (sea level, standard atmosphere)
    reference_pressure_ = this->declare_parameter<double>("reference_pressure", 101325.0); // Pa
    
    // Current altitude (can be changed to simulate altitude changes)
    current_altitude_ = this->declare_parameter<double>("current_altitude", 0.0); // meters
    
    // Atmospheric constants
    // Pressure decreases with altitude: P = P0 * exp(-g*M*h/(R*T))
    // Simplified: P = P0 * (1 - h/44330) for small altitudes
    // Or: P = P0 * exp(-h/8400) for more accurate
    sea_level_pressure_ = reference_pressure_;
    
    pub_ = this->create_publisher<sensor_msgs::msg::FluidPressure>("/barometer/pressure", 10);
    
    // Timer for publishing
    int period_ms = static_cast<int>(1000.0 / publish_rate_hz_);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&FakeBarometerNode::on_timer, this));
    
    // Initialize
    time_elapsed_ = 0.0;
    
    RCLCPP_INFO(this->get_logger(), 
      "Fake Barometer Node initialized (rate: %.1f Hz, ref_pressure: %.1f Pa)", 
      publish_rate_hz_, reference_pressure_);
  }

private:
  void on_timer()
  {
    sensor_msgs::msg::FluidPressure msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "barometer";
    
    // Update time
    double dt = 1.0 / publish_rate_hz_;
    time_elapsed_ += dt;
    
    // Calculate pressure from altitude using barometric formula
    // P = P0 * exp(-g*M*h/(R*T))
    // Simplified: P = P0 * (1 - h/44330) for altitudes < 11 km
    // Or more accurate: P = P0 * exp(-h/8400)
    double altitude_m = current_altitude_;
    double pressure = sea_level_pressure_ * std::exp(-altitude_m / 8400.0);
    
    // Add noise
    double noise = (rand() % 100 - 50) / 50.0 * noise_level_;
    msg.fluid_pressure = pressure + noise;
    
    // Variance (uncertainty in pressure measurement)
    msg.variance = noise_level_ * noise_level_;
    
    pub_->publish(msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  // State
  double time_elapsed_;
  double publish_rate_hz_;
  double noise_level_;
  double reference_pressure_;
  double sea_level_pressure_;
  double current_altitude_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeBarometerNode>());
  rclcpp::shutdown();
  return 0;
}

