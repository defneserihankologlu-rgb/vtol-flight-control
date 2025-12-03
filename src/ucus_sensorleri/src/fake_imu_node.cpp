#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <cmath>
using namespace std::chrono_literals;

class FakeImuNode : public rclcpp::Node
{
public:
  FakeImuNode() : Node("fake_imu_node")
  {
    // Parameters
    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 100.0);
    noise_level_ = this->declare_parameter<double>("noise_level", 0.01);
    
    pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
    
    // Timer for publishing
    int period_ms = static_cast<int>(1000.0 / publish_rate_hz_);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&FakeImuNode::on_timer, this));
    
    // Initialize state
    time_elapsed_ = 0.0;
    
    RCLCPP_INFO(this->get_logger(), 
      "Fake IMU Node initialized (rate: %.1f Hz, noise: %.3f)", 
      publish_rate_hz_, noise_level_);
  }

private:
  void on_timer()
  {
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link";
    
    // Update time
    double dt = 1.0 / publish_rate_hz_;
    time_elapsed_ += dt;
    
    // ========== Linear Acceleration ==========
    // Gravity + small noise
    msg.linear_acceleration.x = 0.0 + (rand() % 100 - 50) / 5000.0 * noise_level_;
    msg.linear_acceleration.y = 0.0 + (rand() % 100 - 50) / 5000.0 * noise_level_;
    msg.linear_acceleration.z = 9.81 + (rand() % 100 - 50) / 5000.0 * noise_level_;
    
    // ========== Angular Velocity ==========
    // Small oscillations or zero (can be modified for testing)
    msg.angular_velocity.x = 0.0 + (rand() % 100 - 50) / 5000.0 * noise_level_;
    msg.angular_velocity.y = 0.0 + (rand() % 100 - 50) / 5000.0 * noise_level_;
    msg.angular_velocity.z = 0.0 + (rand() % 100 - 50) / 5000.0 * noise_level_;
    
    // Optional: Add sinusoidal motion for testing
    // msg.angular_velocity.z = 0.1 * std::sin(time_elapsed_);
    
    // ========== Orientation ==========
    // Default: identity quaternion (no rotation)
    // Can be modified to simulate attitude changes
    msg.orientation.w = 1.0;
    msg.orientation.x = 0.0;
    msg.orientation.y = 0.0;
    msg.orientation.z = 0.0;
    
    // Optional: Add small orientation noise
    // msg.orientation.w = 1.0 + (rand() % 100 - 50) / 10000.0 * noise_level_;
    // msg.orientation.x = (rand() % 100 - 50) / 10000.0 * noise_level_;
    // msg.orientation.y = (rand() % 100 - 50) / 10000.0 * noise_level_;
    // msg.orientation.z = (rand() % 100 - 50) / 10000.0 * noise_level_;
    
    // ========== Covariance Matrices ==========
    // Linear acceleration covariance (3x3)
    msg.linear_acceleration_covariance[0] = noise_level_ * noise_level_; // x
    msg.linear_acceleration_covariance[4] = noise_level_ * noise_level_; // y
    msg.linear_acceleration_covariance[8] = noise_level_ * noise_level_; // z
    
    // Angular velocity covariance (3x3)
    msg.angular_velocity_covariance[0] = noise_level_ * noise_level_; // x
    msg.angular_velocity_covariance[4] = noise_level_ * noise_level_; // y
    msg.angular_velocity_covariance[8] = noise_level_ * noise_level_; // z
    
    // Orientation covariance (3x3: roll, pitch, yaw)
    msg.orientation_covariance[0] = noise_level_ * noise_level_; // roll
    msg.orientation_covariance[4] = noise_level_ * noise_level_; // pitch
    msg.orientation_covariance[8] = noise_level_ * noise_level_; // yaw
    
    // Mark other covariance elements as unknown (-1 means unknown)
    for (int i = 0; i < 9; i++) {
      if (msg.linear_acceleration_covariance[i] == 0.0) {
        msg.linear_acceleration_covariance[i] = -1.0;
      }
      if (msg.angular_velocity_covariance[i] == 0.0) {
        msg.angular_velocity_covariance[i] = -1.0;
      }
      if (msg.orientation_covariance[i] == 0.0) {
        msg.orientation_covariance[i] = -1.0;
      }
    }
    
    pub_->publish(msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  // State
  double time_elapsed_;
  double publish_rate_hz_;
  double noise_level_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeImuNode>());
  rclcpp::shutdown();
  return 0;
}
