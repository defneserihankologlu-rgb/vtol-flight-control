#!/usr/bin/env python3
"""
Test script to publish fake sensor data for safety monitor testing.
This allows the safety checks to pass in simulation.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from sensor_msgs.msg import NavSatFix, NavSatStatus
import time

class SafetySensorPublisher(Node):
    def __init__(self):
        super().__init__('safety_sensor_publisher')
        
        # Publishers
        self.gps_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.battery_pub = self.create_publisher(Float32, '/battery/voltage', 10)
        self.rc_link_pub = self.create_publisher(Bool, '/rc/link_ok', 10)
        
        # Timer to publish at 10 Hz
        self.create_timer(0.1, self.publish_data)
        
        # Initialize GPS fix
        self.gps_fix_count = 0
        
        self.get_logger().info("🚀 Safety Sensor Publisher started")
        self.get_logger().info("📡 Publishing:")
        self.get_logger().info("   - /gps/fix (GPS data)")
        self.get_logger().info("   - /battery/voltage (12.6V)")
        self.get_logger().info("   - /rc/link_ok (True)")
    
    def publish_data(self):
        # Publish GPS fix
        gps_msg = NavSatFix()
        gps_msg.header.stamp = self.get_clock().now().to_msg()
        gps_msg.header.frame_id = "gps"
        
        # Simulate GPS fix (STATUS_FIX = 2 means 2D fix)
        gps_msg.status.status = NavSatStatus.STATUS_FIX
        gps_msg.status.service = NavSatStatus.SERVICE_GPS
        
        # Set a fake location (Istanbul, Turkey)
        gps_msg.latitude = 41.0082
        gps_msg.longitude = 28.9784
        gps_msg.altitude = 100.0
        
        # Set position covariance (good accuracy: 2m)
        gps_msg.position_covariance[0] = 2.0 * 2.0  # x variance (m^2)
        gps_msg.position_covariance[4] = 2.0 * 2.0  # y variance (m^2)
        gps_msg.position_covariance[8] = 4.0 * 4.0  # z variance (m^2)
        # Use COVARIANCE_TYPE_APPROXIMATED constant
        gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        
        self.gps_pub.publish(gps_msg)
        
        # Publish battery voltage (fully charged 3S LiPo)
        battery_msg = Float32()
        battery_msg.data = 12.6  # Volts
        self.battery_pub.publish(battery_msg)
        
        # Publish RC link status (OK)
        rc_msg = Bool()
        rc_msg.data = True
        self.rc_link_pub.publish(rc_msg)
        
        # Log first few messages
        self.gps_fix_count += 1
        if self.gps_fix_count <= 3:
            self.get_logger().info(f"✅ Published sensor data (count: {self.gps_fix_count})")

def main():
    rclpy.init()
    node = SafetySensorPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("🛑 Shutting down safety sensor publisher")
        rclpy.shutdown()

if __name__ == '__main__':
    main()

