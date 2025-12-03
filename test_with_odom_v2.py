#!/usr/bin/env python3
"""Test mode manager with continuous odometry - improved version"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8
from ucus_msgs.msg import FlightMode
from nav_msgs.msg import Odometry
import time

class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')
        self.mode_cmd_pub = self.create_publisher(UInt8, '/mode_cmd', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.mode_sub = self.create_subscription(
            FlightMode, '/flight_mode', 
            self.mode_callback, 10)
        self.current_mode = None
        
        # Use ROS 2 timer for odometry publishing (more reliable than thread)
        self.create_timer(0.1, self.publish_odom)  # 10 Hz
        
        self.get_logger().info("Test node started, publishing odometry continuously at 10 Hz")
    
    def mode_callback(self, msg):
        mode_names = {0: "MC", 1: "VTOL_TAKEOFF", 2: "HOVER", 3: "TRANSITION_MC_TO_FW",
                      4: "FW", 5: "TRANSITION_FW_TO_MC", 6: "LAND", 7: "EMERGENCY"}
        if self.current_mode != msg.mode:
            self.get_logger().info(
                f"🔄 Mode: {mode_names.get(msg.mode, 'UNKNOWN')} (reason: {msg.reason})")
        self.current_mode = msg.mode
    
    def publish_odom(self):
        """Publish odometry - called by timer"""
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.z = 25.0  # 25m altitude
        odom.pose.pose.orientation.w = 1.0
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        self.odom_pub.publish(odom)
    
    def send_mode(self, mode):
        msg = UInt8()
        msg.data = mode
        self.mode_cmd_pub.publish(msg)
        mode_names = {0: "MC", 1: "VTOL_TAKEOFF", 2: "HOVER", 3: "TRANSITION_MC_TO_FW",
                      4: "FW", 5: "TRANSITION_FW_TO_MC", 6: "LAND", 7: "EMERGENCY"}
        self.get_logger().info(f"📤 Sent mode command: {mode_names.get(mode, 'UNKNOWN')}")

def main():
    rclpy.init()
    node = TestNode()
    
    print("\n" + "="*60)
    print("Mode Manager Test - Interactive (v2)")
    print("="*60)
    print("Odometry is being published continuously (25m altitude, 10 Hz)")
    print("Commands: 0=MC, 1=TAKEOFF, 2=HOVER, 3=TRANS_MC_FW, 4=FW,")
    print("          5=TRANS_FW_MC, 6=LAND, 7=EMERGENCY, q=quit")
    print("="*60 + "\n")
    
    # Wait for mode manager
    time.sleep(2)
    
    # Use a separate thread for user input so ROS spinning continues
    import threading
    
    def user_input_thread():
        try:
            while True:
                choice = input("Enter mode (0-7) or 'q' to quit: ").strip()
                if choice == 'q':
                    rclpy.shutdown()
                    break
                try:
                    mode = int(choice)
                    if 0 <= mode <= 7:
                        node.send_mode(mode)
                    else:
                        print("Mode must be 0-7")
                except ValueError:
                    print("Invalid input")
        except (EOFError, KeyboardInterrupt):
            rclpy.shutdown()
    
    input_thread = threading.Thread(target=user_input_thread, daemon=True)
    input_thread.start()
    
    try:
        # Keep spinning to maintain odometry publishing and process callbacks
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

