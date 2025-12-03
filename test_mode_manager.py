#!/usr/bin/env python3
"""
Interactive test script for VTOL Mode Manager
Allows testing mode transitions and monitoring state changes
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8
from ucus_msgs.msg import FlightMode
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Point, Twist, Vector3, Pose, PoseWithCovariance, TwistWithCovariance
import time
import math

class ModeManagerTester(Node):
    def __init__(self):
        super().__init__('mode_manager_tester')
        
        # Subscriptions
        self.mode_sub = self.create_subscription(
            FlightMode,
            '/flight_mode',
            self.mode_callback,
            10
        )
        
        # Publishers
        self.mode_cmd_pub = self.create_publisher(UInt8, '/mode_cmd', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # State
        self.current_mode = None
        self.current_reason = None
        
        # Simulated odometry
        self.sim_altitude = 0.0
        self.sim_airspeed = 0.0
        self.sim_pitch = 0.0
        
        # Create timer for simulated odometry
        self.create_timer(0.1, self.publish_odom)  # 10 Hz
        
        self.get_logger().info("Mode Manager Tester started")
        self.get_logger().info("Publishing simulated odometry at 10 Hz")
        
    def mode_callback(self, msg):
        mode_names = {
            0: "MC",
            1: "VTOL_TAKEOFF",
            2: "HOVER",
            3: "TRANSITION_MC_TO_FW",
            4: "FW",
            5: "TRANSITION_FW_TO_MC",
            6: "LAND",
            7: "EMERGENCY"
        }
        reason_names = {
            0: "MANUAL",
            1: "MISSION",
            2: "FAILSAFE"
        }
        
        if self.current_mode != msg.mode:
            self.get_logger().info(
                f"🔄 Mode changed: {mode_names.get(self.current_mode, 'UNKNOWN')} → "
                f"{mode_names.get(msg.mode, 'UNKNOWN')} "
                f"(reason: {reason_names.get(msg.reason, 'UNKNOWN')})"
            )
        
        self.current_mode = msg.mode
        self.current_reason = msg.reason
        
        # Display current state
        mode_name = mode_names.get(msg.mode, f"UNKNOWN({msg.mode})")
        reason_name = reason_names.get(msg.reason, f"UNKNOWN({msg.reason})")
        self.get_logger().info(
            f"📊 Current: {mode_name} | Reason: {reason_name} | "
            f"Alt: {self.sim_altitude:.1f}m | Airspeed: {self.sim_airspeed:.1f}m/s"
        )
    
    def publish_odom(self):
        """Publish simulated odometry"""
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        
        # Position
        odom.pose.pose.position.x = 0.0
        odom.pose.pose.position.y = 0.0
        odom.pose.pose.position.z = self.sim_altitude
        
        # Orientation (quaternion from pitch)
        q = self.euler_to_quaternion(0.0, self.sim_pitch, 0.0)
        odom.pose.pose.orientation = q
        
        # Velocity
        odom.twist.twist.linear.x = self.sim_airspeed
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        
        self.odom_pub.publish(odom)
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion"""
        q = Quaternion()
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q
    
    def send_mode_cmd(self, mode):
        """Send a mode command"""
        msg = UInt8()
        msg.data = mode
        self.mode_cmd_pub.publish(msg)
        mode_names = {
            0: "MC", 1: "VTOL_TAKEOFF", 2: "HOVER", 3: "TRANSITION_MC_TO_FW",
            4: "FW", 5: "TRANSITION_FW_TO_MC", 6: "LAND", 7: "EMERGENCY"
        }
        self.get_logger().info(f"📤 Sent mode command: {mode_names.get(mode, f'UNKNOWN({mode})')}")
    
    def simulate_takeoff(self):
        """Simulate takeoff by increasing altitude"""
        self.get_logger().info("🚁 Simulating takeoff - increasing altitude...")
        for alt in range(0, 35, 2):
            self.sim_altitude = float(alt)
            time.sleep(0.5)
            rclpy.spin_once(self, timeout_sec=0.1)
    
    def simulate_transition_mc_to_fw(self):
        """Simulate MC to FW transition"""
        self.get_logger().info("✈️ Simulating MC→FW transition - increasing airspeed and pitch...")
        # Increase airspeed and pitch gradually
        for i in range(15):
            self.sim_airspeed = min(15.0, i * 1.0)
            self.sim_pitch = min(0.3, i * 0.02)
            time.sleep(0.5)
            rclpy.spin_once(self, timeout_sec=0.1)
    
    def simulate_transition_fw_to_mc(self):
        """Simulate FW to MC transition"""
        self.get_logger().info("🛬 Simulating FW→MC transition - decreasing airspeed...")
        # Decrease airspeed gradually
        for i in range(15, 0, -1):
            self.sim_airspeed = max(0.0, i * 1.0)
            self.sim_pitch = max(0.0, i * 0.02)
            time.sleep(0.5)
            rclpy.spin_once(self, timeout_sec=0.1)
    
    def simulate_landing(self):
        """Simulate landing by decreasing altitude"""
        self.get_logger().info("🛬 Simulating landing - decreasing altitude...")
        start_alt = self.sim_altitude
        for alt in range(int(start_alt), -1, -1):
            self.sim_altitude = float(max(0.0, alt))
            self.sim_airspeed = max(0.0, self.sim_airspeed - 0.5)
            time.sleep(0.5)
            rclpy.spin_once(self, timeout_sec=0.1)


def print_menu():
    print("\n" + "="*60)
    print("VTOL Mode Manager Test Menu")
    print("="*60)
    print("0. MC (Multicopter)")
    print("1. VTOL_TAKEOFF")
    print("2. HOVER")
    print("3. TRANSITION_MC_TO_FW")
    print("4. FW (Fixed-Wing)")
    print("5. TRANSITION_FW_TO_MC")
    print("6. LAND")
    print("7. EMERGENCY")
    print("="*60)
    print("s. Simulate Takeoff Sequence")
    print("t. Simulate MC→FW Transition")
    print("r. Simulate FW→MC Transition")
    print("l. Simulate Landing")
    print("a. Set altitude (for testing)")
    print("v. Set airspeed (for testing)")
    print("q. Quit")
    print("="*60)


def main():
    rclpy.init()
    tester = ModeManagerTester()
    
    print("\n✅ Mode Manager Tester initialized")
    print("📡 Listening to /flight_mode")
    print("📤 Publishing to /mode_cmd and /odom")
    print("\nWaiting for mode manager to start...")
    time.sleep(2)
    
    try:
        while True:
            print_menu()
            choice = input("\nEnter choice: ").strip().lower()
            
            if choice == 'q':
                break
            elif choice == 's':
                tester.send_mode_cmd(1)  # VTOL_TAKEOFF
                time.sleep(1)
                tester.simulate_takeoff()
            elif choice == 't':
                tester.send_mode_cmd(3)  # TRANSITION_MC_TO_FW
                time.sleep(1)
                tester.simulate_transition_mc_to_fw()
            elif choice == 'r':
                tester.send_mode_cmd(5)  # TRANSITION_FW_TO_MC
                time.sleep(1)
                tester.simulate_transition_fw_to_mc()
            elif choice == 'l':
                tester.send_mode_cmd(6)  # LAND
                time.sleep(1)
                tester.simulate_landing()
            elif choice == 'a':
                alt = float(input("Enter altitude (m): "))
                tester.sim_altitude = alt
                tester.get_logger().info(f"Set altitude to {alt} m")
            elif choice == 'v':
                airspeed = float(input("Enter airspeed (m/s): "))
                tester.sim_airspeed = airspeed
                tester.get_logger().info(f"Set airspeed to {airspeed} m/s")
            elif choice.isdigit():
                mode = int(choice)
                if 0 <= mode <= 7:
                    tester.send_mode_cmd(mode)
                    time.sleep(0.5)
                else:
                    print("Invalid mode number")
            else:
                print("Invalid choice")
            
            # Spin to process callbacks
            rclpy.spin_once(tester, timeout_sec=0.1)
            
    except KeyboardInterrupt:
        print("\n\nShutting down...")
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

