#!/usr/bin/env python3
"""
VTOL Ground Control Station (GCS) Monitor
Simple text-based monitoring interface for VTOL system status
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from ucus_msgs.msg import FlightMode, ActuatorCmd, SafetyStatus, ArmingStatus
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from geometry_msgs.msg import Quaternion
import math
import os
import sys


class VtolGCSMonitor(Node):
    def __init__(self):
        super().__init__('vtol_gcs_monitor')
        
        # State variables
        self.odom = None
        self.flight_mode = None
        self.actuator_cmd = None
        self.safety_status = None
        self.arming_status = None
        self.armed = False
        self.imu = None
        
        # QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscriptions
        self.create_subscription(Odometry, '/odom', self._odom_cb, qos_profile)
        self.create_subscription(FlightMode, '/flight_mode', self._flight_mode_cb, qos_profile)
        self.create_subscription(ActuatorCmd, '/actuator_cmd', self._actuator_cb, qos_profile)
        self.create_subscription(SafetyStatus, '/safety/status', self._safety_cb, qos_profile)
        self.create_subscription(ArmingStatus, '/safety/arming_status', self._arming_cb, qos_profile)
        self.create_subscription(Bool, '/armed', self._armed_cb, qos_profile)
        self.create_subscription(Imu, '/imu/data', self._imu_cb, qos_profile)
        
        # Display timer
        self.create_timer(0.5, self._display_status)  # 2 Hz update rate
        
        self.get_logger().info("🖥️  VTOL GCS Monitor started")
        self.get_logger().info("   Press Ctrl+C to exit")
        
        # Clear screen
        os.system('clear' if os.name != 'nt' else 'cls')
    
    def _odom_cb(self, msg):
        self.odom = msg
    
    def _flight_mode_cb(self, msg):
        self.flight_mode = msg
    
    def _actuator_cb(self, msg):
        self.actuator_cmd = msg
    
    def _safety_cb(self, msg):
        self.safety_status = msg
    
    def _arming_cb(self, msg):
        self.arming_status = msg
    
    def _armed_cb(self, msg):
        self.armed = msg.data
    
    def _imu_cb(self, msg):
        self.imu = msg
    
    def _quaternion_to_euler(self, q):
        """Convert quaternion to Euler angles (roll, pitch, yaw)"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)
    
    def _get_mode_name(self, mode):
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
        return mode_names.get(mode, f"UNKNOWN({mode})")
    
    def _get_reason_name(self, reason):
        reason_names = {
            0: "MANUAL",
            1: "MISSION",
            2: "FAILSAFE"
        }
        return reason_names.get(reason, f"UNKNOWN({reason})")
    
    def _display_status(self):
        """Display current system status"""
        # Clear screen and move cursor to top
        os.system('clear' if os.name != 'nt' else 'cls')
        
        print("=" * 80)
        print(" " * 25 + "VTOL GROUND CONTROL STATION")
        print("=" * 80)
        print()
        
        # Flight Mode Section
        print("📊 FLIGHT MODE")
        print("-" * 80)
        if self.flight_mode:
            mode_name = self._get_mode_name(self.flight_mode.mode)
            reason_name = self._get_reason_name(self.flight_mode.reason)
            print(f"  Mode:     {mode_name:20s}  Reason: {reason_name}")
        else:
            print("  Mode:     NO DATA")
        print()
        
        # Position & Attitude Section
        print("📍 POSITION & ATTITUDE")
        print("-" * 80)
        if self.odom:
            pos = self.odom.pose.pose.position
            vel = self.odom.twist.twist.linear
            q = self.odom.pose.pose.orientation
            roll, pitch, yaw = self._quaternion_to_euler(q)
            
            print(f"  Position:  X: {pos.x:7.2f} m  Y: {pos.y:7.2f} m  Z: {pos.z:7.2f} m")
            print(f"  Velocity:  Vx: {vel.x:7.2f} m/s  Vy: {vel.y:7.2f} m/s  Vz: {vel.z:7.2f} m/s")
            print(f"  Attitude:  Roll: {roll:6.1f}°  Pitch: {pitch:6.1f}°  Yaw: {yaw:6.1f}°")
        else:
            print("  Position:  NO DATA")
        print()
        
        # Safety & Arming Section
        print("🛡️  SAFETY & ARMING")
        print("-" * 80)
        armed_status = "ARMED" if self.armed else "DISARMED"
        armed_color = "\033[91m" if self.armed else "\033[92m"  # Red if armed, green if disarmed
        reset_color = "\033[0m"
        print(f"  Status:    {armed_color}{armed_status}{reset_color}")
        
        if self.arming_status:
            can_arm = "YES" if self.arming_status.can_arm else "NO"
            print(f"  Can Arm:   {can_arm}")
            print(f"  Reason:    {self.arming_status.reason}")
        
        if self.safety_status:
            print(f"  IMU OK:    {'YES' if self.safety_status.imu_ok else 'NO'}")
            print(f"  GPS OK:    {'YES' if self.safety_status.gps_ok else 'NO'}")
            print(f"  Battery:   {self.safety_status.battery_voltage:.2f} V")
            print(f"  Link OK:   {'YES' if self.safety_status.link_ok else 'NO'}")
        print()
        
        # Actuator Commands Section
        print("⚙️  ACTUATOR COMMANDS")
        print("-" * 80)
        if self.actuator_cmd and len(self.actuator_cmd.outputs) > 0:
            outputs = self.actuator_cmd.outputs
            if len(outputs) >= 4:
                print(f"  Motors:    M1: {outputs[0]:5.3f}  M2: {outputs[1]:5.3f}  M3: {outputs[2]:5.3f}  M4: {outputs[3]:5.3f}")
            if len(outputs) >= 7:
                print(f"  Surfaces:  Ail: {outputs[4]:5.3f}  Elv: {outputs[5]:5.3f}  Rud: {outputs[6]:5.3f}")
            if len(outputs) >= 8:
                print(f"  Throttle:  {outputs[7]:5.3f}")
        else:
            print("  Actuators: NO DATA")
        print()
        
        # IMU Section
        print("📡 IMU DATA")
        print("-" * 80)
        if self.imu:
            accel = self.imu.linear_acceleration
            gyro = self.imu.angular_velocity
            print(f"  Accel:     X: {accel.x:7.3f}  Y: {accel.y:7.3f}  Z: {accel.z:7.3f} m/s²")
            print(f"  Gyro:      X: {gyro.x:7.3f}  Y: {gyro.y:7.3f}  Z: {gyro.z:7.3f} rad/s")
        else:
            print("  IMU:       NO DATA")
        print()
        
        print("=" * 80)
        print("  Press Ctrl+C to exit")
        print("=" * 80)


def main(args=None):
    rclpy.init(args=args)
    node = VtolGCSMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nShutting down GCS Monitor...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

