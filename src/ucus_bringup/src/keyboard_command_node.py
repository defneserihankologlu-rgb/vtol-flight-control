#!/usr/bin/env python3
"""
Keyboard Command Node for VTOL System Testing

Reads keyboard input and publishes commands as if from a joystick/GCS.
Useful for testing flight mode switching and system behavior.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8, Bool
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import sys
import select
import termios
import tty
import threading

class KeyboardCommandNode(Node):
    def __init__(self):
        super().__init__('keyboard_command_node')
        
        # Publishers
        self.mode_cmd_pub = self.create_publisher(UInt8, '/mode_cmd', 10)
        self.arm_cmd_pub = self.create_publisher(Bool, '/arm_cmd', 10)
        self.setpoint_pub = self.create_publisher(PoseStamped, '/setpoint/pos', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # State
        self.current_altitude = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        
        # Odometry state (for setting altitude that mode manager sees)
        self.odom_altitude = 0.0
        self.odom_x = 0.0
        self.odom_y = 0.0
        
        # Save terminal settings
        self.old_settings = termios.tcgetattr(sys.stdin)
        
        self.get_logger().info("=" * 70)
        self.get_logger().info("Keyboard Command Node Started")
        self.get_logger().info("=" * 70)
        self.print_help()
        self.get_logger().info("=" * 70)
        
        # Start keyboard input thread
        self.running = True
        self.input_thread = threading.Thread(target=self.read_keyboard, daemon=True)
        self.input_thread.start()
        
    def print_help(self):
        """Print keyboard command help"""
        help_text = """
FLIGHT MODE COMMANDS:
  0  - MC (Multicopter)
  1  - VTOL_TAKEOFF
  2  - HOVER
  3  - TRANSITION_MC_TO_FW
  4  - FW (Forward Flight) - Note: Use mode 3 first!
  5  - TRANSITION_FW_TO_MC
  6  - LAND
  7  - EMERGENCY

POSITION SETPOINT COMMANDS:
  w  - Move forward (+X)
  s  - Move backward (-X)
  a  - Move left (+Y)
  d  - Move right (-Y)
  q  - Move up (+Z, increase altitude) - Also updates odometry!
  e  - Move down (-Z, decrease altitude) - Also updates odometry!
  
  r  - Reset position to (0, 0, 25) - Also sets odometry altitude!

ARMING COMMANDS:
  space - Toggle arm/disarm

OTHER:
  h  - Show this help
  x  - Exit

Note: Make sure terminal has focus for keyboard input to work!
        """
        self.get_logger().info(help_text)
    
    def read_keyboard(self):
        """Read keyboard input in a separate thread"""
        tty.setraw(sys.stdin.fileno())
        
        try:
            while self.running and rclpy.ok():
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    self.handle_key(key)
        except Exception as e:
            self.get_logger().error(f"Keyboard input error: {e}")
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
    
    def handle_key(self, key):
        """Handle keyboard input"""
        key = key.lower()
        
        # Flight mode commands (0-7)
        if key in '01234567':
            mode = int(key)
            self.send_mode_command(mode)
        
        # Position setpoint commands
        elif key == 'w':
            self.current_x += 5.0
            self.send_position_setpoint()
        elif key == 's':
            self.current_x -= 5.0
            self.send_position_setpoint()
        elif key == 'a':
            self.current_y += 5.0
            self.send_position_setpoint()
        elif key == 'd':
            self.current_y -= 5.0
            self.send_position_setpoint()
        elif key == 'q':
            self.current_altitude += 5.0
            self.send_position_setpoint()
            # Also update odometry altitude
            self.odom_altitude += 5.0
            self.send_odometry()
        elif key == 'e':
            self.current_altitude = max(0.0, self.current_altitude - 5.0)
            self.send_position_setpoint()
            # Also update odometry altitude
            self.odom_altitude = max(0.0, self.odom_altitude - 5.0)
            self.send_odometry()
        elif key == 'r':
            self.current_x = 0.0
            self.current_y = 0.0
            self.current_altitude = 25.0
            self.send_position_setpoint()
            # Also set odometry altitude (mode manager reads from /odom)
            self.odom_x = 0.0
            self.odom_y = 0.0
            self.odom_altitude = 25.0
            self.send_odometry()
            self.get_logger().info("📍 Position reset to (0, 0, 25m) - Ready for FW mode!")
            self.get_logger().info("📍 Odometry altitude set to 25m (mode manager will see this)")
        
        # Arming commands
        elif key == ' ':
            self.toggle_arm()
        
        # Help
        elif key == 'h':
            self.print_help()
        
        # Exit
        elif key == 'x' or key == '\x03':  # x or Ctrl+C
            self.get_logger().info("Exiting keyboard command node...")
            self.running = False
            rclpy.shutdown()
        
        # Unknown key
        else:
            if key != '\n' and key != '\r':  # Ignore newlines
                self.get_logger().warn(f"Unknown key: '{key}' (press 'h' for help)")
    
    def send_mode_command(self, mode):
        """Send flight mode command"""
        mode_names = {
            0: "MC (Multicopter)",
            1: "VTOL_TAKEOFF",
            2: "HOVER",
            3: "TRANSITION_MC_TO_FW",
            4: "FW (Forward Flight)",
            5: "TRANSITION_FW_TO_MC",
            6: "LAND",
            7: "EMERGENCY"
        }
        
        msg = UInt8()
        msg.data = mode
        self.mode_cmd_pub.publish(msg)
        
        mode_name = mode_names.get(mode, f"Mode {mode}")
        self.get_logger().info(f"📡 Sent mode command: {mode} ({mode_name})")
        
        # Warnings for altitude requirements
        if mode == 3 or mode == 4:
            # Check odometry altitude (mode manager reads from /odom)
            if self.odom_altitude < 20.0:
                self.get_logger().warn(
                    f"⚠️  WARNING: Mode {mode} requires odometry altitude >= 20m! "
                    f"Current odometry altitude: {self.odom_altitude:.1f}m. "
                    f"Press 'q' to increase or 'r' to reset to 25m."
                )
            else:
                self.get_logger().info(f"✓ Odometry altitude OK: {self.odom_altitude:.1f}m >= 20m")
        
        if mode == 4:
            self.get_logger().warn("⚠️  Note: Can't go directly MC→FW. Use mode 3 (TRANSITION) first!")
    
    def send_position_setpoint(self):
        """Send position setpoint"""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = self.current_x
        msg.pose.position.y = self.current_y
        msg.pose.position.z = self.current_altitude
        msg.pose.orientation.w = 1.0
        
        self.setpoint_pub.publish(msg)
        self.get_logger().info(
            f"📍 Position setpoint: X={self.current_x:.1f}m, "
            f"Y={self.current_y:.1f}m, Z={self.current_altitude:.1f}m"
        )
    
    def send_odometry(self):
        """Send odometry message (mode manager reads altitude from this)"""
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = self.odom_x
        msg.pose.pose.position.y = self.odom_y
        msg.pose.pose.position.z = self.odom_altitude
        msg.pose.pose.orientation.w = 1.0
        msg.twist.twist.linear.x = 0.0
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.linear.z = 0.0
        
        self.odom_pub.publish(msg)
        self.get_logger().info(
            f"📊 Odometry set: X={self.odom_x:.1f}m, "
            f"Y={self.odom_y:.1f}m, Z={self.odom_altitude:.1f}m "
            f"(Mode manager reads this!)"
        )
    
    def toggle_arm(self):
        """Toggle arm/disarm"""
        # Note: We don't track current arm state, so we'll just send arm command
        # In real system, you'd check current state first
        msg = Bool()
        msg.data = True
        self.arm_cmd_pub.publish(msg)
        self.get_logger().info("🔓 Sent ARM command")
    
    def destroy_node(self):
        """Cleanup on shutdown"""
        self.running = False
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    node = KeyboardCommandNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

