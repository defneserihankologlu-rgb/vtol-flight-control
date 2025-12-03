#!/usr/bin/env python3
"""
Flight Mode Marker Publisher
Publishes visual markers for current flight mode in RViz2
"""

import rclpy
from rclpy.node import Node
from ucus_msgs.msg import FlightMode
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry


class FlightModeMarkerPublisher(Node):
    def __init__(self):
        super().__init__('flight_mode_marker_publisher')
        
        # Parameters
        self.declare_parameter('marker_scale', 0.5)  # Marker scale
        self.declare_parameter('marker_lifetime', 1.0)  # Marker lifetime in seconds
        
        self.marker_scale = self.get_parameter('marker_scale').get_parameter_value().double_value
        self.marker_lifetime = self.get_parameter('marker_lifetime').get_parameter_value().double_value
        
        # Mode colors (RGBA)
        self.mode_colors = {
            0: (0.0, 1.0, 0.0, 1.0),      # MC - Green
            1: (0.0, 0.5, 1.0, 1.0),      # VTOL_TAKEOFF - Blue
            2: (0.0, 1.0, 1.0, 1.0),      # HOVER - Cyan
            3: (1.0, 0.5, 0.0, 1.0),      # TRANSITION_MC_TO_FW - Orange
            4: (1.0, 0.0, 0.0, 1.0),      # FW - Red
            5: (1.0, 0.5, 0.0, 1.0),      # TRANSITION_FW_TO_MC - Orange
            6: (0.5, 0.0, 1.0, 1.0),      # LAND - Purple
            7: (1.0, 0.0, 0.0, 1.0),      # EMERGENCY - Red (bright)
        }
        
        # Mode names
        self.mode_names = {
            0: "MC",
            1: "VTOL_TAKEOFF",
            2: "HOVER",
            3: "TRANSITION_MC_TO_FW",
            4: "FW",
            5: "TRANSITION_FW_TO_MC",
            6: "LAND",
            7: "EMERGENCY"
        }
        
        # State
        self.current_mode = None
        self.current_odom = None
        
        # QoS profile for subscriptions (Best Effort for sensor data)
        sub_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # QoS profile for publisher (Reliable for visualization)
        pub_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscriptions
        self.create_subscription(FlightMode, '/flight_mode', self._flight_mode_cb, sub_qos_profile)
        self.create_subscription(Odometry, '/odom', self._odom_cb, sub_qos_profile)
        
        # Publishers (use Reliable QoS for RViz compatibility)
        self.marker_pub = self.create_publisher(Marker, '/flight_mode_marker', pub_qos_profile)
        
        # Timer for publishing marker
        self.create_timer(0.1, self._publish_marker)  # 10 Hz
        
        self.get_logger().info("🎯 Flight Mode Marker Publisher started")
    
    def _flight_mode_cb(self, msg):
        self.current_mode = msg
    
    def _odom_cb(self, msg):
        self.current_odom = msg
    
    def _publish_marker(self):
        """Publish marker for current flight mode"""
        # Always publish, even if no data (will show at origin)
        if self.current_mode is None:
            # Create a default marker if no mode data
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "flight_mode"
            marker.id = 0
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose.position.x = 0.0
            marker.pose.position.y = 0.0
            marker.pose.position.z = 2.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = self.marker_scale
            marker.scale.y = self.marker_scale
            marker.scale.z = self.marker_scale
            marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            marker.text = "Waiting for data..."
            marker.lifetime.sec = int(self.marker_lifetime)
            marker.lifetime.nanosec = int((self.marker_lifetime - int(self.marker_lifetime)) * 1e9)
            self.marker_pub.publish(marker)
            return
        
        if self.current_odom is None:
            # Publish at origin if no odometry
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "flight_mode"
            marker.id = 0
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose.position.x = 0.0
            marker.pose.position.y = 0.0
            marker.pose.position.z = 2.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = self.marker_scale
            marker.scale.y = self.marker_scale
            marker.scale.z = self.marker_scale
            mode = self.current_mode.mode
            color = self.mode_colors.get(mode, (1.0, 1.0, 1.0, 1.0))
            marker.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3])
            mode_name = self.mode_names.get(mode, f"UNKNOWN({mode})")
            marker.text = f"Mode: {mode_name}"
            marker.lifetime.sec = int(self.marker_lifetime)
            marker.lifetime.nanosec = int((self.marker_lifetime - int(self.marker_lifetime)) * 1e9)
            self.marker_pub.publish(marker)
            return
        
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "flight_mode"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        # Position (above vehicle)
        marker.pose.position.x = self.current_odom.pose.pose.position.x
        marker.pose.position.y = self.current_odom.pose.pose.position.y
        marker.pose.position.z = self.current_odom.pose.pose.position.z + 2.0  # 2m above
        marker.pose.orientation.w = 1.0
        
        # Scale
        marker.scale.x = self.marker_scale
        marker.scale.y = self.marker_scale
        marker.scale.z = self.marker_scale
        
        # Color based on mode
        mode = self.current_mode.mode
        color = self.mode_colors.get(mode, (1.0, 1.0, 1.0, 1.0))
        marker.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3])
        
        # Text
        mode_name = self.mode_names.get(mode, f"UNKNOWN({mode})")
        marker.text = f"Mode: {mode_name}"
        
        # Lifetime
        marker.lifetime.sec = int(self.marker_lifetime)
        marker.lifetime.nanosec = int((self.marker_lifetime - int(self.marker_lifetime)) * 1e9)
        
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = FlightModeMarkerPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()

