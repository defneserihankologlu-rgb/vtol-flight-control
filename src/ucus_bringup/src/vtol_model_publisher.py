#!/usr/bin/env python3
"""
VTOL Model Publisher
Publishes a simple 3D model/marker representation of the VTOL vehicle
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import math


class VtolModelPublisher(Node):
    def __init__(self):
        super().__init__('vtol_model_publisher')
        
        # Parameters
        self.declare_parameter('model_scale', 1.0)  # Model scale
        self.declare_parameter('show_propellers', True)  # Show propeller markers
        
        self.model_scale = self.get_parameter('model_scale').get_parameter_value().double_value
        self.show_propellers = self.get_parameter('show_propellers').get_parameter_value().bool_value
        
        # State
        self.current_odom = None
        
        # QoS profile for subscription (Best Effort for sensor data)
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
        
        # Subscribe to odometry
        self.create_subscription(Odometry, '/odom', self._odom_cb, sub_qos_profile)
        
        # Publisher for markers (use Reliable QoS for RViz compatibility)
        self.marker_pub = self.create_publisher(Marker, '/vtol_model', pub_qos_profile)
        
        # Timer for publishing model
        self.create_timer(0.1, self._publish_model)  # 10 Hz
        
        self.get_logger().info("✈️  VTOL Model Publisher started")
    
    def _odom_cb(self, msg):
        self.current_odom = msg
    
    def _publish_model(self):
        """Publish VTOL model as markers"""
        # Always publish - use odom frame if no odometry, base_link if available
        use_base_link = self.current_odom is not None
        frame_id = "base_link" if use_base_link else "odom"
        
        if not use_base_link:
            # Publish a default model at origin if no odometry
            self._publish_default_model()
            return
        
        # Main body (cylinder) - use base_link frame if odometry exists
        body_marker = Marker()
        body_marker.header.frame_id = "base_link"
        body_marker.header.stamp = self.get_clock().now().to_msg()
        body_marker.ns = "vtol_model"
        body_marker.id = 0
        body_marker.type = Marker.CYLINDER
        body_marker.action = Marker.ADD
        
        # Position and orientation from odometry (relative to base_link, so always at origin)
        body_marker.pose.position.x = 0.0
        body_marker.pose.position.y = 0.0
        body_marker.pose.position.z = 0.0
        body_marker.pose.orientation = self.current_odom.pose.pose.orientation
        
        # Scale (body dimensions)
        body_marker.scale.x = 0.8 * self.model_scale  # Width
        body_marker.scale.y = 0.8 * self.model_scale  # Depth
        body_marker.scale.z = 0.3 * self.model_scale  # Height
        
        # Color (blue-gray)
        body_marker.color = ColorRGBA(r=0.3, g=0.5, b=0.8, a=1.0)
        
        body_marker.lifetime.sec = 1
        self.marker_pub.publish(body_marker)
        
        # Wings (if in FW mode or transition)
        # Simple representation with boxes
        wing_marker = Marker()
        wing_marker.header.frame_id = "base_link"
        wing_marker.header.stamp = self.get_clock().now().to_msg()
        wing_marker.ns = "vtol_model"
        wing_marker.id = 1
        wing_marker.type = Marker.CUBE
        wing_marker.action = Marker.ADD
        
        wing_marker.pose.position.x = 0.0
        wing_marker.pose.position.y = 0.0
        wing_marker.pose.position.z = 0.0
        wing_marker.pose.orientation = self.current_odom.pose.pose.orientation
        
        wing_marker.scale.x = 2.0 * self.model_scale  # Wing span
        wing_marker.scale.y = 0.1 * self.model_scale  # Wing chord
        wing_marker.scale.z = 0.05 * self.model_scale  # Wing thickness
        
        wing_marker.color = ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.7)
        
        wing_marker.lifetime.sec = 1
        self.marker_pub.publish(wing_marker)
        
        # Propellers (4 motors for quadcopter configuration)
        if self.show_propellers:
            prop_positions = [
                (0.4, 0.4, 0.2),   # Front right
                (0.4, -0.4, 0.2),  # Front left
                (-0.4, 0.4, 0.2),  # Back right
                (-0.4, -0.4, 0.2)  # Back left
            ]
            
            for i, (x, y, z) in enumerate(prop_positions):
                prop_marker = Marker()
                prop_marker.header.frame_id = "base_link"
                prop_marker.header.stamp = self.get_clock().now().to_msg()
                prop_marker.ns = "vtol_model"
                prop_marker.id = 2 + i
                prop_marker.type = Marker.CYLINDER
                prop_marker.action = Marker.ADD
                
                prop_marker.pose.position.x = x * self.model_scale
                prop_marker.pose.position.y = y * self.model_scale
                prop_marker.pose.position.z = z * self.model_scale
                prop_marker.pose.orientation.w = 1.0
                
                prop_marker.scale.x = 0.3 * self.model_scale  # Propeller diameter
                prop_marker.scale.y = 0.3 * self.model_scale
                prop_marker.scale.z = 0.02 * self.model_scale  # Thickness
                
                prop_marker.color = ColorRGBA(r=0.8, g=0.8, b=0.8, a=0.6)
                
                prop_marker.lifetime.sec = 1
                self.marker_pub.publish(prop_marker)
    
    def _publish_default_model(self):
        """Publish default model at origin when no odometry available"""
        # Main body
        body_marker = Marker()
        body_marker.header.frame_id = "odom"
        body_marker.header.stamp = self.get_clock().now().to_msg()
        body_marker.ns = "vtol_model"
        body_marker.id = 0
        body_marker.type = Marker.CYLINDER
        body_marker.action = Marker.ADD
        
        body_marker.pose.position.x = 0.0
        body_marker.pose.position.y = 0.0
        body_marker.pose.position.z = 0.5  # Slightly above ground
        body_marker.pose.orientation.w = 1.0
        
        body_marker.scale.x = 0.8 * self.model_scale
        body_marker.scale.y = 0.8 * self.model_scale
        body_marker.scale.z = 0.3 * self.model_scale
        
        body_marker.color = ColorRGBA(r=0.3, g=0.5, b=0.8, a=1.0)
        
        body_marker.lifetime.sec = 1
        self.marker_pub.publish(body_marker)
        
        # Wings
        wing_marker = Marker()
        wing_marker.header.frame_id = "odom"
        wing_marker.header.stamp = self.get_clock().now().to_msg()
        wing_marker.ns = "vtol_model"
        wing_marker.id = 1
        wing_marker.type = Marker.CUBE
        wing_marker.action = Marker.ADD
        
        wing_marker.pose.position.x = 0.0
        wing_marker.pose.position.y = 0.0
        wing_marker.pose.position.z = 0.5
        wing_marker.pose.orientation.w = 1.0
        
        wing_marker.scale.x = 2.0 * self.model_scale
        wing_marker.scale.y = 0.1 * self.model_scale
        wing_marker.scale.z = 0.05 * self.model_scale
        
        wing_marker.color = ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.7)
        
        wing_marker.lifetime.sec = 1
        self.marker_pub.publish(wing_marker)
        
        # Propellers
        if self.show_propellers:
            prop_positions = [
                (0.4, 0.4, 0.6),   # Front right
                (0.4, -0.4, 0.6),  # Front left
                (-0.4, 0.4, 0.6),  # Back right
                (-0.4, -0.4, 0.6)  # Back left
            ]
            
            for i, (x, y, z) in enumerate(prop_positions):
                prop_marker = Marker()
                prop_marker.header.frame_id = "odom"
                prop_marker.header.stamp = self.get_clock().now().to_msg()
                prop_marker.ns = "vtol_model"
                prop_marker.id = 2 + i
                prop_marker.type = Marker.CYLINDER
                prop_marker.action = Marker.ADD
                
                prop_marker.pose.position.x = x * self.model_scale
                prop_marker.pose.position.y = y * self.model_scale
                prop_marker.pose.position.z = z * self.model_scale
                prop_marker.pose.orientation.w = 1.0
                
                prop_marker.scale.x = 0.3 * self.model_scale
                prop_marker.scale.y = 0.3 * self.model_scale
                prop_marker.scale.z = 0.02 * self.model_scale
                
                prop_marker.color = ColorRGBA(r=0.8, g=0.8, b=0.8, a=0.6)
                
                prop_marker.lifetime.sec = 1
                self.marker_pub.publish(prop_marker)


def main(args=None):
    rclpy.init(args=args)
    node = VtolModelPublisher()
    
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

