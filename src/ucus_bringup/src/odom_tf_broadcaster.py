#!/usr/bin/env python3
"""
Odometry TF Broadcaster
Publishes transform from odom to base_link based on odometry messages
This is needed for RViz2 to visualize the vehicle
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


class OdomTfBroadcaster(Node):
    def __init__(self):
        super().__init__('odom_tf_broadcaster')
        
        # Create TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribe to odometry
        self.create_subscription(
            Odometry,
            '/odom',
            self._odom_cb,
            qos_profile
        )
        
        # Store current odometry
        self.current_odom = None
        
        # Timer to publish transform periodically (even if no odometry)
        self.create_timer(0.1, self._publish_tf)  # 10 Hz
        
        self.get_logger().info("📡 Odometry TF Broadcaster started")
        self.get_logger().info("   Publishing transform: odom -> base_link")
    
    def _odom_cb(self, msg):
        """Store odometry message"""
        self.current_odom = msg
    
    def _publish_tf(self):
        """Publish transform based on odometry, or identity if no odometry"""
        t = TransformStamped()
        
        # Header
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        
        if self.current_odom is not None:
            # Use odometry data
            t.transform.translation.x = self.current_odom.pose.pose.position.x
            t.transform.translation.y = self.current_odom.pose.pose.position.y
            t.transform.translation.z = self.current_odom.pose.pose.position.z
            t.transform.rotation = self.current_odom.pose.pose.orientation
        else:
            # Publish identity transform (no movement) so frame exists
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            t.transform.rotation.w = 1.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
        
        # Broadcast transform
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTfBroadcaster()
    
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

