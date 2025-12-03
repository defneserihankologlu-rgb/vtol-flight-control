#!/usr/bin/env python3
"""
Flight Path Publisher
Publishes flight path as a Path message for visualization in RViz2
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


class FlightPathPublisher(Node):
    def __init__(self):
        super().__init__('flight_path_publisher')
        
        # Parameters
        self.declare_parameter('max_path_length', 1000)  # Maximum number of poses in path
        self.declare_parameter('update_rate_hz', 10.0)  # Path update rate
        self.declare_parameter('min_distance', 0.1)  # Minimum distance between path points (m)
        
        self.max_path_length = self.get_parameter('max_path_length').get_parameter_value().integer_value
        self.min_distance = self.get_parameter('min_distance').get_parameter_value().double_value
        update_rate = self.get_parameter('update_rate_hz').get_parameter_value().double_value
        
        # Path storage
        self.path = Path()
        self.path.header.frame_id = "odom"
        self.last_position = None
        
        # QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscriptions
        self.create_subscription(Odometry, '/odom', self._odom_cb, qos_profile)
        
        # Publishers
        self.path_pub = self.create_publisher(Path, '/flight_path', 10)
        
        # Timer for publishing path
        self.create_timer(1.0 / update_rate, self._publish_path)
        
        self.get_logger().info("🛤️  Flight Path Publisher started")
        self.get_logger().info(f"   Max path length: {self.max_path_length}")
        self.get_logger().info(f"   Min distance: {self.min_distance} m")
        self.get_logger().info(f"   Update rate: {update_rate} Hz")
    
    def _odom_cb(self, msg):
        """Add new pose to path if it's far enough from last point"""
        current_pos = msg.pose.pose.position
        
        # Check if we should add this point
        should_add = False
        if self.last_position is None:
            should_add = True
        else:
            dx = current_pos.x - self.last_position.x
            dy = current_pos.y - self.last_position.y
            dz = current_pos.z - self.last_position.z
            distance = (dx*dx + dy*dy + dz*dz) ** 0.5
            
            if distance >= self.min_distance:
                should_add = True
        
        if should_add:
            pose_stamped = PoseStamped()
            pose_stamped.header = msg.header
            pose_stamped.pose = msg.pose.pose
            
            self.path.poses.append(pose_stamped)
            self.last_position = current_pos
            
            # Limit path length
            if len(self.path.poses) > self.max_path_length:
                self.path.poses.pop(0)
    
    def _publish_path(self):
        """Publish the current path"""
        if len(self.path.poses) > 0:
            self.path.header.stamp = self.get_clock().now().to_msg()
            self.path_pub.publish(self.path)


def main(args=None):
    rclpy.init(args=args)
    node = FlightPathPublisher()
    
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

