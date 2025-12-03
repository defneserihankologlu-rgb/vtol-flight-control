#!/usr/bin/env python3
"""
Telemetry Logger Node for VTOL System
Records all important topics to rosbag2 for post-flight analysis
Uses rosbag2 Python API for programmatic recording
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from ucus_msgs.msg import FlightMode, ActuatorCmd, SafetyStatus, ArmingStatus
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
import os
from datetime import datetime

# Try to import rosbag2
try:
    from rclpy.serialization import serialize_message
    from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions
    ROSBAG2_AVAILABLE = True
except ImportError:
    ROSBAG2_AVAILABLE = False


class TelemetryLoggerNode(Node):
    def __init__(self):
        super().__init__('telemetry_logger')
        
        # Parameters
        self.declare_parameter('log_directory', '~/ucus_kontrol_ws/logs')
        self.declare_parameter('log_prefix', 'vtol_flight')
        self.declare_parameter('auto_start', True)
        self.declare_parameter('max_bag_size', 1073741824)  # 1GB default
        self.declare_parameter('topics_to_log', [
            '/odom',
            '/flight_mode',
            '/actuator_cmd',
            '/safety/status',
            '/safety/arming_status',
            '/armed',
            '/imu/data',
            '/setpoint/pos'
        ])
        
        log_dir = self.get_parameter('log_directory').get_parameter_value().string_value
        log_prefix = self.get_parameter('log_prefix').get_parameter_value().string_value
        auto_start = self.get_parameter('auto_start').get_parameter_value().bool_value
        self.max_bag_size = self.get_parameter('max_bag_size').get_parameter_value().integer_value
        
        # Expand ~ in path
        log_dir = os.path.expanduser(log_dir)
        
        # Create log directory if it doesn't exist
        os.makedirs(log_dir, exist_ok=True)
        
        # Generate log filename with timestamp
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        log_filename = f"{log_prefix}_{timestamp}"
        self.log_path = os.path.join(log_dir, log_filename)
        
        # Rosbag2 writer
        self.writer = None
        self.is_logging = False
        self.logging_started = False
        
        # Check if rosbag2 is available
        if not ROSBAG2_AVAILABLE:
            self.get_logger().error("⚠️  rosbag2_py not available - logging disabled")
            self.get_logger().info("   Install with: sudo apt install ros-humble-rosbag2-py")
            return
        
        # Initialize rosbag2 writer if auto_start
        if auto_start:
            self._start_logging()
        else:
            self.get_logger().info("📝 Telemetry Logger initialized (auto_start=false)")
            self.get_logger().info("   Use service to start logging: /telemetry_logger/start")
        
        # Statistics
        self.message_counts = {}
        self.start_time = self.get_clock().now()
        self.total_bytes_written = 0
        
        # Topic subscriptions for logging
        self.topics_to_log = self.get_parameter('topics_to_log').get_parameter_value().string_array_value
        
        # Create subscriptions with callbacks that log messages
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create subscriptions for all topics
        self._create_subscriptions(qos_profile)
        
        # Statistics timer
        self.create_timer(10.0, self._print_statistics)
        
        # Service to start/stop logging
        from rclpy.qos import qos_profile_services_default
        from std_srvs.srv import SetBool
        
        self.start_service = self.create_service(
            SetBool,
            '/telemetry_logger/start',
            self._start_logging_service_cb
        )
        
        self.stop_service = self.create_service(
            SetBool,
            '/telemetry_logger/stop',
            self._stop_logging_service_cb
        )
        
        self.get_logger().info("✅ Telemetry Logger Node started")
        if auto_start:
            self.get_logger().info(f"   📝 Logging to: {self.log_path}")
        else:
            self.get_logger().info("   ⏸️  Logging paused - use service to start")
    
    def _create_subscriptions(self, qos_profile):
        """Create subscriptions for all topics to log"""
        # Map topic names to message types
        topic_types = {
            '/odom': Odometry,
            '/flight_mode': FlightMode,
            '/actuator_cmd': ActuatorCmd,
            '/safety/status': SafetyStatus,
            '/safety/arming_status': ArmingStatus,
            '/armed': Bool,
            '/imu/data': Imu,
            '/setpoint/pos': PoseStamped
        }
        
        for topic in self.topics_to_log:
            msg_type = topic_types.get(topic)
            if msg_type:
                self.create_subscription(
                    msg_type,
                    topic,
                    lambda msg, t=topic: self._log_message_cb(msg, t),
                    qos_profile
                )
            else:
                self.get_logger().warn(f"⚠️  Unknown topic type for {topic}, skipping")
    
    def _start_logging(self):
        """Start rosbag2 logging"""
        if self.is_logging:
            self.get_logger().warn("⚠️  Logging already in progress")
            return
        
        if not ROSBAG2_AVAILABLE:
            self.get_logger().error("❌ rosbag2_py not available")
            return
        
        try:
            # Create storage options
            storage_options = StorageOptions(
                uri=self.log_path,
                storage_id='sqlite3'
            )
            
            # Create converter options
            converter_options = ConverterOptions(
                input_serialization_format='cdr',
                output_serialization_format='cdr'
            )
            
            # Create writer
            self.writer = SequentialWriter()
            self.writer.open(storage_options, converter_options)
            
            # Register topics
            topic_types = {
                '/odom': 'nav_msgs/msg/Odometry',
                '/flight_mode': 'ucus_msgs/msg/FlightMode',
                '/actuator_cmd': 'ucus_msgs/msg/ActuatorCmd',
                '/safety/status': 'ucus_msgs/msg/SafetyStatus',
                '/safety/arming_status': 'ucus_msgs/msg/ArmingStatus',
                '/armed': 'std_msgs/msg/Bool',
                '/imu/data': 'sensor_msgs/msg/Imu',
                '/setpoint/pos': 'geometry_msgs/msg/PoseStamped'
            }
            
            for topic in self.topics_to_log:
                topic_type = topic_types.get(topic)
                if topic_type:
                    topic_metadata = {
                        'name': topic,
                        'type': topic_type,
                        'serialization_format': 'cdr'
                    }
                    self.writer.create_topic(topic_metadata)
            
            self.is_logging = True
            self.logging_started = True
            self.start_time = self.get_clock().now()
            self.message_counts = {}
            self.total_bytes_written = 0
            
            self.get_logger().info(f"✅ Started logging to: {self.log_path}")
            self.get_logger().info(f"   Topics: {', '.join(self.topics_to_log)}")
            
        except Exception as e:
            self.get_logger().error(f"❌ Failed to start logging: {str(e)}")
            self.is_logging = False
            self.writer = None
    
    def _stop_logging(self):
        """Stop rosbag2 logging"""
        if not self.is_logging:
            self.get_logger().warn("⚠️  Logging not in progress")
            return
        
        try:
            if self.writer:
                self.writer.close()
                self.writer = None
            
            self.is_logging = False
            
            # Print summary
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            total_messages = sum(self.message_counts.values())
            
            self.get_logger().info("⏹️  Logging stopped")
            self.get_logger().info(f"   Duration: {elapsed:.1f} seconds")
            self.get_logger().info(f"   Total messages: {total_messages}")
            self.get_logger().info(f"   Bag location: {self.log_path}")
            
            # Print per-topic statistics
            for topic, count in self.message_counts.items():
                rate = count / elapsed if elapsed > 0 else 0
                self.get_logger().info(f"   {topic}: {count} messages ({rate:.1f} Hz)")
            
        except Exception as e:
            self.get_logger().error(f"❌ Error stopping logging: {str(e)}")
    
    def _log_message_cb(self, msg, topic_name):
        """Callback to log messages to rosbag2"""
        if not self.is_logging or not self.writer:
            return
        
        try:
            # Serialize message
            serialized_msg = serialize_message(msg)
            
            # Get timestamp
            timestamp = self.get_clock().now().nanoseconds
            
            # Write to bag
            self.writer.write(topic_name, serialized_msg, timestamp)
            
            # Update statistics
            if topic_name not in self.message_counts:
                self.message_counts[topic_name] = 0
            self.message_counts[topic_name] += 1
            self.total_bytes_written += len(serialized_msg)
            
        except Exception as e:
            self.get_logger().error(f"❌ Error writing message to bag: {str(e)}")
    
    def _start_logging_service_cb(self, request, response):
        """Service callback to start logging"""
        if request.data:
            self._start_logging()
            response.success = self.is_logging
            response.message = "Logging started" if self.is_logging else "Failed to start logging"
        else:
            response.success = False
            response.message = "Use data=true to start logging"
        return response
    
    def _stop_logging_service_cb(self, request, response):
        """Service callback to stop logging"""
        if request.data:
            self._stop_logging()
            response.success = not self.is_logging
            response.message = "Logging stopped" if not self.is_logging else "Failed to stop logging"
        else:
            response.success = False
            response.message = "Use data=true to stop logging"
        return response
    
    def _print_statistics(self):
        """Print logging statistics periodically"""
        if not self.is_logging:
            return
        
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        total_messages = sum(self.message_counts.values())
        
        if elapsed > 0:
            self.get_logger().info(
                f"📊 Logging: {elapsed:.1f}s | "
                f"{total_messages} msgs | "
                f"{self.total_bytes_written / 1024 / 1024:.1f} MB"
            )
    
    def destroy_node(self):
        """Cleanup on shutdown"""
        if self.is_logging:
            self.get_logger().info("Shutting down - stopping logging...")
            self._stop_logging()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryLoggerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down telemetry logger...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
