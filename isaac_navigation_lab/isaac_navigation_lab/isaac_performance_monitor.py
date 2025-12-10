#!/usr/bin/env python3

"""
Isaac Navigation Performance Monitor

This script monitors and compares the performance of Isaac navigation vs standard navigation.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
import time
import numpy as np


class IsaacPerformanceMonitor(Node):
    """
    A node that monitors navigation performance metrics.
    """

    def __init__(self):
        super().__init__('isaac_performance_monitor')

        # Subscribers for navigation metrics
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Publishers for performance metrics
        self.cpu_usage_publisher = self.create_publisher(Float32, '/performance/cpu_usage', 10)
        self.gpu_usage_publisher = self.create_publisher(Float32, '/performance/gpu_usage', 10)
        self.navigation_time_publisher = self.create_publisher(Float32, '/performance/navigation_time', 10)

        # Performance tracking
        self.start_time = time.time()
        self.navigation_start_time = None
        self.navigation_end_time = None
        self.odom_count = 0
        self.scan_count = 0

        # Timers for periodic monitoring
        self.monitor_timer = self.create_timer(1.0, self.monitor_performance)

        self.get_logger().info('Isaac Performance Monitor initialized')

    def odom_callback(self, msg):
        """Track odometry processing rate."""
        self.odom_count += 1

    def scan_callback(self, msg):
        """Track scan processing rate."""
        self.scan_count += 1

    def monitor_performance(self):
        """Monitor and publish performance metrics."""
        # Calculate processing rates
        current_time = time.time()
        elapsed = current_time - self.start_time

        odom_rate = self.odom_count / elapsed if elapsed > 0 else 0
        scan_rate = self.scan_count / elapsed if elapsed > 0 else 0

        self.get_logger().info(f'Performance - Odom rate: {odom_rate:.2f} Hz, '
                             f'Scan rate: {scan_rate:.2f} Hz')

        # Publish dummy performance metrics (in a real system, these would come from system monitoring)
        cpu_usage_msg = Float32()
        cpu_usage_msg.data = np.random.uniform(20.0, 80.0)  # Simulated CPU usage
        self.cpu_usage_publisher.publish(cpu_usage_msg)

        gpu_usage_msg = Float32()
        gpu_usage_msg.data = np.random.uniform(40.0, 95.0)  # Simulated GPU usage for Isaac
        self.gpu_usage_publisher.publish(gpu_usage_msg)

    def start_navigation_timer(self):
        """Start timing for navigation task."""
        self.navigation_start_time = time.time()
        self.get_logger().info('Navigation timer started')

    def stop_navigation_timer(self):
        """Stop timing for navigation task and publish result."""
        if self.navigation_start_time is not None:
            self.navigation_end_time = time.time()
            navigation_time = self.navigation_end_time - self.navigation_start_time

            time_msg = Float32()
            time_msg.data = float(navigation_time)
            self.navigation_time_publisher.publish(time_msg)

            self.get_logger().info(f'Navigation completed in {navigation_time:.3f} seconds')

            # Reset timers
            self.navigation_start_time = None
            self.navigation_end_time = None


def main(args=None):
    """Main function to run the performance monitor."""
    rclpy.init(args=args)

    monitor = IsaacPerformanceMonitor()

    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        monitor.get_logger().info('Performance monitor interrupted')
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()