#!/usr/bin/env python3

"""
Gazebo Sensor Subscriber

This script demonstrates how to subscribe to sensor data from Gazebo simulation.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu, Image
from geometry_msgs.msg import Twist
import numpy as np


class GazeboSensorSubscriber(Node):
    """
    A node that subscribes to various sensor data from Gazebo simulation.
    """

    def __init__(self):
        super().__init__('gazebo_sensor_subscriber')

        # Create subscribers for different sensor types
        self.laser_subscriber = self.create_subscription(
            LaserScan,
            '/scan',  # Standard topic for laser scan data in Gazebo
            self.laser_callback,
            10
        )

        self.imu_subscriber = self.create_subscription(
            Imu,
            '/imu',  # Standard topic for IMU data in Gazebo
            self.imu_callback,
            10
        )

        self.camera_subscriber = self.create_subscription(
            Image,
            '/camera/image_raw',  # Standard topic for camera images in Gazebo
            self.camera_callback,
            10
        )

        # Publisher for robot control (to demonstrate integration)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info('Gazebo sensor subscriber initialized')

    def laser_callback(self, msg):
        """
        Callback function for laser scan data.

        Args:
            msg: LaserScan message from Gazebo
        """
        # Process laser scan data
        ranges = np.array(msg.ranges)

        # Filter out invalid ranges (inf or nan)
        valid_ranges = ranges[np.isfinite(ranges)]

        if len(valid_ranges) > 0:
            min_distance = np.min(valid_ranges)
            self.get_logger().info(f'Laser scan - Min distance: {min_distance:.2f}m, '
                                 f'Number of valid readings: {len(valid_ranges)}')

            # Simple obstacle avoidance behavior
            if min_distance < 1.0:  # If obstacle is closer than 1 meter
                self.avoid_obstacle()

    def imu_callback(self, msg):
        """
        Callback function for IMU data.

        Args:
            msg: Imu message from Gazebo
        """
        orientation = msg.orientation
        angular_velocity = msg.angular_velocity
        linear_acceleration = msg.linear_acceleration

        self.get_logger().info(f'IMU - Orientation: ({orientation.x:.2f}, {orientation.y:.2f}, '
                             f'{orientation.z:.2f}, {orientation.w:.2f})')

    def camera_callback(self, msg):
        """
        Callback function for camera image data.

        Args:
            msg: Image message from Gazebo camera
        """
        height = msg.height
        width = msg.width
        encoding = msg.encoding

        self.get_logger().info(f'Camera - Resolution: {width}x{height}, '
                             f'Encoding: {encoding}, '
                             f'Step: {msg.step} bytes per row')

    def avoid_obstacle(self):
        """
        Simple obstacle avoidance behavior.
        """
        twist = Twist()
        # Stop forward motion and turn
        twist.linear.x = 0.0
        twist.angular.z = 0.5  # Turn right
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info('Obstacle detected! Turning to avoid...')


def main(args=None):
    """
    Main function to run the sensor subscriber.
    """
    rclpy.init(args=args)

    subscriber = GazeboSensorSubscriber()

    try:
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        subscriber.get_logger().info('Interrupted, shutting down...')
    finally:
        subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()