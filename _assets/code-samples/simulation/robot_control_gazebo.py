#!/usr/bin/env python3

"""
Robot Control for Gazebo Simulation

This script demonstrates how to control a robot in Gazebo simulation using ROS 2.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from tf2_ros import TransformListener, Buffer
import math


class GazeboRobotController(Node):
    """
    A node that controls a robot in Gazebo simulation.
    """

    def __init__(self):
        super().__init__('gazebo_robot_controller')

        # Publisher for velocity commands
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber for odometry
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz

        # Robot state
        self.current_pose = Pose()
        self.current_twist = Twist()
        self.target_x = 2.0  # Target position x
        self.target_y = 1.0  # Target position y
        self.reached_target = False

        # PID controller parameters
        self.kp_linear = 0.5
        self.ki_linear = 0.0
        self.kd_linear = 0.0
        self.kp_angular = 1.0
        self.ki_angular = 0.0
        self.kd_angular = 0.0

        # PID terms
        self.prev_error_linear = 0.0
        self.integral_error_linear = 0.0
        self.prev_error_angular = 0.0
        self.integral_error_angular = 0.0

        self.get_logger().info('Gazebo robot controller initialized')

    def odom_callback(self, msg):
        """
        Callback function for odometry data.

        Args:
            msg: Odometry message from Gazebo
        """
        self.current_pose = msg.pose.pose
        self.current_twist = msg.twist.twist

    def control_loop(self):
        """
        Main control loop for robot navigation.
        """
        if self.reached_target:
            # Stop the robot when target is reached
            self.stop_robot()
            return

        # Calculate distance to target
        dx = self.target_x - self.current_pose.position.x
        dy = self.target_y - self.current_pose.position.y
        distance_to_target = math.sqrt(dx*dx + dy*dy)

        # Calculate target angle
        target_angle = math.atan2(dy, dx)

        # Get current robot angle from quaternion
        current_angle = self.quaternion_to_yaw(
            self.current_pose.orientation.x,
            self.current_pose.orientation.y,
            self.current_pose.orientation.z,
            self.current_pose.orientation.w
        )

        # Calculate angle error
        angle_error = self.normalize_angle(target_angle - current_angle)

        # Check if target is reached
        if distance_to_target < 0.1:  # 10 cm tolerance
            self.reached_target = True
            self.get_logger().info(f'Reached target position ({self.target_x}, {self.target_y})')
            self.stop_robot()
            return

        # PID control for linear velocity
        linear_error = distance_to_target
        self.integral_error_linear += linear_error * 0.1  # dt = 0.1
        derivative_error_linear = (linear_error - self.prev_error_linear) / 0.1
        linear_velocity = (
            self.kp_linear * linear_error +
            self.ki_linear * self.integral_error_linear +
            self.kd_linear * derivative_error_linear
        )

        # Limit linear velocity
        linear_velocity = max(0.0, min(linear_velocity, 0.5))

        # PID control for angular velocity
        angular_error = angle_error
        self.integral_error_angular += angular_error * 0.1  # dt = 0.1
        derivative_error_angular = (angular_error - self.prev_error_angular) / 0.1
        angular_velocity = (
            self.kp_angular * angular_error +
            self.ki_angular * self.integral_error_angular +
            self.kd_angular * derivative_error_angular
        )

        # Limit angular velocity
        angular_velocity = max(-1.0, min(angular_velocity, 1.0))

        # Create and publish velocity command
        twist = Twist()
        twist.linear.x = linear_velocity
        twist.angular.z = angular_velocity
        self.cmd_vel_publisher.publish(twist)

        self.prev_error_linear = linear_error
        self.prev_error_angular = angular_error

        self.get_logger().info(f'Control - Linear: {linear_velocity:.2f}, '
                             f'Angular: {angular_velocity:.2f}, '
                             f'Distance: {distance_to_target:.2f}')

    def quaternion_to_yaw(self, x, y, z, w):
        """
        Convert quaternion to yaw angle.

        Args:
            x, y, z, w: Quaternion components

        Returns:
            Yaw angle in radians
        """
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        """
        Normalize angle to [-pi, pi] range.

        Args:
            angle: Angle in radians

        Returns:
            Normalized angle in radians
        """
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def stop_robot(self):
        """
        Stop the robot by sending zero velocities.
        """
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)

    def set_target(self, x, y):
        """
        Set a new target position for the robot.

        Args:
            x: Target x position
            y: Target y position
        """
        self.target_x = x
        self.target_y = y
        self.reached_target = False
        self.get_logger().info(f'Set new target: ({x}, {y})')


def main(args=None):
    """
    Main function to run the robot controller.
    """
    rclpy.init(args=args)

    controller = GazeboRobotController()

    # Set a target position after a short delay
    def set_target():
        controller.set_target(2.0, 1.0)

    # Schedule the target setting
    timer = controller.create_timer(1.0, set_target)

    # Cancel the timer after setting the target once
    def cancel_timer():
        timer.cancel()

    controller.create_timer(1.1, cancel_timer)

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Interrupted, stopping robot...')
        controller.stop_robot()
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()