---
title: Week 13 - Humanoid Locomotion and Navigation
sidebar_position: 6
week: 13
module: module-4-vla-systems
learningObjectives:
  - Implement bipedal locomotion algorithms for humanoid robots
  - Design navigation systems for complex humanoid environments
  - Integrate perception with locomotion planning
  - Apply dynamic balance control during movement
prerequisites:
  - Week 1-12 content: Complete textbook modules
  - Understanding of kinematics and dynamics
  - Knowledge of control theory
  - Experience with ROS 2 navigation systems
description: Advanced locomotion and navigation techniques for humanoid robots with dynamic balance control
---

# Week 13: Humanoid Locomotion and Navigation

## Learning Objectives

By the end of this week, students will be able to:
- Implement bipedal locomotion algorithms for humanoid robots
- Design dynamic balance control systems for walking
- Integrate perception with navigation planning for humanoid robots
- Apply advanced path planning techniques for complex environments
- Evaluate and optimize humanoid locomotion performance

## Overview

Humanoid locomotion represents one of the most challenging aspects of humanoid robotics, requiring sophisticated algorithms to maintain balance while achieving stable, efficient walking. This week explores the fundamental principles and practical implementations of humanoid locomotion and navigation, building on the control and perception concepts learned in previous weeks.

## Fundamentals of Humanoid Locomotion

### Walking Patterns and Gait Analysis

Humanoid walking involves complex coordination of multiple joints and balance control:

```python
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal

class GaitAnalyzer:
    def __init__(self):
        self.step_length = 0.3  # meters
        self.step_width = 0.2   # meters (distance between feet)
        self.step_height = 0.05 # meters (clearance height)
        self.step_duration = 1.0 # seconds
        self.zmp_margin = 0.05  # safety margin for ZMP

    def generate_walk_pattern(self, num_steps, walking_speed=0.3):
        """
        Generate walking pattern parameters based on desired speed.

        Args:
            num_steps: Number of steps to generate
            walking_speed: Desired walking speed (m/s)

        Returns:
            Dictionary containing walking parameters
        """
        # Adjust step parameters based on speed
        adjusted_duration = self.step_length / walking_speed if walking_speed > 0 else self.step_duration
        adjusted_height = self.step_height * (walking_speed / 0.3)  # Scale height with speed

        # Generate footstep positions
        left_footsteps = []
        right_footsteps = []

        for i in range(num_steps):
            # Left foot step
            if i % 2 == 0:  # Even steps: left foot
                left_pos = np.array([
                    i * self.step_length,
                    0.0 if i == 0 else self.step_width/2,
                    0.0
                ])
                left_footsteps.append(left_pos)
            else:  # Odd steps: right foot
                right_pos = np.array([
                    i * self.step_length,
                    -self.step_width/2,
                    0.0
                ])
                right_footsteps.append(right_pos)

        return {
            'left_footsteps': left_footsteps,
            'right_footsteps': right_footsteps,
            'step_length': self.step_length,
            'step_duration': adjusted_duration,
            'step_height': adjusted_height,
            'walking_speed': walking_speed
        }

    def analyze_gait_stability(self, com_trajectory, zmp_trajectory):
        """
        Analyze gait stability based on center of mass and ZMP trajectories.

        Args:
            com_trajectory: Center of mass trajectory over time
            zmp_trajectory: Zero Moment Point trajectory over time

        Returns:
            Stability metrics
        """
        # Calculate stability margins
        stability_margins = []
        for i in range(len(com_trajectory)):
            com = com_trajectory[i]
            zmp = zmp_trajectory[i]

            # Calculate distance from ZMP to support polygon
            # For simplicity, assume rectangular support polygon
            support_width = self.step_width
            support_length = self.step_length * 0.5  # Half step length

            # Calculate margins
            x_margin = support_length - abs(com[0] - zmp[0])
            y_margin = support_width/2 - abs(com[1] - zmp[1])

            stability_margins.append(min(x_margin, y_margin))

        avg_stability = np.mean(stability_margins)
        min_stability = np.min(stability_margins)

        return {
            'average_stability_margin': avg_stability,
            'minimum_stability_margin': min_stability,
            'stability_margins': stability_margins
        }

# Example usage
gait_analyzer = GaitAnalyzer()
walking_params = gait_analyzer.generate_walk_pattern(5, walking_speed=0.4)
print(f"Generated walk pattern for {len(walking_params['left_footsteps']) + len(walking_params['right_footsteps'])} steps")
```

### Inverted Pendulum Model

The inverted pendulum model is fundamental to humanoid balance control:

```python
class InvertedPendulumController:
    def __init__(self, com_height=0.8, gravity=9.81):
        self.com_height = com_height  # Height of center of mass
        self.gravity = gravity
        self.omega = np.sqrt(gravity / com_height)  # Natural frequency

    def calculate_zmp_from_com(self, com_pos, com_vel, com_acc):
        """
        Calculate ZMP from center of mass state.

        Args:
            com_pos: Center of mass position [x, y, z]
            com_vel: Center of mass velocity [vx, vy, vz]
            com_acc: Center of mass acceleration [ax, ay, az]

        Returns:
            ZMP position [x, y]
        """
        zmp_x = com_pos[0] - (self.com_height * com_acc[0]) / (self.gravity + com_acc[2])
        zmp_y = com_pos[1] - (self.com_height * com_acc[1]) / (self.gravity + com_acc[2])

        return np.array([zmp_x, zmp_y])

    def calculate_com_from_zmp_trajectory(self, zmp_trajectory, timesteps):
        """
        Calculate CoM trajectory from desired ZMP trajectory using the inverted pendulum model.

        Args:
            zmp_trajectory: Desired ZMP trajectory
            timesteps: Time steps for the trajectory

        Returns:
            CoM trajectory
        """
        com_trajectory = []
        dt = timesteps[1] - timesteps[0] if len(timesteps) > 1 else 0.1

        # Initial conditions
        com_pos = np.array([zmp_trajectory[0][0], zmp_trajectory[0][1], self.com_height])
        com_vel = np.zeros(3)
        com_acc = np.zeros(3)

        for i, zmp in enumerate(zmp_trajectory):
            # Simple integration to get CoM trajectory from ZMP
            # This is a simplified approach; full solution requires more complex dynamics
            zmp_x, zmp_y = zmp[0], zmp[1]

            # Calculate CoM acceleration based on ZMP
            com_acc[0] = self.gravity / self.com_height * (com_pos[0] - zmp_x)
            com_acc[1] = self.gravity / self.com_height * (com_pos[1] - zmp_y)
            # z acceleration is mainly gravity effect

            # Integrate to get velocity and position
            com_vel += com_acc * dt
            com_pos += com_vel * dt

            com_trajectory.append(com_pos.copy())

        return np.array(com_trajectory)

    def generate_com_trajectory_for_step(self, start_pos, end_pos, step_duration):
        """
        Generate CoM trajectory for a single step using 5th order polynomial.

        Args:
            start_pos: Starting CoM position
            end_pos: Ending CoM position
            step_duration: Duration of the step

        Returns:
            Time vector and CoM trajectory
        """
        # 5th order polynomial for smooth trajectory
        t = np.linspace(0, step_duration, int(step_duration * 100))  # 100 Hz sampling

        # Calculate polynomial coefficients for smooth transition
        # s(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
        a0 = start_pos
        a1 = 0  # Start with zero velocity
        a2 = 0  # Start with zero acceleration
        a5 = 6 * (end_pos - start_pos) / (step_duration ** 5)
        a4 = -15 * (end_pos - start_pos) / (step_duration ** 4)
        a3 = 10 * (end_pos - start_pos) / (step_duration ** 3)

        # Calculate trajectory
        com_trajectory = []
        for ti in t:
            si = a0 + a1*ti + a2*ti**2 + a3*ti**3 + a4*ti**4 + a5*ti**5
            com_trajectory.append(si)

        return t, np.array(com_trajectory)
```

## Walking Pattern Generation

### Footstep Planning

Generating stable walking patterns:

```python
class FootstepPlanner:
    def __init__(self, step_length=0.3, step_width=0.2, max_turn_rate=0.2):
        self.step_length = step_length
        self.step_width = step_width
        self.max_turn_rate = max_turn_rate  # rad/s
        self.support_polygon_margin = 0.05

    def plan_footsteps(self, start_pos, goal_pos, start_orientation=0.0, goal_orientation=None):
        """
        Plan a sequence of footsteps from start to goal position.

        Args:
            start_pos: Starting position [x, y]
            goal_pos: Goal position [x, y]
            start_orientation: Starting orientation (radians)
            goal_orientation: Goal orientation (radians), if None, keep same as start

        Returns:
            List of footsteps [(position, orientation), ...]
        """
        if goal_orientation is None:
            goal_orientation = start_orientation

        # Calculate total displacement
        dx = goal_pos[0] - start_pos[0]
        dy = goal_pos[1] - start_pos[1]
        total_distance = np.sqrt(dx**2 + dy**2)

        # Calculate number of steps needed
        num_steps = int(np.ceil(total_distance / self.step_length))

        if num_steps == 0:
            return []

        # Calculate step vector
        step_dx = dx / num_steps
        step_dy = dy / num_steps

        footsteps = []
        current_pos = np.array(start_pos)
        current_orientation = start_orientation

        for i in range(num_steps):
            # Alternate between left and right foot
            foot_offset = self.step_width / 2 if i % 2 == 0 else -self.step_width / 2

            # Calculate foot position with alternating offset
            foot_pos = current_pos.copy()
            foot_pos[1] += foot_offset

            # Calculate orientation change
            if i == num_steps - 1:
                # Last step, set to goal orientation
                foot_orientation = goal_orientation
            else:
                # Gradually change orientation
                orientation_change = (goal_orientation - start_orientation) / num_steps
                foot_orientation = start_orientation + orientation_change * (i + 1)

            footsteps.append({
                'position': foot_pos,
                'orientation': foot_orientation,
                'foot': 'left' if i % 2 == 0 else 'right',
                'step_number': i
            })

            # Update current position for next step
            current_pos[0] += step_dx
            current_pos[1] += step_dy

        return footsteps

    def plan_omni_directional_steps(self, start_pos, goal_pos, start_orientation=0.0):
        """
        Plan footsteps for omnidirectional walking (forward, backward, sideways).
        """
        # Calculate displacement vector
        displacement = np.array(goal_pos) - np.array(start_pos)
        distance = np.linalg.norm(displacement)

        if distance < 0.05:  # If very close, no steps needed
            return []

        # Normalize displacement vector
        direction = displacement / distance

        # Determine walking direction (forward, backward, left, right)
        forward_dir = np.array([np.cos(start_orientation), np.sin(start_orientation)])
        side_dir = np.array([-np.sin(start_orientation), np.cos(start_orientation)])

        forward_projection = np.dot(direction, forward_dir)
        side_projection = np.dot(direction, side_dir)

        # Calculate number of steps based on direction
        if abs(forward_projection) > abs(side_projection):
            # Mainly forward/backward motion
            step_count = int(np.ceil(abs(forward_projection) * distance / self.step_length))
            step_direction = 'forward' if forward_projection > 0 else 'backward'
        else:
            # Mainly lateral motion
            step_count = int(np.ceil(abs(side_projection) * distance / self.step_width))
            step_direction = 'left' if side_projection > 0 else 'right'

        footsteps = []
        current_pos = np.array(start_pos)

        for i in range(step_count):
            # Calculate step offset based on direction
            if step_direction == 'forward':
                step_offset = forward_dir * self.step_length * (1 if forward_projection > 0 else -1)
            elif step_direction == 'backward':
                step_offset = forward_dir * self.step_length * (1 if forward_projection < 0 else -1)
            elif step_direction == 'left':
                step_offset = side_dir * self.step_width * (1 if side_projection > 0 else -1)
            else:  # right
                step_offset = side_dir * self.step_width * (1 if side_projection < 0 else -1)

            # Alternate feet
            foot_offset = self.step_width / 2 if i % 2 == 0 else -self.step_width / 2
            foot_pos = current_pos + step_offset
            foot_pos[1] += foot_offset

            footsteps.append({
                'position': foot_pos,
                'orientation': start_orientation,
                'foot': 'left' if i % 2 == 0 else 'right',
                'step_number': i
            })

            current_pos = foot_pos

        return footsteps
```

### Trajectory Generation

Creating smooth trajectories for walking:

```python
class WalkingTrajectoryGenerator:
    def __init__(self, com_height=0.8, step_height=0.05, step_duration=1.0):
        self.com_height = com_height
        self.step_height = step_height
        self.step_duration = step_duration
        self.inverted_pendulum = InvertedPendulumController(com_height)

    def generate_swing_foot_trajectory(self, start_pos, end_pos, step_height=None):
        """
        Generate trajectory for swing foot during a step.

        Args:
            start_pos: Starting foot position
            end_pos: Ending foot position
            step_height: Maximum step height (if None, use default)

        Returns:
            Array of foot positions over time
        """
        if step_height is None:
            step_height = self.step_height

        # Create time vector
        t = np.linspace(0, self.step_duration, int(self.step_duration * 100))  # 100 Hz

        # Generate 3D trajectory
        trajectory = []
        for ti in t:
            # Interpolate x and y positions
            progress = ti / self.step_duration
            x = start_pos[0] + (end_pos[0] - start_pos[0]) * progress
            y = start_pos[1] + (end_pos[1] - start_pos[1]) * progress

            # Calculate z position with parabolic trajectory
            z = start_pos[2]
            if progress < 0.5:
                # Rising phase
                z = start_pos[2] + step_height * (1 - np.cos(np.pi * progress * 2)) / 2
            else:
                # Falling phase
                z = start_pos[2] + step_height * (1 - np.cos(np.pi * (2 - progress * 2))) / 2

            trajectory.append([x, y, z])

        return np.array(trajectory)

    def generate_com_trajectory(self, footsteps, support_switch_times=None):
        """
        Generate center of mass trajectory following footsteps.

        Args:
            footsteps: List of footsteps from footstep planner
            support_switch_times: Times when support foot switches

        Returns:
            CoM trajectory over time
        """
        if support_switch_times is None:
            # Default: switch support foot halfway through each step
            support_switch_times = [i * self.step_duration + self.step_duration/2
                                  for i in range(len(footsteps))]

        # Create overall time vector
        total_time = len(footsteps) * self.step_duration
        t = np.linspace(0, total_time, int(total_time * 100))  # 100 Hz

        # Initialize CoM trajectory
        com_trajectory = []
        z_positions = []

        for ti in t:
            # Determine which step we're in
            step_idx = int(ti / self.step_duration)
            if step_idx >= len(footsteps):
                step_idx = len(footsteps) - 1

            # Calculate CoM position based on current support foot
            if step_idx < len(footsteps):
                support_foot_pos = footsteps[step_idx]['position']
                # CoM should be over the support foot with some stability margin
                com_x = support_foot_pos[0]
                com_y = support_foot_pos[1]
                com_z = self.com_height  # Maintain constant height

                # Add small adjustments for smooth transitions
                if step_idx > 0:
                    prev_support_pos = footsteps[step_idx - 1]['position']
                    transition_progress = (ti % self.step_duration) / self.step_duration
                    # Smooth transition between support positions
                    com_x = prev_support_pos[0] + (support_foot_pos[0] - prev_support_pos[0]) * transition_progress
                    com_y = prev_support_pos[1] + (support_foot_pos[1] - prev_support_pos[1]) * transition_progress

                com_trajectory.append([com_x, com_y, com_z])
                z_positions.append(com_z)

        return np.array(com_trajectory), t

    def generate_ankle_trajectories(self, footsteps):
        """
        Generate ankle joint trajectories to follow the footsteps.

        Args:
            footsteps: List of footsteps

        Returns:
            Dictionary of ankle trajectories for each leg
        """
        # This would generate joint space trajectories for the legs
        # to achieve the desired foot positions
        left_ankle_traj = []
        right_ankle_traj = []

        for i, step in enumerate(footsteps):
            foot_pos = step['position']
            foot_orientation = step['orientation']

            # Convert to ankle position (simplified)
            ankle_pos = foot_pos.copy()
            ankle_pos[2] = 0.1  # Ankle height above ground

            if step['foot'] == 'left':
                left_ankle_traj.append(ankle_pos)
            else:
                right_ankle_traj.append(ankle_pos)

        return {
            'left_ankle': left_ankle_traj,
            'right_ankle': right_ankle_traj
        }
```

## ROS 2 Navigation Integration

### Humanoid Navigation Node

Integrating navigation with humanoid-specific constraints:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import String, Bool
from visualization_msgs.msg import MarkerArray
from tf2_ros import TransformListener, Buffer
import numpy as np
from scipy.spatial import distance

class HumanoidNavigationNode(Node):
    def __init__(self):
        super().__init__('humanoid_navigation_node')

        # Initialize components
        self.footstep_planner = FootstepPlanner()
        self.trajectory_generator = WalkingTrajectoryGenerator()
        self.inverted_pendulum = InvertedPendulumController()

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers
        self.local_plan_pub = self.create_publisher(Path, '/local_plan', 10)
        self.global_plan_pub = self.create_publisher(Path, '/global_plan', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/navigation_status', 10)
        self.visualization_pub = self.create_publisher(MarkerArray, '/navigation_markers', 10)

        # Subscribers
        self.goal_sub = self.create_subscription(
            PoseStamped, '/move_base_simple/goal', self.goal_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10
        )

        # Service servers
        self.nav_service = self.create_service(
            PoseStamped, '/navigate_to_pose', self.navigate_to_pose_callback
        )

        # Timers
        self.navigation_timer = self.create_timer(0.1, self.navigation_loop)

        # State variables
        self.current_pose = None
        self.current_goal = None
        self.global_path = None
        self.local_path = None
        self.navigation_active = False
        self.footsteps = []
        self.current_step_index = 0
        self.map_data = None
        self.scan_data = None

        self.get_logger().info('Humanoid Navigation Node initialized')

    def goal_callback(self, msg):
        """Handle navigation goal."""
        self.get_logger().info(f'Received navigation goal: {msg.pose.position.x}, {msg.pose.position.y}')
        self.current_goal = msg.pose
        self.plan_path()

    def odom_callback(self, msg):
        """Handle odometry updates."""
        self.current_pose = msg.pose.pose

    def scan_callback(self, msg):
        """Handle laser scan data."""
        self.scan_data = msg

    def map_callback(self, msg):
        """Handle map updates."""
        self.map_data = msg

    def navigate_to_pose_callback(self, request, response):
        """Service callback for navigation."""
        self.current_goal = request.pose
        self.plan_path()
        self.navigation_active = True

        response = String()
        response.data = "Navigation started"
        return response

    def plan_path(self):
        """Plan path from current pose to goal."""
        if self.current_pose is None or self.current_goal is None:
            return

        # Get current position
        current_pos = [self.current_pose.position.x, self.current_pose.position.y]
        goal_pos = [self.current_goal.position.x, self.current_goal.position.y]

        # Plan footsteps using footstep planner
        self.footsteps = self.footstep_planner.plan_footsteps(
            current_pos, goal_pos,
            start_orientation=self.get_yaw_from_quaternion(self.current_pose.orientation)
        )

        # Generate detailed trajectories
        self.local_path, _ = self.trajectory_generator.generate_com_trajectory(self.footsteps)

        # Publish visualization
        self.publish_path_visualization()

        self.get_logger().info(f'Planned {len(self.footsteps)} footsteps to goal')

    def get_yaw_from_quaternion(self, quaternion):
        """Extract yaw angle from quaternion."""
        siny_cosp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        return np.arctan2(siny_cosp, cosy_cosp)

    def navigation_loop(self):
        """Main navigation control loop."""
        if not self.navigation_active or not self.footsteps or self.current_step_index >= len(self.footsteps):
            return

        # Get current step
        current_step = self.footsteps[self.current_step_index]

        # Check if we've reached the current step target
        if self.current_pose:
            current_pos = np.array([self.current_pose.position.x, self.current_pose.position.y])
            step_pos = np.array([current_step['position'][0], current_step['position'][1]])
            distance_to_step = np.linalg.norm(current_pos - step_pos)

            if distance_to_step < 0.1:  # Within 10cm of step target
                self.current_step_index += 1
                if self.current_step_index >= len(self.footsteps):
                    # Reached goal
                    self.navigation_complete()
                    return

        # Generate command to move toward next step
        cmd_vel = self.generate_step_command()
        self.cmd_vel_pub.publish(cmd_vel)

    def generate_step_command(self):
        """Generate velocity command for next step."""
        if self.current_step_index >= len(self.footsteps) or not self.current_pose:
            cmd = Twist()
            return cmd

        next_step = self.footsteps[self.current_step_index]
        current_pos = np.array([self.current_pose.position.x, self.current_pose.position.y])
        step_pos = np.array([next_step['position'][0], next_step['position'][1]])

        # Calculate direction to next step
        direction = step_pos - current_pos
        distance = np.linalg.norm(direction)

        cmd = Twist()
        if distance > 0.05:  # If more than 5cm away
            direction = direction / distance  # Normalize

            # Calculate desired velocity based on distance
            desired_speed = min(0.2, distance * 2)  # Max 0.2 m/s, scale with distance

            # Project desired velocity onto robot's coordinate frame
            robot_yaw = self.get_yaw_from_quaternion(self.current_pose.orientation)
            robot_direction_x = np.cos(robot_yaw)
            robot_direction_y = np.sin(robot_yaw)

            # Calculate forward and angular velocity
            forward_projection = direction[0] * robot_direction_x + direction[1] * robot_direction_y
            left_projection = -direction[0] * robot_direction_y + direction[1] * robot_direction_x

            cmd.linear.x = forward_projection * desired_speed
            cmd.linear.y = left_projection * desired_speed * 0.5  # Less lateral movement
            cmd.angular.z = np.arctan2(left_projection, forward_projection) * 0.5  # Turn toward goal

        return cmd

    def navigation_complete(self):
        """Handle navigation completion."""
        self.navigation_active = False
        self.current_step_index = 0
        self.footsteps = []

        # Stop robot
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)

        status_msg = String()
        status_msg.data = "Navigation completed successfully"
        self.status_pub.publish(status_msg)

        self.get_logger().info('Navigation completed')

    def publish_path_visualization(self):
        """Publish visualization markers for the planned path."""
        marker_array = MarkerArray()

        # Create markers for footsteps
        for i, step in enumerate(self.footsteps):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD

            marker.pose.position.x = step['position'][0]
            marker.pose.position.y = step['position'][1]
            marker.pose.position.z = 0.01  # Slightly above ground
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.1  # Diameter
            marker.scale.y = 0.1
            marker.scale.z = 0.02  # Height

            marker.color.a = 0.7  # Alpha
            marker.color.r = 1.0 if step['foot'] == 'left' else 0.0  # Red for left, Green for right
            marker.color.g = 0.0 if step['foot'] == 'left' else 1.0
            marker.color.b = 0.0

            marker_array.markers.append(marker)

        self.visualization_pub.publish(marker_array)

    def check_step_feasibility(self, step_pos):
        """Check if a step is feasible given current map and sensor data."""
        # Check if step location is in free space
        if self.map_data:
            map_x, map_y = self.world_to_map_coordinates(step_pos[0], step_pos[1])
            if 0 <= map_x < self.map_data.info.width and 0 <= map_y < self.map_data.info.height:
                map_index = map_y * self.map_data.info.width + map_x
                cell_value = self.map_data.data[map_index]

                # If cell value > 50, it's considered occupied
                if cell_value > 50:
                    return False

        # Check with laser scan for immediate obstacles
        if self.scan_data:
            # This would involve checking if the step location is clear of obstacles
            # based on the laser scan data
            pass

        return True

    def world_to_map_coordinates(self, x, y):
        """Convert world coordinates to map coordinates."""
        if self.map_data:
            map_x = int((x - self.map_data.info.origin.position.x) / self.map_data.info.resolution)
            map_y = int((y - self.map_data.info.origin.position.y) / self.map_data.info.resolution)
            return map_x, map_y
        return 0, 0
```

## Dynamic Balance Control

### Balance Feedback Controllers

Implementing feedback control for dynamic balance:

```python
class BalanceController:
    def __init__(self, com_height=0.8, gravity=9.81):
        self.com_height = com_height
        self.gravity = gravity
        self.omega = np.sqrt(gravity / com_height)

        # PID gains for balance control
        self.kp = 10.0   # Proportional gain
        self.ki = 1.0    # Integral gain
        self.kd = 5.0    # Derivative gain

        # Integral error accumulation
        self.integral_x = 0.0
        self.integral_y = 0.0

        # Previous errors for derivative calculation
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0

    def update_balance(self, measured_zmp, desired_zmp, dt=0.01):
        """
        Update balance control based on ZMP error.

        Args:
            measured_zmp: Measured Zero Moment Point [x, y]
            desired_zmp: Desired Zero Moment Point [x, y]
            dt: Time step

        Returns:
            Balance correction commands [x, y]
        """
        # Calculate errors
        error_x = desired_zmp[0] - measured_zmp[0]
        error_y = desired_zmp[1] - measured_zmp[1]

        # Update integral terms
        self.integral_x += error_x * dt
        self.integral_y += error_y * dt

        # Calculate derivative terms
        derivative_x = (error_x - self.prev_error_x) / dt if dt > 0 else 0
        derivative_y = (error_y - self.prev_error_y) / dt if dt > 0 else 0

        # Calculate control outputs using PID
        control_x = (self.kp * error_x +
                    self.ki * self.integral_x +
                    self.kd * derivative_x)

        control_y = (self.kp * error_y +
                    self.ki * self.integral_y +
                    self.kd * derivative_y)

        # Update previous errors
        self.prev_error_x = error_x
        self.prev_error_y = error_y

        return np.array([control_x, control_y])

    def calculate_desired_com_position(self, support_polygon_center, stability_margin=0.05):
        """
        Calculate desired CoM position based on support polygon.

        Args:
            support_polygon_center: Center of the support polygon
            stability_margin: Minimum distance from polygon edge

        Returns:
            Desired CoM position
        """
        # For bipedal walking, the CoM should be within the support polygon
        # with some stability margin
        desired_com = support_polygon_center.copy()

        # Add small adjustments to maintain balance
        # This could be based on walking gait, upcoming steps, etc.
        return desired_com

    def adjust_for_external_perturbation(self, force_vector, torque_vector):
        """
        Adjust balance control to counteract external perturbations.

        Args:
            force_vector: External force vector [fx, fy, fz]
            torque_vector: External torque vector [tx, ty, tz]

        Returns:
            Adjusted control commands
        """
        # Calculate the effect of external forces on balance
        # This is a simplified model - real implementation would be more complex
        force_effect_x = force_vector[0] * self.com_height / self.gravity
        force_effect_y = force_vector[1] * self.com_height / self.gravity

        # Adjust control to counteract the perturbation
        adjustment_x = -force_effect_x * 0.5  # Scaling factor
        adjustment_y = -force_effect_y * 0.5

        return np.array([adjustment_x, adjustment_y])
```

## Advanced Navigation Techniques

### Humanoid-Specific Path Planning

```python
class HumanoidPathPlanner:
    def __init__(self, step_length=0.3, step_width=0.2, turn_radius=0.5):
        self.step_length = step_length
        self.step_width = step_width
        self.turn_radius = turn_radius
        self.min_clearance = 0.3  # Minimum clearance for humanoid

    def plan_humanoid_path(self, start_pose, goal_pose, occupancy_map):
        """
        Plan a path specifically for humanoid locomotion considering step constraints.

        Args:
            start_pose: Starting pose
            goal_pose: Goal pose
            occupancy_map: Occupancy grid map

        Returns:
            List of waypoints for humanoid navigation
        """
        # This would implement a path planner that considers:
        # - Minimum turning radius for humanoid
        # - Step length and width constraints
        # - Required clearance for bipedal walking
        # - Dynamic balance requirements

        # Simplified implementation - in practice, this would use
        # specialized algorithms like Footstep-based A* or other
        # humanoid-aware path planning algorithms
        waypoints = self.generate_waypoints_with_constraints(
            start_pose, goal_pose, occupancy_map
        )

        return waypoints

    def generate_waypoints_with_constraints(self, start_pose, goal_pose, occupancy_map):
        """Generate waypoints respecting humanoid constraints."""
        # Calculate straight-line path first
        start_pos = np.array([start_pose.position.x, start_pose.position.y])
        goal_pos = np.array([goal_pose.position.x, goal_pose.position.y])

        # Generate intermediate waypoints
        direction = goal_pos - start_pos
        distance = np.linalg.norm(direction)
        num_waypoints = max(2, int(distance / (self.step_length * 0.8)))  # 80% step length overlap

        waypoints = []
        for i in range(num_waypoints + 1):
            progress = i / num_waypoints if num_waypoints > 0 else 0
            pos = start_pos + direction * progress

            # Check if this position is feasible for humanoid
            if self.is_position_feasible_for_humanoid(pos, occupancy_map):
                waypoint = PoseStamped()
                waypoint.pose.position.x = pos[0]
                waypoint.pose.position.y = pos[1]
                waypoint.pose.position.z = start_pose.position.z  # Maintain height
                waypoints.append(waypoint)

        return waypoints

    def is_position_feasible_for_humanoid(self, position, occupancy_map):
        """Check if a position is feasible for humanoid navigation."""
        # Check if the position itself is free
        if not self.is_free_space(position, occupancy_map):
            return False

        # Check if there's enough space for humanoid steps around this position
        # Consider the step width and potential for side-stepping
        step_area = [
            position + np.array([0, self.step_width/2]),  # Left foot position
            position + np.array([0, -self.step_width/2])  # Right foot position
        ]

        for pos in step_area:
            if not self.is_free_space(pos, occupancy_map, clearance=self.min_clearance):
                return False

        return True

    def is_free_space(self, position, occupancy_map, clearance=0.1):
        """Check if space around position is free."""
        # This would check the occupancy map for obstacles
        # considering the specified clearance
        if occupancy_map is None:
            return True

        # Convert world position to map coordinates
        map_x, map_y = self.world_to_map_coordinates(
            position[0], position[1], occupancy_map
        )

        # Check a small area around the position
        clearance_cells = int(clearance / occupancy_map.info.resolution)
        for dx in range(-clearance_cells, clearance_cells + 1):
            for dy in range(-clearance_cells, clearance_cells + 1):
                check_x, check_y = map_x + dx, map_y + dy
                if (0 <= check_x < occupancy_map.info.width and
                    0 <= check_y < occupancy_map.info.height):
                    idx = check_y * occupancy_map.info.width + check_x
                    if idx < len(occupancy_map.data) and occupancy_map.data[idx] > 50:
                        return False  # Occupied space found

        return True

    def world_to_map_coordinates(self, x, y, occupancy_map):
        """Convert world coordinates to map coordinates."""
        map_x = int((x - occupancy_map.info.origin.position.x) / occupancy_map.info.resolution)
        map_y = int((y - occupancy_map.info.origin.position.y) / occupancy_map.info.resolution)
        return map_x, map_y
```

## Quality Assessment and Evaluation

### Walking Performance Metrics

```python
class WalkingPerformanceEvaluator:
    def __init__(self):
        self.step_success_count = 0
        self.step_attempt_count = 0
        self.balance_margins = []
        self.energy_consumption = []
        self.walking_speeds = []

    def evaluate_step(self, step_executed, step_successful, balance_margin, energy_used):
        """
        Evaluate a single walking step.

        Args:
            step_executed: Information about the step executed
            step_successful: Whether the step was successful
            balance_margin: Balance margin during the step
            energy_used: Energy consumed during the step

        Returns:
        """
        if step_successful:
            self.step_success_count += 1

        self.step_attempt_count += 1
        self.balance_margins.append(balance_margin)
        self.energy_consumption.append(energy_used)

    def evaluate_walking_sequence(self, step_sequence, success_sequence, balance_sequence, energy_sequence):
        """
        Evaluate a sequence of walking steps.

        Args:
            step_sequence: List of steps executed
            success_sequence: List of success indicators
            balance_sequence: List of balance margins
            energy_sequence: List of energy consumption values

        Returns:
            Dictionary of performance metrics
        """
        if not success_sequence:
            return {}

        # Calculate metrics
        success_rate = sum(success_sequence) / len(success_sequence)
        avg_balance_margin = np.mean(balance_sequence) if balance_sequence else 0
        avg_energy = np.mean(energy_sequence) if energy_sequence else 0
        std_balance = np.std(balance_sequence) if balance_sequence else 0

        # Calculate step timing consistency
        if len(step_sequence) > 1:
            step_durations = [step['duration'] for step in step_sequence if 'duration' in step]
            timing_consistency = 1.0 - (np.std(step_durations) / np.mean(step_durations)) if step_durations else 1.0
        else:
            timing_consistency = 1.0

        metrics = {
            'success_rate': success_rate,
            'average_balance_margin': avg_balance_margin,
            'average_energy_per_step': avg_energy,
            'balance_stability': 1.0 - std_balance,  # Lower std = more stable
            'timing_consistency': timing_consistency,
            'efficiency_score': success_rate * avg_balance_margin / (avg_energy + 0.1)  # Higher is better
        }

        return metrics

    def generate_performance_report(self):
        """Generate a comprehensive performance report."""
        if self.step_attempt_count == 0:
            return "No steps evaluated yet."

        success_rate = self.step_success_count / self.step_attempt_count if self.step_attempt_count > 0 else 0
        avg_balance = np.mean(self.balance_margins) if self.balance_margins else 0
        avg_energy = np.mean(self.energy_consumption) if self.energy_consumption else 0
        std_balance = np.std(self.balance_margins) if self.balance_margins else 0

        report = f"""
        Walking Performance Report:
        - Total steps: {self.step_attempt_count}
        - Successful steps: {self.step_success_count}
        - Success rate: {success_rate:.2%}
        - Average balance margin: {avg_balance:.3f}m
        - Balance stability (std): {std_balance:.3f}m
        - Average energy per step: {avg_energy:.3f}J
        """

        return report
```

## Safety and Robustness

### Fall Prevention and Recovery

```python
class FallPreventionSystem:
    def __init__(self, balance_controller, max_lean_angle=15.0, recovery_timeout=2.0):
        self.balance_controller = balance_controller
        self.max_lean_angle = np.radians(max_lean_angle)  # Convert to radians
        self.recovery_timeout = recovery_timeout
        self.in_recovery_mode = False
        self.recovery_start_time = None

    def check_stability(self, current_orientation, zmp_position, com_position):
        """
        Check if the humanoid is in a stable state.

        Args:
            current_orientation: Current body orientation
            zmp_position: Current Zero Moment Point
            com_position: Current Center of Mass position

        Returns:
            Stability status and recommended action
        """
        # Check orientation limits
        roll, pitch, _ = self.orientation_to_rpy(current_orientation)

        orientation_stable = (abs(roll) < self.max_lean_angle and
                             abs(pitch) < self.max_lean_angle)

        # Check ZMP within support polygon
        zmp_stable = self.is_zmp_stable(zmp_position, com_position)

        overall_stable = orientation_stable and zmp_stable

        if not overall_stable:
            if not self.in_recovery_mode:
                self.start_recovery_procedure()
                action = "recovery"
            else:
                action = "emergency_stop"
        else:
            if self.in_recovery_mode:
                self.end_recovery_procedure()
            action = "continue"

        return {
            'stable': overall_stable,
            'orientation_stable': orientation_stable,
            'zmp_stable': zmp_stable,
            'recommended_action': action
        }

    def orientation_to_rpy(self, orientation):
        """Convert quaternion to roll-pitch-yaw."""
        import math
        w, x, y, z = orientation.w, orientation.x, orientation.y, orientation.z

        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def is_zmp_stable(self, zmp_position, com_position):
        """Check if ZMP is within acceptable bounds."""
        # Simplified check - in reality, this would check against support polygon
        # which changes based on foot positions
        support_margin = 0.05  # 5cm safety margin

        # This is a simplified check - real implementation would be more complex
        # and consider the actual support polygon based on foot positions
        return True

    def start_recovery_procedure(self):
        """Initiate balance recovery procedure."""
        self.in_recovery_mode = True
        self.recovery_start_time = self.get_current_time()

        # Implement recovery strategy
        # This might include: stepping, crouching, arm movements, etc.
        self.execute_balance_recovery()

    def execute_balance_recovery(self):
        """Execute specific balance recovery actions."""
        # This would implement specific recovery strategies such as:
        # - Taking a quick step in the direction of the fall
        # - Adjusting CoM position
        # - Using arms for balance
        # - Reducing walking speed
        pass

    def end_recovery_procedure(self):
        """End balance recovery procedure."""
        self.in_recovery_mode = False
        self.recovery_start_time = None

    def get_current_time(self):
        """Get current time in seconds."""
        import time
        return time.time()
```

## Best Practices

### Design Principles

1. **Stability First**: Always prioritize dynamic balance over speed
2. **Gradual Adaptation**: Start with simple walking patterns and increase complexity
3. **Sensor Integration**: Use multiple sensors for robust perception
4. **Safety Margins**: Maintain adequate safety margins in all calculations
5. **Modular Design**: Design components as independent, testable modules

### Implementation Guidelines

1. **Simulation First**: Test extensively in simulation before hardware
2. **Parameter Tuning**: Carefully tune control parameters based on robot dynamics
3. **State Machines**: Use state machines to manage different walking phases
4. **Logging**: Log all relevant data for analysis and debugging
5. **Fallback Behaviors**: Implement safe fallback behaviors for failures

## Key Takeaways

- Humanoid locomotion requires sophisticated balance control algorithms
- ZMP-based control is fundamental to stable bipedal walking
- Path planning must consider humanoid-specific constraints
- Safety and fall prevention are critical in humanoid navigation
- Integration of perception, planning, and control is essential

## Practice Exercises

### Exercise 1: Walking Pattern Generation
1. Implement a footstep planner that generates stable walking patterns for a humanoid robot.
2. Test the planner with different walking speeds and turning radii.
3. Verify that the generated footsteps maintain dynamic balance during walking.
4. Evaluate the stability margins for different walking parameters.

### Exercise 2: Inverted Pendulum Control
1. Implement an inverted pendulum controller for humanoid balance.
2. Test the controller with different center of mass heights and walking speeds.
3. Evaluate the controller's response to external disturbances.
4. Compare the performance of different control strategies (PID, LQR, etc.).

### Exercise 3: ZMP-Based Walking
1. Implement a ZMP tracking controller for stable bipedal walking.
2. Test the controller with various walking patterns and terrain conditions.
3. Measure the tracking accuracy of the ZMP controller.
4. Analyze the relationship between ZMP tracking error and walking stability.

### Exercise 4: Humanoid Navigation Integration
1. Integrate the walking controller with a navigation system for goal-directed locomotion.
2. Implement obstacle avoidance during walking using sensor feedback.
3. Test navigation performance in cluttered environments.
4. Evaluate the system's ability to handle dynamic obstacles.

### Discussion Questions
1. What are the main challenges in maintaining balance during dynamic humanoid locomotion?
2. How does the Zero Moment Point (ZMP) concept enable stable bipedal walking?
3. What are the trade-offs between walking speed and stability in humanoid robots?
4. How can sensory feedback improve the robustness of humanoid locomotion?

### Challenge Exercise
Design and implement a complete humanoid locomotion system:
- Integrate footstep planning with dynamic balance control
- Implement ZMP-based walking with real-time balance feedback
- Create a navigation system that works with humanoid-specific constraints
- Add fall prevention and recovery mechanisms
- Test the complete system in simulation with various terrain types and obstacles
- Evaluate the system's performance in terms of stability, speed, and safety
- Document the control architecture, parameters, and experimental results

## References

[Humanoid Locomotion Bibliography](/docs/references/humanoid-locomotion-bibliography.md)