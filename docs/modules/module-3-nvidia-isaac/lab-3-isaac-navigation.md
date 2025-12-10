---
title: Lab 3 - Isaac Navigation Exercise
sidebar_position: 3
week: 7
module: module-3-nvidia-isaac
learningObjectives:
  - Set up Isaac ROS navigation environment
  - Configure GPU-accelerated SLAM system
  - Implement Isaac navigation stack with real sensors
  - Test navigation in Isaac Sim environment
  - Compare Isaac navigation performance with standard ROS 2 navigation
prerequisites:
  - Week 1-7 content: Complete textbook modules
  - NVIDIA GPU with CUDA support
  - Isaac ROS installed and configured
  - Isaac Sim environment
  - Basic understanding of navigation concepts
description: Hands-on lab to implement and test NVIDIA Isaac navigation systems
---

# Lab 3: Isaac Navigation Exercise

## Learning Objectives

After completing this lab, you will be able to:
- Set up Isaac ROS navigation environment with GPU acceleration
- Configure and run GPU-accelerated SLAM system
- Implement Isaac navigation stack with sensor integration
- Test navigation performance in Isaac Sim
- Compare Isaac navigation with standard ROS 2 navigation

## Lab Duration

Estimated time: 150 minutes

## Prerequisites

- NVIDIA GPU with CUDA support (Compute Capability 6.0+)
- Isaac ROS installed (Humble Hawksbill)
- Isaac Sim installed
- ROS 2 Navigation2 stack
- Basic knowledge of Linux command line
- Understanding of navigation concepts

## Scenario: Warehouse Robot Navigation with Isaac

You are developing a warehouse robot that needs to navigate efficiently using NVIDIA Isaac's GPU-accelerated navigation capabilities. In this lab, you'll set up Isaac navigation, configure SLAM, and test navigation performance in a simulated warehouse environment.

## Step 1: Verify Isaac ROS Installation

First, verify that Isaac ROS is properly installed:

```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Check Isaac ROS packages
apt list --installed | grep isaac

# Verify GPU and CUDA
nvidia-smi
nvcc --version

# Check Isaac ROS nodes
ros2 pkg list | grep isaac
```

## Step 2: Create Isaac Navigation Workspace

Create a workspace for Isaac navigation:

```bash
mkdir -p ~/isaac_navigation_ws/src
cd ~/isaac_navigation_ws
source /opt/ros/humble/setup.bash
```

## Step 3: Create Isaac Navigation Package

Create a package for our Isaac navigation implementation:

```bash
cd ~/isaac_navigation_ws/src
ros2 pkg create --build-type ament_python isaac_navigation_lab --dependencies rclpy std_msgs geometry_msgs sensor_msgs nav_msgs visualization_msgs cv_bridge
```

## Step 4: Create Isaac SLAM Configuration

Create a configuration directory:

```bash
mkdir -p ~/isaac_navigation_ws/src/isaac_navigation_lab/isaac_navigation_lab/config
```

Create SLAM configuration file `isaac_visual_slam_config.yaml`:

```bash
nano ~/isaac_navigation_ws/src/isaac_navigation_lab/isaac_navigation_lab/config/isaac_visual_slam_config.yaml
```

Add the following content:

```yaml
# Isaac Visual SLAM Configuration
/**:
  ros__parameters:
    # Camera parameters
    rectified_images: true
    image_width: 640
    image_height: 480

    # GPU acceleration parameters
    enable_gpu_acceleration: true
    max_num_corners: 1000
    min_tracked_features: 50

    # SLAM parameters
    map_frame: "map"
    odom_frame: "odom"
    base_frame: "base_link"

    # Loop closure parameters
    enable_loop_closure: true
    loop_closure_threshold: 0.5

    # Optimization parameters
    num_frames_threshold: 10
    num_corners_threshold: 50
    min_num_stereo_matches: 20

    # IMU integration
    use_imu: true
    imu_topic: "/imu/data"
```

## Step 5: Create Isaac Navigation Configuration

Create navigation configuration file `isaac_navigation_config.yaml`:

```bash
nano ~/isaac_navigation_ws/src/isaac_navigation_lab/isaac_navigation_lab/config/isaac_navigation_config.yaml
```

Add the following content:

```yaml
amcl:
  ros__parameters:
    use_sim_time: True
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_footprint"
    beam_span: 0.0628318530718
    beta: 0.0174532925199
    first_map_only: False
    lambda_short: 0.1
    likelihood_max_dist: 2.0
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_rate: 0.5
    scan_topic: scan
    sigma_hit: 0.2
    tf_broadcast: True
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.2
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05
    map_topic: map
    initial_pose:
      x: 0.0
      y: 0.0
      z: 0.0
      yaw: 0.0

bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: True
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    navigate_through_poses: False
    navigate_to_pose: True
    action_server_result_timeout: 900.0
    simple_goal_checker:
      plugin: "nav2_navfn_planner::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: True
    goal_blackboard_visibility:
      plugin: "nav2_behavior_tree::GoalBlackboardPlugin"
    progress_checker:
      plugin: "nav2_navfn_planner::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0
    behavior_tree:
      plugin: "nav2_behavior_tree::NavigateToPoseTolerance"
      tree_filename: "navigate_to_pose_w_replanning_and_recovery.xml"

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    # Isaac-specific controller parameters
    FollowPath:
      plugin: "nav2_rotation_shim::RotationShimController"
      subscription_topic: "local_plan"
      rotation:
        plugin: "nav2_controller::SimplePurePursuit"
        velocity_topic: "cmd_vel"
        lookahead_dist: 0.6
        transform_tolerance: 0.1
        velocity_scaling_smooth: 1.0
        velocity_scaling_min_abs: 0.0
        velocity_scaling_error_abs: 0.0
        velocity_scaling_error_sq: 0.0
        velocity_scaling_factor: 1.0
        min_approach_linear_velocity: 0.05
        approach_velocity_scaling_factor: 1.0

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 10.0  # Higher frequency for Isaac
      publish_frequency: 5.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: True
      rolling_window: true
      width: 5  # Larger window for Isaac
      height: 5
      resolution: 0.05
      robot_radius: 0.22
      plugins: ["voxel_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: False
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 16
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 5.0  # Extended range for Isaac
          raytrace_min_range: 0.0
          obstacle_max_range: 4.0  # Extended range for Isaac
          obstacle_min_range: 0.0
      always_send_full_costmap: True
  local_costmap_client:
    ros__parameters:
      use_sim_time: True
  local_costmap_rclcpp_node:
    ros__parameters:
      use_sim_time: True

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 2.0  # Lower frequency for global map
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: True
      robot_radius: 0.22
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 5.0
          raytrace_min_range: 0.0
          obstacle_max_range: 4.0
          obstacle_min_range: 0.0
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      always_send_full_costmap: True
  global_costmap_client:
    ros__parameters:
      use_sim_time: True
  global_costmap_rclcpp_node:
    ros__parameters:
      use_sim_time: True

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0  # Higher frequency for Isaac
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

velocity_smoother:
  ros__parameters:
    use_sim_time: True
    smoothing_frequency: 20.0
    scale_velocities: False
    velocity_timeout: 1.0
    filter_size: 5
    velocity_scaling: 1.0
    max_velocity: [0.8, 0.0, 1.0]  # Adjusted for Isaac performance
    min_velocity: [-0.8, 0.0, -1.0]
    max_accel: [2.5, 0.0, 3.2]
    max_decel: [-2.5, 0.0, -3.2]
```

## Step 6: Create Isaac Navigation Launch File

Create a launch directory:

```bash
mkdir -p ~/isaac_navigation_ws/src/isaac_navigation_lab/isaac_navigation_lab/launch
```

Create Isaac navigation launch file `isaac_navigation.launch.py`:

```bash
nano ~/isaac_navigation_ws/src/isaac_navigation_lab/isaac_navigation_lab/launch/isaac_navigation.launch.py
```

Add the following content:

```python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString, RewrittenYaml


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    params_file = LaunchConfiguration('params_file')
    bt_xml_file = LaunchConfiguration('bt_xml_file')
    autostart = LaunchConfiguration('autostart', default='true')
    use_composition = LaunchConfiguration('use_composition', default='False')
    use_respawn = LaunchConfiguration('use_respawn', default='False')
    log_level = LaunchConfiguration('log_level', default='info')

    # Package names
    pkg_isaac_navigation_lab = FindPackageShare('isaac_navigation_lab')
    pkg_nav2_bringup = FindPackageShare('nav2_bringup')

    # Paths
    default_params_file_path = PathJoinSubstitution([
        FindPackageShare('isaac_navigation_lab'),
        'config',
        'isaac_navigation_config.yaml'
    ])

    default_bt_xml_file_path = PathJoinSubstitution([
        FindPackageShare('nav2_bt_navigator'),
        'behavior_trees',
        'navigate_to_pose_w_replanning_and_recovery.xml'
    ])

    # Launch configuration variables
    default_nav_to_pose_bt_xml = PathJoinSubstitution([
        FindPackageShare('nav2_bt_navigator'),
        'behavior_trees',
        'navigate_to_pose_w_replanning_and_recovery.xml'
    ])

    default_inspect_bt_xml = PathJoinSubstitution([
        FindPackageShare('nav2_bt_navigator'),
        'behavior_trees',
        'navigate_to_pose_inspect.xml'
    ])

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file_path,
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )

    declare_bt_xml_cmd = DeclareLaunchArgument(
        'bt_xml_file',
        default_value=default_nav_to_pose_bt_xml,
        description='Full path to the behavior tree xml file to use'
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='False',
        description='Whether to use composed bringup'
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Whether to respawn if a node crashes'
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='log level'
    )

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'bt_xml_filename': bt_xml_file,
        'autostart': autostart
    }

    # Use custom configuration file
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True
    )

    # Nodes launching commands
    if use_composition:
        # TODO: Composed bringup not yet implemented for Isaac navigation
        pass
    else:
        # Isaac Visual SLAM node (if available)
        isaac_visual_slam_cmd = Node(
            condition=UnlessCondition(use_composition),
            name='isaac_visual_slam_node',
            package='isaac_ros_visual_slam',
            executable='visual_slam_node',
            parameters=[configured_params],
            remappings=[
                ('/visual_slam/camera/left/image_rect', '/camera/left/image_rect_color'),
                ('/visual_slam/camera/right/image_rect', '/camera/right/image_rect_color'),
                ('/visual_slam/camera/left/camera_info', '/camera/left/camera_info'),
                ('/visual_slam/camera/right/camera_info', '/camera/right/camera_info'),
            ],
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
        )

        # Navigation stack nodes
        navigation_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('nav2_bringup'),
                    'launch',
                    'navigation_launch.py'
                ])
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': configured_params,
                'use_composition': use_composition,
                'autostart': autostart,
                'use_respawn': use_respawn,
                'log_level': log_level
            }.items()
        )

    # Create launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)

    # Add nodes and launch descriptions
    ld.add_action(navigation_cmd)
    # ld.add_action(isaac_visual_slam_cmd)  # Uncomment when Isaac SLAM is available

    return ld
```

## Step 7: Create Isaac Performance Monitoring Node

Create a performance monitoring script to compare Isaac vs standard navigation:

```bash
nano ~/isaac_navigation_ws/src/isaac_navigation_lab/isaac_navigation_lab/isaac_performance_monitor.py
```

Add the following content:

```python
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


## Step 8: Create Isaac Sim World for Navigation Testing

Now create a warehouse world for Isaac navigation testing:

```bash
mkdir -p ~/isaac_navigation_ws/src/isaac_navigation_lab/isaac_navigation_lab/worlds
```

Create warehouse world file `warehouse_navigation.sdf`:

```bash
nano ~/isaac_navigation_ws/src/isaac_navigation_lab/isaac_navigation_lab/worlds/warehouse_navigation.sdf
```

Add the following content:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="warehouse_navigation">
    <!-- Physics engine -->
    <physics name="1ms" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- Scene configuration -->
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
    </scene>

    <!-- Sun light -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Warehouse structure -->
    <!-- Outer walls -->
    <model name="wall_north">
      <pose>0 10 1 0 0 0</pose>
      <link name="wall_link">
        <collision name="collision">
          <geometry>
            <box>
              <size>20 0.2 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>20 0.2 2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="wall_south">
      <pose>0 -10 1 0 0 0</pose>
      <link name="wall_link">
        <collision name="collision">
          <geometry>
            <box>
              <size>20 0.2 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>20 0.2 2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="wall_east">
      <pose>10 0 1 0 0 1.5707</pose>
      <link name="wall_link">
        <collision name="collision">
          <geometry>
            <box>
              <size>20 0.2 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>20 0.2 2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="wall_west">
      <pose>-10 0 1 0 0 1.5707</pose>
      <link name="wall_link">
        <collision name="collision">
          <geometry>
            <box>
              <size>20 0.2 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>20 0.2 2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Warehouse shelves -->
    <model name="shelf_1">
      <pose>-5 5 0.8 0 0 0</pose>
      <link name="shelf_link">
        <collision name="collision">
          <geometry>
            <box>
              <size>2 0.5 1.5</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>2 0.5 1.5</size>
            </box>
          </geometry>
          <material>
            <ambient>0.6 0.4 0.2 1</ambient>
            <diffuse>0.6 0.4 0.2 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="shelf_2">
      <pose>5 5 0.8 0 0 0</pose>
      <link name="shelf_link">
        <collision name="collision">
          <geometry>
            <box>
              <size>2 0.5 1.5</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>2 0.5 1.5</size>
            </box>
          </geometry>
          <material>
            <ambient>0.6 0.4 0.2 1</ambient>
            <diffuse>0.6 0.4 0.2 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="shelf_3">
      <pose>-5 -5 0.8 0 0 0</pose>
      <link name="shelf_link">
        <collision name="collision">
          <geometry>
            <box>
              <size>2 0.5 1.5</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>2 0.5 1.5</size>
            </box>
          </geometry>
          <material>
            <ambient>0.6 0.4 0.2 1</ambient>
            <diffuse>0.6 0.4 0.2 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="shelf_4">
      <pose>5 -5 0.8 0 0 0</pose>
      <link name="shelf_link">
        <collision name="collision">
          <geometry>
            <box>
              <size>2 0.5 1.5</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>2 0.5 1.5</size>
            </box>
          </geometry>
          <material>
            <ambient>0.6 0.4 0.2 1</ambient>
            <diffuse>0.6 0.4 0.2 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Navigation waypoints -->
    <model name="waypoint_1">
      <pose>-8 0 0.1 0 0 0</pose>
      <static>true</static>
      <link name="waypoint_link">
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.1</radius>
              <length>0.2</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0 1 0 1</ambient>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="waypoint_2">
      <pose>0 8 0.1 0 0 0</pose>
      <static>true</static>
      <link name="waypoint_link">
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.1</radius>
              <length>0.2</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0 1 0 1</ambient>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="waypoint_3">
      <pose>8 -8 0.1 0 0 0</pose>
      <static>true</static>
      <link name="waypoint_link">
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.1</radius>
              <length>0.2</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0 1 0 1</ambient>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Robot spawn location -->
    <model name="turtlebot3_waffle" static="false">
      <include>
        <uri>model://turtlebot3_waffle</uri>
      </include>
      <pose>-8 8 0.01 0 0 0</pose>
    </model>
  </world>
</sdf>
```

## Step 9: Create Isaac Navigation Test Script

Create a navigation test script to demonstrate Isaac navigation capabilities:

```bash
nano ~/isaac_navigation_ws/src/isaac_navigation_lab/isaac_navigation_lab/test_isaac_navigation.py
```

Add the following content:

```python
#!/usr/bin/env python3

"""
Isaac Navigation Test Script

This script demonstrates Isaac navigation capabilities by sending navigation goals
and comparing performance with standard navigation.
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Float32
import time


class IsaacNavigationTester(Node):
    """
    A node that tests Isaac navigation performance.
    """

    def __init__(self):
        super().__init__('isaac_navigation_tester')

        # Navigation action client
        self._action_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        # Performance monitoring
        self.performance_sub = self.create_subscription(
            Float32,
            '/performance/navigation_time',
            self.performance_callback,
            10
        )

        self.navigation_times = []
        self.current_goal_id = 0
        self.max_goals = 5

        self.get_logger().info('Isaac Navigation Tester initialized')

    def performance_callback(self, msg):
        """Record navigation performance."""
        self.navigation_times.append(msg.data)
        self.get_logger().info(f'Navigation time: {msg.data:.3f} seconds')

    def send_navigation_goals(self):
        """Send a series of navigation goals to test performance."""
        # Define a sequence of waypoints in the warehouse
        waypoints = [
            {'x': -8.0, 'y': 0.0, 'theta': 0.0},  # Waypoint 1
            {'x': 0.0, 'y': 8.0, 'theta': 1.57},  # Waypoint 2
            {'x': 8.0, 'y': -8.0, 'theta': 3.14},  # Waypoint 3
            {'x': 0.0, 'y': 0.0, 'theta': 0.0},   # Center
            {'x': -8.0, 'y': 8.0, 'theta': -1.57} # Back to start
        ]

        self.get_logger().info(f'Starting navigation test with {len(waypoints)} waypoints')

        # Send each goal sequentially
        for i, waypoint in enumerate(waypoints):
            self.get_logger().info(f'Sending goal {i+1}: ({waypoint["x"]}, {waypoint["y"]})')

            # Send the goal
            future = self.send_goal(waypoint['x'], waypoint['y'], waypoint['theta'])

            # Wait for completion before sending next goal
            while not future.done():
                rclpy.spin_once(self, timeout_sec=0.1)

            # Small delay between goals
            time.sleep(1.0)

        # Print performance summary
        if self.navigation_times:
            avg_time = sum(self.navigation_times) / len(self.navigation_times)
            min_time = min(self.navigation_times)
            max_time = max(self.navigation_times)

            self.get_logger().info(f'Navigation Performance Summary:')
            self.get_logger().info(f'  Average time: {avg_time:.3f} seconds')
            self.get_logger().info(f'  Min time: {min_time:.3f} seconds')
            self.get_logger().info(f'  Max time: {max_time:.3f} seconds')
            self.get_logger().info(f'  Total goals: {len(self.navigation_times)}')

    def send_goal(self, x, y, theta):
        """Send a single navigation goal."""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        # Convert theta to quaternion
        import math
        from tf_transformations import quaternion_from_euler
        quat = quaternion_from_euler(0, 0, theta)
        goal_msg.pose.pose.orientation.x = quat[0]
        goal_msg.pose.pose.orientation.y = quat[1]
        goal_msg.pose.pose.orientation.z = quat[2]
        goal_msg.pose.pose.orientation.w = quat[3]

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=None
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)
        return self._send_goal_future

    def goal_response_callback(self, future):
        """Handle goal response."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Handle navigation result."""
        result = future.result().result
        self.get_logger().info(f'Navigation result: {result}')


def main(args=None):
    """Main function to run the navigation test."""
    rclpy.init(args=args)

    tester = IsaacNavigationTester()

    # Allow some time for setup
    time.sleep(2.0)

    # Run the navigation test
    tester.send_navigation_goals()

    # Shutdown
    tester.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Step 10: Update Package Configuration

Update the package.xml file:

```bash
nano ~/isaac_navigation_ws/src/isaac_navigation_lab/package.xml
```

Add the following content:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>isaac_navigation_lab</name>
  <version>0.0.0</version>
  <description>Isaac Navigation Lab Package</description>
  <maintainer email="your.email@example.com">Your Name</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_python</buildtool_depend>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>visualization_msgs</depend>
  <depend>cv_bridge</depend>
  <depend>nav2_msgs</depend>
  <depend>tf_transformations</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

Update the setup.py file:

```bash
nano ~/isaac_navigation_ws/src/isaac_navigation_lab/setup.py
```

Add the following content:

```python
from setuptools import setup
from glob import glob
import os

package_name = 'isaac_navigation_lab'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('isaac_navigation_lab/config/*')),
        (os.path.join('share', package_name, 'launch'), glob('isaac_navigation_lab/launch/*')),
        (os.path.join('share', package_name, 'worlds'), glob('isaac_navigation_lab/worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Isaac Navigation Lab Package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'isaac_performance_monitor = isaac_navigation_lab.isaac_performance_monitor:main',
            'test_isaac_navigation = isaac_navigation_lab.test_isaac_navigation:main',
        ],
    },
)
```

## Step 11: Build and Test the Isaac Navigation System

Build the package:

```bash
cd ~/isaac_navigation_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select isaac_navigation_lab
```

Source the workspace:

```bash
source install/setup.bash
```

## Step 12: Run Isaac Navigation Test

To run the Isaac navigation test in simulation:

1. Start Gazebo with the warehouse world:
```bash
# Terminal 1
cd ~/isaac_navigation_ws
source install/setup.bash
ros2 launch gazebo_ros gazebo.launch.py world:=$(pwd)/src/isaac_navigation_lab/isaac_navigation_lab/worlds/warehouse_navigation.sdf
```

2. Spawn the robot:
```bash
# Terminal 2
cd ~/isaac_navigation_ws
source install/setup.bash
ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity turtlebot3_waffle -x -8 -y 8 -z 0.01
```

3. Run the navigation stack:
```bash
# Terminal 3
cd ~/isaac_navigation_ws
source install/setup.bash
ros2 launch isaac_navigation_lab isaac_navigation.launch.py
```

4. Run the performance monitor:
```bash
# Terminal 4
cd ~/isaac_navigation_ws
source install/setup.bash
ros2 run isaac_navigation_lab isaac_performance_monitor
```

5. Run the navigation test:
```bash
# Terminal 5
cd ~/isaac_navigation_ws
source install/setup.bash
ros2 run isaac_navigation_lab test_isaac_navigation
```

## Step 13: Compare Isaac vs Standard Navigation

To compare Isaac navigation performance with standard ROS 2 navigation:

1. Run the same test with standard Navigation2 stack
2. Record performance metrics for both systems
3. Compare:
   - Navigation time to reach goals
   - CPU utilization
   - Path optimality
   - Obstacle avoidance effectiveness

## Troubleshooting Tips

1. **Isaac packages not found**: Ensure Isaac ROS packages are properly installed
2. **GPU acceleration not working**: Verify CUDA installation and GPU compatibility
3. **Navigation performance issues**: Check sensor data quality and timing
4. **SLAM instability**: Verify camera calibration and lighting conditions
5. **Performance monitoring**: Ensure all monitoring nodes are running

## Expected Results

When running the Isaac navigation system, you should observe:
- Faster SLAM processing with GPU acceleration
- Improved navigation performance in complex environments
- Better obstacle detection and avoidance
- Higher frame rates for visual processing
- More stable localization in dynamic environments

## Practice Exercises

### Exercise 1: Isaac SLAM Optimization
1. Configure Isaac Visual SLAM with different parameter sets for various environments (outdoor, indoor, low-texture).
2. Compare the mapping quality and processing speed between different configurations.
3. Analyze the impact of GPU acceleration on SLAM performance.
4. Document the optimal parameter sets for different scenarios.

### Exercise 2: Isaac Navigation Performance Analysis
1. Implement both Isaac and standard Navigation2 systems in the same environment.
2. Collect performance metrics for both systems (CPU usage, path optimality, navigation time).
3. Analyze the differences in performance and computational requirements.
4. Create a comparative report highlighting the advantages of Isaac navigation.

### Exercise 3: Dynamic Obstacle Integration
1. Add dynamic obstacles to the warehouse simulation environment.
2. Configure Isaac's dynamic obstacle avoidance system.
3. Test navigation performance with moving obstacles of different speeds.
4. Tune parameters to optimize safety and efficiency trade-offs.

### Exercise 4: Multi-Sensor Fusion with Isaac
1. Integrate multiple sensor types (camera, LiDAR, IMU) with Isaac navigation.
2. Configure sensor fusion parameters for optimal localization accuracy.
3. Test navigation performance with different sensor combinations.
4. Analyze how each sensor contributes to overall navigation robustness.

### Discussion Questions
1. What are the key differences between Isaac's GPU-accelerated navigation and standard CPU-based navigation in terms of performance and capabilities?
2. How does Isaac's Visual SLAM compare to traditional laser-based SLAM systems in various environments?
3. What are the main challenges when deploying Isaac navigation on edge computing platforms like Jetson?
4. How can Isaac's navigation system be adapted for outdoor environments with GPS integration?

### Challenge Exercise
Design and implement a complete Isaac-based navigation system for a warehouse robot:
- Create a detailed warehouse map using Isaac Visual SLAM
- Integrate multiple sensors (stereo camera, LiDAR, IMU) for robust navigation
- Implement dynamic obstacle avoidance for humans and other vehicles
- Set up fleet coordination for multiple robots
- Deploy the system on a Jetson platform and test in a simulated warehouse environment
- Create a comprehensive performance analysis comparing with standard ROS 2 navigation
- Document the system architecture, performance metrics, and optimization strategies

## Key Takeaways

- You've implemented Isaac navigation with GPU acceleration
- You've compared Isaac performance with standard ROS 2 navigation
- You've learned to configure Isaac-specific parameters
- You've tested navigation in a realistic warehouse environment
- You understand the benefits of hardware acceleration for robotics

## References

[Isaac Bibliography](/docs/references/isaac-bibliography.md)