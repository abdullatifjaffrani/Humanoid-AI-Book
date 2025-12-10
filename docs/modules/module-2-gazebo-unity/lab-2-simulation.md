---
title: Lab 2 - Simulation Exercise
sidebar_position: 3
week: 5
module: module-2-gazebo-unity
learningObjectives:
  - Create a custom Gazebo world file
  - Design and implement a robot model for simulation
  - Configure sensors and physics properties
  - Integrate the robot with ROS 2 navigation stack
  - Test navigation in simulated environment
prerequisites:
  - Week 1-5 content: ROS 2 Foundations and Simulation Basics
  - Gazebo installed and configured
  - ROS 2 Navigation2 stack installed
  - Basic understanding of URDF/SDF format
description: Hands-on lab to create and test a robot in a custom Gazebo environment
---

# Lab 2: Simulation Exercise

## Learning Objectives

After completing this lab, you will be able to:
- Create a custom Gazebo world file with specific environment features
- Design and implement a robot model suitable for simulation
- Configure sensors and physics properties for realistic simulation
- Integrate the robot with ROS 2 navigation stack
- Test navigation and path planning in simulated environment

## Lab Duration

Estimated time: 120 minutes

## Prerequisites

- ROS 2 Humble Hawksbill installed
- Gazebo Garden installed
- Navigation2 stack installed
- Basic knowledge of Linux command line
- Understanding of URDF/SDF format

## Scenario: Indoor Robot Navigation

You are developing a mobile robot for indoor navigation tasks. In this lab, you'll create a complete simulation environment with a robot model, custom world, and navigation capabilities. The robot should be able to navigate through a simple indoor environment with obstacles.

## Step 1: Create a Workspace for the Simulation Package

First, create a workspace for our simulation package:

```bash
# Create workspace directory if not already done
mkdir -p ~/ros2_simulation_ws/src
cd ~/ros2_simulation_ws

# Source ROS 2 environment
source /opt/ros/humble/setup.bash
```

Now create a package for our simulation:

```bash
cd ~/ros2_simulation_ws/src
ros2 pkg create --build-type ament_cmake simulation_lab --dependencies rclcpp rclpy std_msgs geometry_msgs sensor_msgs nav2_msgs
```

## Step 2: Create Robot URDF Model

Create the robot description directory:

```bash
mkdir -p ~/ros2_simulation_ws/src/simulation_lab/simulation_lab/urdf
```

Create the robot URDF file `turtlebot3_burger_custom.urdf`:

```bash
nano ~/ros2_simulation_ws/src/simulation_lab/simulation_lab/urdf/turtlebot3_burger_custom.urdf
```

Add the following content:

```xml
<?xml version="1.0"?>
<robot name="turtlebot3_burger_custom" xmlns:xacro="http://ros.org/wiki/xacro">
  <!-- Defining the colors used in this robot -->
  <material name="black">
    <color rgba="0.0 0.0 0.0 1.0"/>
  </material>

  <material name="dark">
    <color rgba="0.3 0.3 0.3 1.0"/>
  </material>

  <material name="light_black">
    <color rgba="0.4 0.4 0.4 1.0"/>
  </material>

  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>

  <material name="green">
    <color rgba="0.0 0.8 0.0 1.0"/>
  </material>

  <material name="grey">
    <color rgba="0.5 0.5 0.5 1.0"/>
  </material>

  <material name="orange">
    <color rgba="1.0 0.4235 0.0392 1.0"/>
  </material>

  <material name="brown">
    <color rgba="0.8706 0.8118 0.7647 1.0"/>
  </material>

  <material name="red">
    <color rgba="0.8 0.0 0.0 1.0"/>
  </material>

  <material name="white">
    <color rgba="1.0 1.0 1.0 1.0"/>
  </material>

  <!-- Defining the base of the robot -->
  <link name="base_link">
    <visual>
      <origin xyz="0.0 0.0 0.0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://turtlebot3_description/meshes/bases/burger_base.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="light_black"/>
    </visual>

    <collision>
      <origin xyz="0.0 0.0 0.0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://turtlebot3_description/meshes/bases/burger_base.stl" scale="0.001 0.001 0.001"/>
      </geometry>
    </collision>

    <inertial>
      <origin xyz="0.0 0.0 0.0" rpy="0 0 0"/>
      <mass value="8.2573504e-01"/>
      <inertia ixx="2.2124416e-03" ixy="0.0" ixz="0.0" iyy="2.1148597e-03" iyz="0.0" izz="2.0000000e-03"/>
    </inertial>
  </link>

  <!-- Left wheel -->
  <link name="wheel_left_link">
    <visual>
      <origin xyz="0.0 0.0 0.0" rpy="1.57079632679 0.0 0.0"/>
      <geometry>
        <mesh filename="package://turtlebot3_description/meshes/wheels/left_tire.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="dark"/>
    </visual>

    <collision>
      <origin xyz="0.0 0.0 0.0" rpy="1.57079632679 0.0 0.0"/>
      <geometry>
        <mesh filename="package://turtlebot3_description/meshes/wheels/left_tire.stl" scale="0.001 0.001 0.001"/>
      </geometry>
    </collision>

    <inertial>
      <origin xyz="0.0 0.0 0.0" rpy="0 0 0"/>
      <mass value="2.6818e-02"/>
      <inertia ixx="1.3883e-05" ixy="0.0" ixz="0.0" iyy="1.3883e-05" iyz="0.0" izz="2.1415e-05"/>
    </inertial>
  </link>

  <!-- Right wheel -->
  <link name="wheel_right_link">
    <visual>
      <origin xyz="0.0 0.0 0.0" rpy="1.57079632679 0.0 0.0"/>
      <geometry>
        <mesh filename="package://turtlebot3_description/meshes/wheels/right_tire.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="dark"/>
    </visual>

    <collision>
      <origin xyz="0.0 0.0 0.0" rpy="1.57079632679 0.0 0.0"/>
      <geometry>
        <mesh filename="package://turtlebot3_description/meshes/wheels/right_tire.stl" scale="0.001 0.001 0.001"/>
      </geometry>
    </collision>

    <inertial>
      <origin xyz="0.0 0.0 0.0" rpy="0 0 0"/>
      <mass value="2.6818e-02"/>
      <inertial ixx="1.3883e-05" ixy="0.0" ixz="0.0" iyy="1.3883e-05" iyz="0.0" izz="2.1415e-05"/>
    </inertial>
  </link>

  <!-- caster wheel -->
  <link name="caster_wheel_link">
    <visual>
      <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
      <geometry>
        <mesh filename="package://turtlebot3_description/meshes/wheels/caster_wheel.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="light_black"/>
    </visual>

    <collision>
      <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
      <geometry>
        <mesh filename="package://turtlebot3_description/meshes/wheels/caster_wheel.stl" scale="0.001 0.001 0.001"/>
      </geometry>
    </collision>

    <inertial>
      <origin xyz="0.0 0.0 0.0" rpy="0 0 0"/>
      <mass value="0.005"/>
      <inertia ixx="1.0e-06" ixy="0.0" ixz="0.0" iyy="1.0e-06" iyz="0.0" izz="1.0e-06"/>
    </inertial>
  </link>

  <!-- Camera -->
  <link name="camera_link">
    <visual>
      <origin xyz="0.0 0.0 0.0" rpy="0 0 0"/>
      <geometry>
        <box size="0.01 0.03 0.03"/>
      </geometry>
      <material name="red"/>
    </visual>

    <collision>
      <origin xyz="0.0 0.0 0.0" rpy="0 0 0"/>
      <geometry>
        <box size="0.01 0.03 0.03"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.01"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Joints -->
  <joint name="base_to_wheel_left_joint" type="continuous">
    <origin xyz="0.0 0.08 0.028" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="wheel_left_link"/>
    <axis xyz="0 1 0"/>
  </joint>

  <joint name="base_to_wheel_right_joint" type="continuous">
    <origin xyz="0.0 -0.08 0.028" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="wheel_right_link"/>
    <axis xyz="0 1 0"/>
  </joint>

  <joint name="base_to_caster_wheel_joint" type="fixed">
    <origin xyz="0.046 0.0 -0.022" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="caster_wheel_link"/>
  </joint>

  <joint name="base_to_camera_joint" type="fixed">
    <origin xyz="0.05 0.0 0.07" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="camera_link"/>
  </joint>

  <!-- Gazebo plugins -->
  <gazebo reference="base_link">
    <material>Gazebo/Black</material>
  </gazebo>

  <gazebo reference="wheel_left_link">
    <mu1>100.0</mu1>
    <mu2>100.0</mu2>
    <kp>1000000.0</kp>
    <kd>100.0</kd>
    <material>Gazebo/FlatBlack</material>
  </gazebo>

  <gazebo reference="wheel_right_link">
    <mu1>100.0</mu1>
    <mu2>100.0</mu2>
    <kp>1000000.0</kp>
    <kd>100.0</kd>
    <material>Gazebo/FlatBlack</material>
  </gazebo>

  <gazebo reference="caster_wheel_link">
    <mu1>100.0</mu1>
    <mu2>100.0</mu2>
    <kp>1000000.0</kp>
    <kd>100.0</kd>
    <material>Gazebo/White</material>
  </gazebo>

  <gazebo reference="camera_link">
    <material>Gazebo/Red</material>
  </gazebo>

  <!-- Gazebo ROS Control plugin -->
  <gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
      <robotNamespace>/tb3</robotNamespace>
      <robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType>
    </plugin>
  </gazebo>

  <!-- Camera plugin -->
  <gazebo reference="camera_link">
    <sensor type="camera" name="camera1">
      <update_rate>30.0</update_rate>
      <camera name="head">
        <horizontal_fov>1.3962634</horizontal_fov>
        <image>
          <width>800</width>
          <height>800</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.02</near>
          <far>300</far>
        </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <frame_name>camera_link</frame_name>
        <min_depth>0.1</min_depth>
        <max_depth>100.0</max_depth>
      </plugin>
    </sensor>
  </gazebo>
</robot>
```

## Step 3: Create a Custom World File

Create a directory for world files:

```bash
mkdir -p ~/ros2_simulation_ws/src/simulation_lab/simulation_lab/worlds
```

Create a custom world file `simple_room.world`:

```bash
nano ~/ros2_simulation_ws/src/simulation_lab/simulation_lab/worlds/simple_room.world
```

Add the following content:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="simple_room">
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

    <!-- Room walls -->
    <!-- Wall 1: Front wall -->
    <model name="wall_1">
      <pose>0 -3 1 0 0 0</pose>
      <link name="wall_1_link">
        <collision name="collision">
          <geometry>
            <box>
              <size>6 0.1 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>6 0.1 2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- Wall 2: Back wall -->
    <model name="wall_2">
      <pose>0 3 1 0 0 0</pose>
      <link name="wall_2_link">
        <collision name="collision">
          <geometry>
            <box>
              <size>6 0.1 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>6 0.1 2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- Wall 3: Left wall -->
    <model name="wall_3">
      <pose>-3 0 1 0 0 1.5707</pose>
      <link name="wall_3_link">
        <collision name="collision">
          <geometry>
            <box>
              <size>6 0.1 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>6 0.1 2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- Wall 4: Right wall -->
    <model name="wall_4">
      <pose>3 0 1 0 0 1.5707</pose>
      <link name="wall_4_link">
        <collision name="collision">
          <geometry>
            <box>
              <size>6 0.1 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>6 0.1 2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- Furniture: Table -->
    <model name="table">
      <pose>1 1 0.4 0 0 0</pose>
      <link name="table_top">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 0.8 0.05</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 0.8 0.05</size>
            </box>
          </geometry>
          <material>
            <ambient>0.6 0.4 0.2 1</ambient>
            <diffuse>0.6 0.4 0.2 1</diffuse>
            <specular>0.6 0.4 0.2 1</specular>
          </material>
        </visual>
      </link>
      <link name="leg_1">
        <pose>-0.4 -0.35 0.2 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.05 0.05 0.35</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.05 0.05 0.35</size>
            </box>
          </geometry>
          <material>
            <ambient>0.4 0.2 0.1 1</ambient>
            <diffuse>0.4 0.2 0.1 1</diffuse>
            <specular>0.4 0.2 0.1 1</specular>
          </material>
        </visual>
      </link>
      <link name="leg_2">
        <pose>0.4 -0.35 0.2 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.05 0.05 0.35</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.05 0.05 0.35</size>
            </box>
          </geometry>
          <material>
            <ambient>0.4 0.2 0.1 1</ambient>
            <diffuse>0.4 0.2 0.1 1</diffuse>
            <specular>0.4 0.2 0.1 1</specular>
          </material>
        </visual>
      </link>
      <link name="leg_3">
        <pose>-0.4 0.35 0.2 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.05 0.05 0.35</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.05 0.05 0.35</size>
            </box>
          </geometry>
          <material>
            <ambient>0.4 0.2 0.1 1</ambient>
            <diffuse>0.4 0.2 0.1 1</diffuse>
            <specular>0.4 0.2 0.1 1</specular>
          </material>
        </visual>
      </link>
      <link name="leg_4">
        <pose>0.4 0.35 0.2 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.05 0.05 0.35</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.05 0.05 0.35</size>
            </box>
          </geometry>
          <material>
            <ambient>0.4 0.2 0.1 1</ambient>
            <diffuse>0.4 0.2 0.1 1</diffuse>
            <specular>0.4 0.2 0.1 1</specular>
          </material>
        </visual>
      </link>
      <joint name="top_to_leg_1" type="fixed">
        <parent>table_top</parent>
        <child>leg_1</child>
      </joint>
      <joint name="top_to_leg_2" type="fixed">
        <parent>table_top</parent>
        <child>leg_2</child>
      </joint>
      <joint name="top_to_leg_3" type="fixed">
        <parent>table_top</parent>
        <child>leg_3</child>
      </joint>
      <joint name="top_to_leg_4" type="fixed">
        <parent>table_top</parent>
        <child>leg_4</child>
      </joint>
    </model>

    <!-- Obstacle -->
    <model name="obstacle">
      <pose>-1 -1 0.25 0 0 0</pose>
      <link name="obstacle_link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.5 0.5 0.5</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.5 0.5 0.5</size>
            </box>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
            <specular>1 0 0 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- Robot spawn location -->
    <model name="turtlebot3_burger" static="false">
      <include>
        <uri>model://turtlebot3_burger</uri>
      </include>
      <pose>0 0 0.01 0 0 0</pose>
    </model>
  </world>
</sdf>
```

## Step 4: Create Launch Files

Create a launch directory:

```bash
mkdir -p ~/ros2_simulation_ws/src/simulation_lab/simulation_lab/launch
```

Create a launch file for the simulation `simulation.launch.py`:

```bash
nano ~/ros2_simulation_ws/src/simulation_lab/simulation_lab/launch/simulation.launch.py
```

Add the following content:

```python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default='simple_room.world')

    # Package names
    pkg_gazebo_ros = FindPackageShare('gazebo_ros')
    pkg_simulation_lab = FindPackageShare('simulation_lab')

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='simple_room.world',
        description='Choose one of the world files from `/simulation_lab/worlds`'
    )

    # Include Gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': PathJoinSubstitution([
                FindPackageShare('simulation_lab'),
                'worlds',
                world
            ]),
            'use_sim_time': use_sim_time,
        }.items()
    )

    # Robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': open(
                os.path.join(
                    os.path.dirname(__file__),
                    '..',
                    'urdf',
                    'turtlebot3_burger_custom.urdf'
                )
            ).read()
        }]
    )

    # Spawn entity node to place robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'turtlebot3_burger_custom',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.01'
        ],
        output='screen'
    )

    # Create launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_world_cmd)

    # Add nodes and launch descriptions
    ld.add_action(gazebo)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_entity)

    return ld
```

## Step 5: Create Navigation Configuration

Create a config directory:

```bash
mkdir -p ~/ros2_simulation_ws/src/simulation_lab/simulation_lab/config
```

Create a basic navigation configuration `nav2_params.yaml`:

```bash
nano ~/ros2_simulation_ws/src/simulation_lab/simulation_lab/config/nav2_params.yaml
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

amcl_map_client:
  ros__parameters:
    use_sim_time: True

amcl_rclcpp_node:
  ros__parameters:
    use_sim_time: True

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

bt_navigator_rclcpp_node:
  ros__parameters:
    use_sim_time: True

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

    # Progress checker parameters
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0

    # Goal checker parameters
    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: True

    # Controller parameters
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

controller_server_rclcpp_node:
  ros__parameters:
    use_sim_time: True

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: True
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      robot_radius: 0.1
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
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
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
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: True
      robot_radius: 0.1
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
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
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

map_server:
  ros__parameters:
    use_sim_time: True
    yaml_filename: "map.yaml"

map_saver:
  ros__parameters:
    use_sim_time: True
    save_map_timeout: 5.0
    free_thresh_default: 0.25
    occupied_thresh_default: 0.65

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

planner_server_rclcpp_node:
  ros__parameters:
    use_sim_time: True

smoother_server:
  ros__parameters:
    use_sim_time: True
    smoother_plugins: ["simple_smoother"]
    simple_smoother:
      plugin: "nav2_smoother::SimpleSmoother"
      tolerance: 1.0e-10
      max_its: 1000
      do_refinement: True

behavior_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    behavior_plugins: ["spin", "backup", "drive_on_heading", "assisted_teleop"]
    spin:
      plugin: "nav2_behaviors::Spin"
      spin_dist: 1.57
    backup:
      plugin: "nav2_behaviors::BackUp"
      backup_dist: 0.15
      backup_speed: 0.025
    drive_on_heading:
      plugin: "nav2_behaviors::DriveOnHeading"
      drive_on_heading_distance: 0.5
    assisted_teleop:
      plugin: "nav2_behaviors::AssistedTeleop"
      min_vel_theta: 0.4

velocity_smoother:
  ros__parameters:
    use_sim_time: True
    smoothing_frequency: 20.0
    scale_velocities: False
    velocity_timeout: 1.0
    filter_size: 5
    velocity_scaling: 1.0
    max_velocity: [1.0, 1.0, 1.0]
    min_velocity: [-1.0, -1.0, -1.0]
    max_accel: [2.5, 2.5, 3.2]
    max_decel: [-2.5, -2.5, -3.2]

waypoint_follower:
  ros__parameters:
    use_sim_time: True
    loop_rate: 20
    stop_on_failure: False
    waypoint_task_executor_plugin: "wait_at_waypoint"
    wait_at_waypoint:
      plugin: "nav2_waypoint_follower::WaitAtWaypoint"
      enabled: True
      waypoint_pause_duration: 200
```

## Step 6: Update Package Configuration

Update the package.xml file:

```bash
nano ~/ros2_simulation_ws/src/simulation_lab/package.xml
```

Add the following content:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>simulation_lab</name>
  <version>0.0.0</version>
  <description>Simulation lab package for Gazebo environments</description>
  <maintainer email="your.email@example.com">Your Name</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>nav2_msgs</depend>
  <depend>gazebo_ros_pkgs</depend>
  <depend>robot_state_publisher</depend>
  <depend>xacro</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

Update the CMakeLists.txt file:

```bash
nano ~/ros2_simulation_ws/src/simulation_lab/CMakeLists.txt
```

Add the following content:

```cmake
cmake_minimum_required(VERSION 3.8)
project(simulation_lab)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(robot_state_publisher REQUIRED)

# Install launch files
install(DIRECTORY
  launch
  worlds
  urdf
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_pep257_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

## Step 7: Build and Run the Simulation

Build the package:

```bash
cd ~/ros2_simulation_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select simulation_lab
```

Source the workspace:

```bash
source install/setup.bash
```

Run the simulation:

```bash
ros2 launch simulation_lab simulation.launch.py
```

## Step 8: Test Navigation

In a new terminal, source the workspace and send navigation goals:

```bash
cd ~/ros2_simulation_ws
source install/setup.bash

# Send a simple navigation goal
ros2 run nav2_test nav2_simple_commander --x 2.0 --y 1.0 --theta 0.0
```

Or use RViz2 for visualization and navigation:

```bash
# In another terminal
cd ~/ros2_simulation_ws
source install/setup.bash
ros2 run rviz2 rviz2
```

In RViz2:
1. Set the fixed frame to "map"
2. Add the robot model display
3. Use the "Navigation 2" panel to set goals
4. Visualize the costmaps and path planning

## Step 9: Advanced Exercise - Create a Navigation Node

Create a simple navigation node to programmatically send goals:

```bash
mkdir -p ~/ros2_simulation_ws/src/simulation_lab/simulation_lab/navigation
nano ~/ros2_simulation_ws/src/simulation_lab/simulation_lab/navigation/simple_navigator.py
```

Add the following content:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


class SimpleNavigator(Node):

    def __init__(self):
        super().__init__('simple_navigator')
        self._action_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

    def send_goal(self, x, y, theta):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        # Convert theta (yaw) to quaternion
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
            feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback}')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result}')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    simple_navigator = SimpleNavigator()

    # Send a navigation goal to move to (2.0, 1.0, 0.0)
    simple_navigator.send_goal(2.0, 1.0, 0.0)

    rclpy.spin(simple_navigator)


if __name__ == '__main__':
    main()
```

## Troubleshooting Tips

1. **Robot not spawning**: Check that URDF file is properly formatted and all mesh files exist
2. **Navigation not working**: Verify that all required navigation nodes are running
3. **Performance issues**: Reduce physics update rates or simplify collision geometries
4. **TF issues**: Check that all coordinate frames are properly connected
5. **Sensors not publishing**: Verify that sensor plugins are correctly configured

## Expected Output

When running the simulation, you should see:
- Gazebo window with the custom room environment
- Robot model spawned at the initial position
- RViz2 visualization showing robot pose and sensor data
- Navigation goals being executed successfully

## Practice Exercises

### Exercise 1: Enhance the Robot Model
1. Add a LIDAR sensor to your robot model in the URDF file.
2. Configure the LIDAR plugin with appropriate parameters (range, resolution, update rate).
3. Verify that the LIDAR topic is publishing data in ROS 2.
4. Test obstacle detection by placing objects in the simulation environment.

### Exercise 2: Advanced Navigation Scenario
1. Create a more complex world file with multiple rooms and narrow passages.
2. Add dynamic obstacles (moving objects) to test real-time navigation capabilities.
3. Implement a navigation node that sends a sequence of waypoints to the robot.
4. Test the robot's ability to navigate around dynamic obstacles.

### Exercise 3: Sensor Fusion Integration
1. Add multiple sensor types to your robot (camera, LIDAR, IMU).
2. Create a sensor fusion node that combines data from different sensors.
3. Use the fused sensor data to improve navigation accuracy.
4. Compare navigation performance with and without sensor fusion.

### Exercise 4: Multi-Robot Simulation
1. Modify your world file to include multiple robot models.
2. Set up proper namespaces for each robot to avoid topic conflicts.
3. Implement coordination between robots (e.g., formation control or task sharing).
4. Test collision avoidance between multiple robots in the same environment.

### Discussion Questions
1. What are the main challenges in making simulation results transferable to real-world robotics applications?
2. How do sensor models in simulation differ from real sensors, and how does this affect algorithm development?
3. What strategies can be used to validate that your simulation environment accurately represents the real world?
4. How do computational constraints affect the complexity of simulation environments?

### Challenge Exercise
Design and implement a complete warehouse automation scenario:
- Create a detailed warehouse environment with aisles, shelves, and loading zones
- Implement multiple robots with different capabilities (transport, inspection, etc.)
- Develop a task allocation system that assigns tasks to appropriate robots
- Create a simulation that demonstrates goods transport from one end of the warehouse to another
- Include failure scenarios (robot breakdowns, blocked paths) and recovery mechanisms
- Document the performance metrics and optimization strategies used

## Key Takeaways

- You've created a complete simulation environment with custom world and robot model
- You've integrated the robot with ROS 2 Navigation2 stack
- You've learned to configure sensors and physics properties for realistic simulation
- You've tested navigation capabilities in a controlled environment

## References

[Simulation Bibliography](/docs/references/simulation-bibliography.md)