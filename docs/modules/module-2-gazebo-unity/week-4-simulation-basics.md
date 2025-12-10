---
title: Week 4 - Simulation Basics
sidebar_position: 1
week: 4
module: module-2-gazebo-unity
learningObjectives:
  - Understand the fundamental concepts of robotics simulation
  - Compare different simulation platforms (Gazebo, Unity, Webots)
  - Set up a basic simulation environment
  - Create and configure simple robot models
  - Understand physics engines and their parameters
prerequisites:
  - Week 1-3 content: ROS 2 Foundations
  - Basic understanding of 3D coordinate systems
  - Familiarity with XML/URDF format
description: Introduction to robotics simulation with focus on Gazebo and Unity platforms
---

# Week 4: Simulation Basics

## Learning Objectives

- Understand the fundamental concepts of robotics simulation
- Compare different simulation platforms (Gazebo, Unity, Webots)
- Set up a basic simulation environment
- Create and configure simple robot models
- Understand physics engines and their parameters

## Overview

Robotics simulation is a critical component of modern robotics development, allowing for rapid prototyping, testing, and validation of robotic systems before deploying to real hardware. This week introduces the fundamental concepts of robotics simulation, focusing primarily on Gazebo as the primary simulation environment with references to Unity for advanced applications.

Simulation provides several advantages:
- **Safety**: Test dangerous scenarios without risk to hardware or humans
- **Cost-effectiveness**: No need for physical hardware during initial development
- **Repeatability**: Consistent conditions for testing and debugging
- **Speed**: Faster than real-time testing and iteration
- **Flexibility**: Easy to modify environments and parameters

## Simulation Platforms Overview

### Gazebo (Primary Focus)

Gazebo is a 3D simulation environment that provides:
- High-fidelity physics simulation using ODE, Bullet, or Simbody
- High-quality graphics rendering
- Sensor simulation (cameras, LIDAR, IMU, etc.)
- Plugin system for custom functionality
- Integration with ROS/ROS 2

### Unity Robotics Simulation

Unity provides:
- High-fidelity visual rendering
- Game engine physics
- VR/AR support
- Cross-platform deployment
- Advanced visualization capabilities

### Other Platforms

- **Webots**: All-in-one simulation platform with built-in IDE
- **Mujoco**: Physics engine focused on research applications
- **PyBullet**: Python interface to Bullet physics engine
- **AirSim**: Microsoft's simulation platform for autonomous vehicles

## Gazebo Architecture

### Core Components

Gazebo consists of several key components:

1. **gzserver**: The physics simulation server
2. **gzclient**: The graphical user interface client
3. **Libraries**: C++ libraries for physics, rendering, and sensors
4. **Plugins**: Dynamic extensions for custom functionality

### Communication Architecture

Gazebo uses a client-server model with message passing:
- **Transport layer**: Custom transport protocol based on ZeroMQ
- **Message types**: Protobuf-based messages for simulation data
- **Services**: RPC-style communication for control operations

![Gazebo Simulation Environment](/img/gazebo_simulation.png)

*Figure 2: Gazebo simulation environment showing a robot model in a 3D world with physics simulation.*

## Setting Up Gazebo

### Installation

For Ubuntu with ROS 2 Humble:

```bash
# Install Gazebo Garden (recommended for ROS 2 Humble)
sudo apt update
sudo apt install ros-humble-gazebo-*
sudo apt install gz-garden
```

### Basic Launch

```bash
# Launch Gazebo server
gz sim

# Launch with GUI
gz sim -g
```

## Robot Model Definition (URDF/SDF)

### URDF vs SDF

**URDF (Unified Robot Description Format)**:
- XML-based format
- Primarily used with ROS
- Good for kinematic descriptions
- Limited physics properties

**SDF (Simulation Description Format)**:
- XML-based format
- Native to Gazebo
- Comprehensive physics and simulation properties
- More flexible than URDF

### Basic URDF Example

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Simple wheel -->
  <link name="wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.002"/>
    </inertial>
  </link>

  <!-- Joint connecting base and wheel -->
  <joint name="wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel"/>
    <origin xyz="0.2 0 -0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>
</robot>
```

## Physics Concepts in Simulation

### Physics Engines

Gazebo supports multiple physics engines:

1. **ODE (Open Dynamics Engine)**: Default for older versions
2. **Bullet**: Good balance of performance and features
3. **Simbody**: High-accuracy multi-body dynamics
4. **DART**: Dynamic Animation and Robotics Toolkit

### Key Physics Parameters

- **Gravity**: Usually set to Earth's gravity (9.81 m/s²)
- **Time Step**: Simulation time increment (affects stability)
- **Iterations**: Number of iterations for constraint solving
- **CFM (Constraint Force Mixing)**: Softness of constraints
- **ERP (Error Reduction Parameter)**: Error correction rate

## Basic Simulation Control

### Launching with World Files

World files define the simulation environment:

```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <world name="simple_world">
    <!-- Include a model -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Include another model -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Define a custom model -->
    <model name="simple_box">
      <pose>0 0 0.5 0 0 0</pose>
      <link name="box_link">
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>
  </world>
</sdf>
```

### Running a Simulation

```bash
# Launch with a specific world
gz sim -r simple_world.sdf

# Or using ROS 2 launch
ros2 launch gazebo_ros gazebo.launch.py world:=$(pwd)/simple_world.sdf
```

## Sensor Simulation

### Common Sensors

Gazebo can simulate various sensors:

- **Camera**: RGB, depth, and stereo cameras
- **LIDAR**: 2D and 3D laser scanners
- **IMU**: Inertial measurement units
- **GPS**: Global positioning system
- **Force/Torque**: Joint force and torque sensors

### Camera Sensor Example

```xml
<sensor name="camera" type="camera">
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>100</far>
    </clip>
  </camera>
  <always_on>1</always_on>
  <update_rate>30</update_rate>
  <visualize>true</visualize>
</sensor>
```

## Integration with ROS 2

### ROS 2 Gazebo Packages

Key packages for ROS 2 integration:
- `gazebo_ros_pkgs`: Core ROS 2 integration
- `gazebo_plugins`: Gazebo plugins for ROS 2
- `gazebo_dev`: Development headers and libraries

### Launching with ROS 2

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('gazebo_ros'),
                    'launch',
                    'gazebo.launch.py'
                ])
            ]),
            launch_arguments={
                'world': PathJoinSubstitution([
                    FindPackageShare('my_robot_description'),
                    'worlds',
                    'my_world.sdf'
                ])
            }.items()
        )
    ])
```

## Best Practices

### Model Design

1. **Collision vs Visual**: Separate collision and visual geometry appropriately
2. **Inertial Properties**: Accurate inertial properties for realistic physics
3. **Simplification**: Simplify collision geometry for performance
4. **Units**: Consistent units (SI units recommended)

### Simulation Tuning

1. **Time Step**: Balance accuracy and performance
2. **Real-time Factor**: Monitor real-time performance
3. **Physics Parameters**: Tune for stability and accuracy
4. **Sensors**: Configure appropriate update rates and noise models

## Key Takeaways

- Simulation is essential for safe and cost-effective robotics development
- Gazebo provides comprehensive simulation capabilities with ROS 2 integration
- Proper model definition is crucial for realistic simulation
- Physics parameters significantly affect simulation quality
- Sensor simulation enables perception algorithm testing

## Cross-References

This simulation foundation connects with:
- [Week 1-3: ROS 2 Foundations](/docs/modules/module-1-ros-foundations/) - where you learned the communication framework used with Gazebo
- [Week 5: Gazebo Environments](/docs/modules/module-2-gazebo-unity/week-5-gazebo-environments.md) - for advanced simulation environments
- [Week 6: Isaac Platform Overview](/docs/modules/module-3-nvidia-isaac/week-6-isaac-platform.md) - where simulation integrates with NVIDIA's platform
- [Week 8: Vision Processing](/docs/modules/module-4-vla-systems/week-8-vision-processing.md) - where simulated cameras provide data for computer vision
- [Week 10: Humanoid Robot Control](/docs/modules/module-4-vla-systems/week-10-humanoid-control.md) - where simulation is used for humanoid robot development

## Practice Exercises

### Exercise 1: Create a Simple Robot Model
1. Create a URDF file for a simple differential drive robot with two wheels and a caster.
2. Include proper visual, collision, and inertial properties for each link.
3. Load your robot model into Gazebo and verify it appears correctly.
4. Test the physics by applying forces to the robot and observing its behavior.

### Exercise 2: Physics Parameter Tuning
1. Create a simple simulation with a falling box.
2. Experiment with different physics parameters (time step, iterations, ERP, CFM).
3. Document how each parameter affects the simulation stability and accuracy.
4. Find the optimal settings for your specific simulation scenario.

### Exercise 3: Sensor Integration
1. Add a camera sensor to your robot model from Exercise 1.
2. Configure the camera with appropriate parameters (resolution, FOV, update rate).
3. Launch the simulation and verify the camera topic is available in ROS 2.
4. Use `ros2 topic echo` to view the camera data being published.

### Exercise 4: World Environment Creation
1. Create a custom world file with multiple objects (boxes, cylinders, etc.).
2. Include different materials and textures to make the environment visually interesting.
3. Add lighting conditions that vary throughout the day.
4. Test your robot's navigation in this custom environment.

### Discussion Questions
1. What are the key differences between URDF and SDF, and when would you choose one over the other?
2. How do physics engine choices (ODE, Bullet, Simbody) affect simulation performance and accuracy?
3. What are the trade-offs between visual fidelity and simulation performance in robotics simulation?
4. How can simulation results be validated against real-world robot behavior?

### Challenge Exercise
Create a complete simulation environment for a mobile robot:
- Design a robot model with multiple sensors (camera, LIDAR, IMU)
- Create a complex world with obstacles, ramps, and different surface materials
- Implement a ROS 2 node that controls the robot to navigate through the environment
- Add dynamic elements (moving objects) to test real-time navigation capabilities
- Document the differences in robot behavior between simulation and theoretical models

## References

[Simulation Bibliography](/docs/references/simulation-bibliography.md)