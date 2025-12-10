---
title: Week 5 - Gazebo Environments
sidebar_position: 2
week: 5
module: module-2-gazebo-unity
learningObjectives:
  - Design and create custom simulation environments
  - Configure lighting, textures, and materials in Gazebo
  - Import and use existing models and environments
  - Understand coordinate frames and transformations
  - Implement environment-specific physics properties
prerequisites:
  - Week 1-4 content: ROS 2 Foundations and Simulation Basics
  - Understanding of 3D coordinate systems
  - Basic knowledge of XML/SDF format
description: Advanced topics in creating and configuring Gazebo simulation environments
---

# Week 5: Gazebo Environments

## Learning Objectives

- Design and create custom simulation environments
- Configure lighting, textures, and materials in Gazebo
- Import and use existing models and environments
- Understand coordinate frames and transformations
- Implement environment-specific physics properties

## Overview

Creating realistic and useful simulation environments is crucial for effective robotics development. This week focuses on advanced techniques for designing Gazebo environments that accurately represent real-world scenarios. We'll cover everything from basic scene setup to complex multi-robot environments with custom physics properties.

## Environment Design Principles

### Realism vs Performance

When designing simulation environments, consider the trade-off between:
- **Visual realism**: High-quality textures, lighting, and models
- **Physics accuracy**: Realistic physics properties and constraints
- **Performance**: Simulation speed and stability
- **Computational requirements**: Hardware demands for running simulations

### Environmental Components

A complete Gazebo environment typically includes:

1. **Terrain**: Ground surfaces, elevation changes, outdoor environments
2. **Static objects**: Buildings, furniture, obstacles, fixtures
3. **Dynamic objects**: Movable objects, props, interactive elements
4. **Lighting**: Sun, artificial lights, shadows, atmospheric effects
5. **Physics properties**: Gravity, friction, restitution, damping

## Creating Custom Environments

### World File Structure

A Gazebo world file (SDF format) contains:

```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <world name="custom_world">
    <!-- Physics engine configuration -->
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

    <!-- Lighting -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Custom models and objects -->
    <model name="my_robot">
      <!-- Model definition -->
    </model>

    <model name="obstacle_1">
      <!-- Obstacle definition -->
    </model>
  </world>
</sdf>
```

### Terrain Creation

#### Flat Surfaces

For simple flat surfaces:

```xml
<model name="flat_surface">
  <pose>0 0 0 0 0 0</pose>
  <link name="surface_link">
    <collision name="collision">
      <geometry>
        <box>
          <size>10 10 0.1</size>
        </box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box>
          <size>10 10 0.1</size>
        </box>
      </geometry>
      <material>
        <ambient>0.7 0.7 0.7 1</ambient>
        <diffuse>0.7 0.7 0.7 1</diffuse>
        <specular>0.1 0.1 0.1 1</specular>
      </material>
    </visual>
  </link>
</model>
```

#### Complex Terrains

For complex terrains, use heightmap models:

```xml
<model name="heightmap_terrain">
  <static>true</static>
  <link name="heightmap_link">
    <collision name="collision">
      <geometry>
        <heightmap>
          <uri>file://path/to/heightmap.png</uri>
          <size>100 100 20</size>
          <sampling>2</sampling>
        </heightmap>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <heightmap>
          <uri>file://path/to/heightmap.png</uri>
          <size>100 100 20</size>
          <sampling>2</sampling>
        </heightmap>
      </geometry>
    </visual>
  </link>
</model>
```

## Lighting and Atmospheric Effects

### Sun Configuration

Configure the sun for realistic outdoor lighting:

```xml
<light name="sun" type="directional">
  <cast_shadows>true</cast_shadows>
  <pose>0 0 10 0 0 0</pose>
  <diffuse>0.8 0.8 0.8 1</diffuse>
  <specular>0.2 0.2 0.2 1</specular>
  <attenuation>
    <range>1000</range>
    <constant>0.9</constant>
    <linear>0.01</linear>
    <quadratic>0.001</quadratic>
  </attenuation>
  <direction>-0.707 -0.707 -0.707</direction>
</light>
```

### Artificial Lighting

Add artificial lights for indoor environments:

```xml
<light name="room_light" type="point">
  <pose>0 0 3 0 0 0</pose>
  <diffuse>1 1 1 1</diffuse>
  <specular>0.5 0.5 0.5 1</specular>
  <attenuation>
    <range>10</range>
    <constant>0.5</constant>
    <linear>0.1</linear>
    <quadratic>0.01</quadratic>
  </attenuation>
</light>
```

## Physics Configuration

### Environment-Specific Physics

Different environments may require different physics properties:

```xml
<world name="underwater_world">
  <!-- Underwater physics -->
  <physics name="underwater" type="ode">
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1</real_time_factor>
    <real_time_update_rate>1000</real_time_update_rate>
    <gravity>0 0 -9.8</gravity>
    <!-- Buoyancy and drag forces would be implemented via plugins -->
  </physics>

  <!-- Underwater environment properties -->
  <scene>
    <ambient>0.1 0.1 0.2 1</ambient>
    <background>0.1 0.1 0.2 1</background>
    <fog>
      <type>linear</type>
      <color>0.2 0.2 0.4 1</color>
      <start>5</start>
      <end>20</end>
    </fog>
  </scene>
</world>
```

### Surface Properties

Configure surface properties for realistic interactions:

```xml
<model name="friction_surface">
  <link name="surface">
    <collision name="collision">
      <surface>
        <friction>
          <ode>
            <mu>0.8</mu>
            <mu2>0.8</mu2>
            <fdir1>0 0 1</fdir1>
          </ode>
        </friction>
        <bounce>
          <restitution_coefficient>0.1</restitution_coefficient>
          <threshold>100000</threshold>
        </bounce>
        <contact>
          <ode>
            <soft_cfm>0</soft_cfm>
            <soft_erp>0.2</soft_erp>
            <kp>1e+13</kp>
            <kd>1</kd>
            <max_vel>0.01</max_vel>
            <min_depth>0</min_depth>
          </ode>
        </contact>
      </surface>
    </collision>
  </link>
</model>
```

## Model Integration

### Using Gazebo Model Database

Include models from the Gazebo model database:

```xml
<include>
  <name>simple_arm</name>
  <uri>model://simple_arm</uri>
  <pose>1 1 0 0 0 0</pose>
</include>
```

### Custom Model Placement

Place custom models with precise positioning:

```xml
<model name="my_custom_robot">
  <include>
    <uri>model://my_robot_description</uri>
  </include>
  <pose>0 0 0.5 0 0 0</pose> <!-- Position 0.5m above ground -->
  <plugin name="ros_control" filename="libgazebo_ros_control.so">
    <robotNamespace>/my_robot</robotNamespace>
  </plugin>
</model>
```

## Multi-Robot Environments

### Coordinating Multiple Robots

For multi-robot simulations, ensure proper namespaces and coordination:

```xml
<!-- Robot 1 -->
<model name="robot1">
  <pose>-2 0 0.1 0 0 0</pose>
  <!-- Robot definition -->
  <plugin name="ros_control_1" filename="libgazebo_ros_control.so">
    <robotNamespace>/robot1</robotNamespace>
  </plugin>
</model>

<!-- Robot 2 -->
<model name="robot2">
  <pose>2 0 0.1 0 0 0</pose>
  <!-- Robot definition -->
  <plugin name="ros_control_2" filename="libgazebo_ros_control.so">
    <robotNamespace>/robot2</robotNamespace>
  </plugin>
</model>
```

### Communication Between Robots

Set up communication channels for multi-robot coordination:

```xml
<!-- World plugin for inter-robot communication -->
<plugin name="multi_robot_coordinator" filename="libmulti_robot_coordinator.so">
  <update_rate>10</update_rate>
  <communication_range>10.0</communication_range>
  <topic_name>/multi_robot/coordination</topic_name>
</plugin>
```

## Environment-Specific Scenarios

### Indoor Environments

For indoor scenarios, focus on:

- **Architecture**: Rooms, corridors, doors, furniture
- **Lighting**: Artificial lights, shadows, reflections
- **Materials**: Surfaces with appropriate friction and appearance
- **Acoustics**: Sound propagation (for audio sensors)

### Outdoor Environments

For outdoor scenarios, consider:

- **Terrain**: Ground types, elevation, vegetation
- **Weather**: Sun position, atmospheric effects
- **Obstacles**: Natural and artificial obstacles
- **Scale**: Larger environments, horizon effects

### Specialized Environments

#### Warehouse Environment

```xml
<world name="warehouse">
  <physics type="ode">
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1</real_time_factor>
  </physics>

  <include>
    <uri>model://warehouse</uri>
  </include>

  <model name="pallet_truck">
    <pose>5 0 0.1 0 0 0</pose>
  </model>

  <model name="cargo_box_1">
    <pose>8 2 0.5 0 0 0</pose>
  </model>
</world>
```

#### Urban Environment

```xml
<world name="urban_scenario">
  <scene>
    <shadows>true</shadows>
    <grid>true</grid>
    <origin_visual>true</origin_visual>
  </scene>

  <include>
    <uri>model://city_block</uri>
  </include>

  <model name="delivery_robot">
    <pose>0 0 0.2 0 0 0</pose>
  </model>
</world>
```

## Performance Optimization

### Level of Detail (LOD)

Use different levels of detail based on distance:

```xml
<model name="detailed_model">
  <link name="main_link">
    <visual name="high_detail">
      <geometry>
        <mesh>
          <uri>model://detailed_mesh.dae</uri>
        </mesh>
      </geometry>
      <transparency>0.5</transparency>
    </visual>
    <visual name="low_detail">
      <geometry>
        <box>
          <size>1 1 1</size>
        </box>
      </geometry>
    </visual>
  </link>
</model>
```

### Simulation Parameters

Tune simulation parameters for optimal performance:

```xml
<physics name="optimized_physics" type="ode">
  <max_step_size>0.01</max_step_size>  <!-- Larger step for performance -->
  <real_time_factor>2</real_time_factor>  <!-- Faster than real-time -->
  <real_time_update_rate>100</real_time_update_rate>
  <gravity>0 0 -9.8</gravity>

  <!-- ODE-specific optimizations -->
  <ode>
    <solver>
      <type>quick</type>
      <iters>10</iters>
      <sor>1.3</sor>
    </solver>
    <constraints>
      <cfm>0.000001</cfm>
      <erp>0.2</erp>
      <contact_max_correcting_vel>100</contact_max_correcting_vel>
      <contact_surface_layer>0.001</contact_surface_layer>
    </constraints>
  </ode>
</physics>
```

## Best Practices

### Environment Design

1. **Start Simple**: Begin with basic environments and add complexity gradually
2. **Modular Design**: Create reusable components for different scenarios
3. **Documentation**: Comment world files and maintain design documentation
4. **Testing**: Validate environments with simple test robots before complex scenarios
5. **Version Control**: Keep environment files under version control

### Performance Considerations

1. **Mesh Complexity**: Simplify meshes for collision detection
2. **Update Rates**: Balance sensor update rates with performance
3. **Physics Parameters**: Tune for stability and performance
4. **Resource Management**: Monitor CPU and GPU usage during simulation

## Key Takeaways

- Environment design requires balancing realism, performance, and functionality
- Proper physics configuration is essential for realistic simulation
- Multi-robot environments need careful coordination and namespace management
- Performance optimization is crucial for complex scenarios
- Modular design enables reusability across different applications

## Practice Exercises

### Exercise 1: Create a Custom Indoor Environment
1. Design a simple indoor environment with walls, furniture, and a floor.
2. Include proper lighting with artificial light sources.
3. Add surface properties with different friction coefficients for various floor materials.
4. Test the environment with a simple robot model to ensure physics behave correctly.

### Exercise 2: Multi-Robot Warehouse Scenario
1. Create a warehouse environment with shelves, aisles, and loading docks.
2. Place multiple robot models (at least 2-3) in the environment with different namespaces.
3. Configure each robot with appropriate sensors (camera, LIDAR).
4. Test communication between robots by having them avoid each other.

### Exercise 3: Outdoor Terrain with Heightmap
1. Create a heightmap image (or use a simple one) to define a terrain with hills and valleys.
2. Import the heightmap into Gazebo as a terrain model.
3. Add realistic outdoor lighting and atmospheric effects (fog).
4. Test a robot's navigation capabilities across the varied terrain.

### Exercise 4: Physics Property Experimentation
1. Create multiple identical objects with different surface properties (friction, restitution).
2. Test how these objects behave when dropped or pushed.
3. Document the differences in behavior and explain the physics principles involved.
4. Optimize the physics parameters for stable simulation while maintaining realistic behavior.

### Discussion Questions
1. How do different physics engines (ODE, Bullet, Simbody) affect the simulation of complex environments?
2. What are the key considerations when designing environments for multi-robot systems?
3. How can you balance visual realism with simulation performance in large environments?
4. What strategies would you use to validate that your simulation environment accurately represents the real world?

### Challenge Exercise
Design and implement a complete urban delivery robot scenario:
- Create a city block environment with roads, sidewalks, buildings, and traffic signs
- Include dynamic elements like moving obstacles (simulated pedestrians)
- Add realistic lighting that changes throughout the day
- Implement a delivery robot with appropriate sensors and navigation capabilities
- Create a task where the robot must navigate through the environment to deliver packages
- Document the environmental parameters that affect robot performance and suggest optimizations

## References

[Simulation Bibliography](/docs/references/simulation-bibliography.md)