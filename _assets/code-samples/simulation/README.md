# Simulation Code Samples

This directory contains code samples demonstrating various robotics simulation concepts and patterns, particularly for Gazebo and Unity integration with ROS 2.

## Code Samples Included

### basic_gazebo_model.py
- A minimal script to spawn a model in Gazebo
- Demonstrates the spawn_entity service call
- Shows proper model positioning and naming

### gazebo_subscriber.py
- Subscriber that receives data from Gazebo sensors
- Demonstrates processing of laser scan, IMU, or camera data
- Shows integration with ROS 2 message types

### robot_control_gazebo.py
- Publisher for controlling robot in simulation
- Demonstrates sending velocity commands to simulated robot
- Shows proper frame handling and coordinate transformations

### custom_world_generator.py
- Script to programmatically generate SDF world files
- Demonstrates creating environments with Python
- Shows how to add objects, lighting, and physics properties

### sensor_plugin_example.cpp
- C++ example of a custom Gazebo sensor plugin
- Demonstrates integration with ROS 2 topics
- Shows proper plugin architecture and lifecycle management

### physics_configurator.py
- Python script to configure physics properties in simulation
- Demonstrates adjusting friction, restitution, and other parameters
- Shows runtime physics modification capabilities

### multi_robot_coordinator.py
- Example of coordinating multiple robots in simulation
- Demonstrates namespace handling and inter-robot communication
- Shows proper resource management in multi-robot scenarios

### simulation_performance_monitor.py
- Tool for monitoring simulation performance
- Tracks real-time factor, update rates, and resource usage
- Shows optimization techniques for simulation environments

## Code Standards

1. **Documentation**: All samples include comprehensive comments
2. **Error Handling**: Proper error handling and recovery patterns
3. **Best Practices**: Follows ROS 2 and Gazebo coding conventions
4. **Clarity**: Code is written for educational purposes
5. **Completeness**: Each sample is a complete, runnable example
6. **Consistency**: Consistent naming and structure across samples

## Usage Instructions

To use these code samples:

1. Ensure ROS 2 and Gazebo are properly installed and sourced
2. Create a workspace and package if needed
3. Copy the sample code to your package
4. Update package.xml and setup.py/CMakeLists.txt as needed
5. Build the package with `colcon build`
6. Run the node with `ros2 run package_name node_name`

## Language Support

Code samples are provided in:
- Python (recommended for beginners and rapid prototyping)
- C++ (for performance-critical applications and plugins)

## Learning Path

Start with basic Gazebo interaction examples, then progress to:
1. Sensor integration
2. Robot control
3. Custom environments
4. Multi-robot systems
5. Performance optimization
6. Advanced plugin development