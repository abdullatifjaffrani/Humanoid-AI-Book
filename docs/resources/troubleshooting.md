---
title: Troubleshooting Guide
sidebar_position: 2
---

# Troubleshooting Guide

This guide addresses common issues and solutions for the Physical AI & Humanoid Robotics Textbook curriculum.

## ROS 2 Common Issues

### Installation Problems
**Issue**: Cannot install ROS 2 packages
**Solution**:
1. Verify Ubuntu version is supported (22.04 recommended)
2. Check internet connection and firewall settings
3. Verify ROS 2 repository keys are correctly added
4. Try manual installation following official ROS 2 documentation

### Node Communication Issues
**Issue**: Nodes cannot communicate with each other
**Solution**:
1. Check that nodes are on the same ROS domain ID
2. Verify network configuration for multi-machine setups
3. Confirm that firewalls allow DDS communication
4. Use `ros2 doctor` to diagnose communication issues

### Package Building Issues
**Issue**: `colcon build` fails with compilation errors
**Solution**:
1. Verify all dependencies are installed with `rosdep install`
2. Check for missing package.xml dependencies
3. Verify correct CMakeLists.txt configuration
4. Clear build directory and rebuild: `rm -rf build/ install/ log/ && colcon build`

## Simulation Environment Issues

### Gazebo Problems
**Issue**: Gazebo crashes or doesn't start
**Solution**:
1. Check graphics drivers are properly installed
2. Verify GPU compatibility with rendering system
3. Try running with software rendering: `export LIBGL_ALWAYS_SOFTWARE=1`
4. Check available system memory and swap space

### Model Loading Issues
**Issue**: Robot models don't load properly in simulation
**Solution**:
1. Verify URDF/SDF files are properly formatted
2. Check that all referenced meshes and textures exist
3. Validate joint limits and kinematic chains
4. Test model with simple Gazebo world first

## NVIDIA Isaac Platform Issues

### GPU Acceleration Problems
**Issue**: Isaac ROS packages not using GPU acceleration
**Solution**:
1. Verify NVIDIA GPU is detected: `nvidia-smi`
2. Check CUDA installation: `nvcc --version`
3. Confirm Isaac ROS packages are properly installed
4. Verify GPU compute capability meets requirements (6.0+)

### Isaac Sim Connection Issues
**Issue**: Cannot connect Isaac Sim to ROS 2 workspace
**Solution**:
1. Check Isaac Sim and ROS 2 versions are compatible
2. Verify Omniverse connection settings
3. Confirm Isaac Sim extensions are enabled
4. Check network connectivity between applications

## Vision Processing Issues

### Camera Stream Problems
**Issue**: Camera topics are not publishing or showing corrupted images
**Solution**:
1. Verify camera is properly connected and detected
2. Check camera calibration files are correctly specified
3. Verify camera driver is properly configured
4. Use `rqt_image_view` to verify raw image stream

### Perception Node Failures
**Issue**: Vision processing nodes crash or produce poor results
**Solution**:
1. Check GPU memory availability for deep learning models
2. Verify input image format matches expected format
3. Confirm model files are properly downloaded and accessible
4. Test with simpler perception pipelines first

## Navigation System Issues

### Localization Problems
**Issue**: Robot cannot localize in map
**Solution**:
1. Verify initial pose is set approximately correctly
2. Check sensor data quality and frame transforms
3. Confirm map file is properly loaded and accessible
4. Adjust AMCL parameters for environment characteristics

### Path Planning Failures
**Issue**: Navigation fails to find path or gets stuck
**Solution**:
1. Verify costmap parameters are properly configured
2. Check for proper sensor coverage of environment
3. Confirm inflation radius is appropriate for robot size
4. Test with smaller, simpler environments first

## Common Build and Runtime Errors

### Python Import Errors
**Issue**: Python modules not found when importing
**Solution**:
1. Source ROS 2 setup: `source /opt/ros/humble/setup.bash`
2. Source workspace: `source install/setup.bash`
3. Check Python path and module installation location
4. Verify correct Python version is being used

### Permission Issues
**Issue**: Cannot access hardware devices or write to files
**Solution**:
1. Add user to appropriate groups: `sudo usermod -a -G dialout $USER`
2. Check file permissions on required directories
3. Verify device access permissions (e.g., serial ports, cameras)
4. Restart terminal/shell after group changes

## Performance Issues

### Slow Performance
**Issue**: Applications running slower than expected
**Solution**:
1. Monitor CPU and GPU utilization
2. Check for memory leaks or excessive memory usage
3. Verify real-time kernel settings if required
4. Reduce simulation complexity or update hardware

### High Latency
**Issue**: Delays in robot response or sensor processing
**Solution**:
1. Check network latency for distributed systems
2. Verify processing nodes are not CPU-bound
3. Check QoS settings for appropriate reliability
4. Profile code to identify bottlenecks

## Development Environment Setup

### IDE Configuration
**Issue**: IDE cannot find ROS 2 packages or dependencies
**Solution**:
1. Configure IDE with proper Python interpreter (source ROS 2 workspace)
2. Install ROS 2 extensions for VS Code or other IDEs
3. Verify CMake configuration for C++ packages
4. Check that build environment is properly sourced

### Git Workflow Issues
**Issue**: Git operations conflict with ROS workspace
**Solution**:
1. Keep source code separate from workspace build directories
2. Add build/install/log directories to .gitignore
3. Use appropriate branching strategies for development
4. Verify workspace overlay configuration

## Hardware Integration

### Sensor Connection Issues
**Issue**: Cannot connect to sensors or actuators
**Solution**:
1. Verify physical connections and power supply
2. Check device permissions and access rights
3. Confirm correct baud rates and communication protocols
4. Test hardware independently before integration

### Actuator Control Problems
**Issue**: Motors or servos not responding as expected
**Solution**:
1. Verify safety systems are properly configured
2. Check joint limits and safety controllers
3. Confirm hardware interface configuration
4. Test with simple position/velocity commands first

## Debugging Strategies

### Logging and Diagnostics
1. Use `ros2 topic echo` to inspect message content
2. Use `rqt_graph` to visualize node connections
3. Enable detailed logging for nodes experiencing issues
4. Use `ros2 doctor` for system diagnostics

### Common Debugging Commands
```bash
# Check active nodes
ros2 node list

# Check active topics
ros2 topic list

# Check topic data
ros2 topic echo /topic_name

# Check service availability
ros2 service list

# Check action servers
ros2 action list
```

## Getting Additional Help

### Community Resources
- ROS Answers: https://answers.ros.org/
- Gazebo Answers: https://answers.gazebosim.org/
- NVIDIA Isaac Forum: https://forums.developer.nvidia.com/
- GitHub Issues: Check repository for known issues

### When to Seek Help
- Reproduced issue on clean installation
- Followed all troubleshooting steps
- Issue affects basic functionality
- Need clarification on textbook content

### Providing Information When Seeking Help
1. ROS 2 version and distribution
2. Operating system and hardware specifications
3. Exact error messages
4. Steps to reproduce the issue
5. What you've already tried to resolve the issue

## Preventive Measures

### Best Practices
- Regularly update and backup your development environment
- Test changes incrementally
- Use version control for your own code
- Document configuration changes
- Follow the textbook sequence for proper foundation building

This troubleshooting guide will be updated as new issues are identified. If you encounter an issue not covered here, please report it through the appropriate channels.