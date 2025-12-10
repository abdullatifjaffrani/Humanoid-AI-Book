---
title: Basic ROS 2 Package Template
sidebar_position: 1
---

# Basic ROS 2 Package Template

## Overview

This template provides a starting point for creating new ROS 2 packages. It includes the basic structure, configuration files, and example nodes that follow ROS 2 best practices.

## Package Structure

```
my_robot_package/
├── CMakeLists.txt              # Build configuration for C++ packages
├── package.xml                 # Package metadata and dependencies
├── setup.py                    # Build configuration for Python packages
├── setup.cfg                   # Installation configuration
├── my_robot_package/           # Python package directory
│   ├── __init__.py            # Python package initialization
│   ├── my_robot_node.py       # Example Python node
│   └── my_robot_component.py  # Example component
├── src/                       # C++ source files
│   └── my_robot_node.cpp      # Example C++ node
├── include/my_robot_package/  # C++ header files
│   └── my_robot_node.hpp      # Example C++ header
├── launch/                    # Launch files
│   └── my_robot_launch.py     # Example launch file
├── config/                    # Configuration files
│   └── my_robot_params.yaml   # Example parameter file
├── worlds/                    # Simulation world files
│   └── my_robot_world.sdf     # Example world file
├── meshes/                    # 3D model files
│   └── components/            # Robot component meshes
├── urdf/                      # Robot description files
│   └── my_robot.urdf.xacro    # Example robot description
└── test/                      # Test files
    ├── test_my_robot.py       # Example Python test
    └── test_my_robot.cpp      # Example C++ test
```

## Template Files

### package.xml

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_package</name>
  <version>0.0.0</version>
  <description>My Robot Package Description</description>
  <maintainer email="maintainer@todo.todo">Maintainer Name</maintainer>
  <license>TODO License Declaration</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>ament_python</buildtool_depend>

  <depend>rclpy</depend>
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_robot_package)

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

# Include directories
include_directories(include)

# Example C++ executable
add_executable(my_robot_node src/my_robot_node.cpp)
ament_target_dependencies(my_robot_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
)

# Install targets
install(TARGETS
  my_robot_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install launch files
install(DIRECTORY
  launch
  config
  worlds
  urdf
  meshes
  DESTINATION share/${PROJECT_NAME}/
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

### setup.py

```python
from setuptools import setup
from glob import glob
import os

package_name = 'my_robot_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/**/*', recursive=True)),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TODO',
    maintainer_email='TODO',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_robot_node = my_robot_package.my_robot_node:main',
        ],
    },
)
```

### Example Python Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class MyRobotNode(Node):
    def __init__(self):
        super().__init__('my_robot_node')

        # Create publishers
        self.publisher_ = self.create_publisher(String, 'robot_status', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Create subscribers
        self.subscription = self.create_subscription(
            String,
            'robot_commands',
            self.command_callback,
            10
        )

        # Create timer
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Robot state
        self.i = 0

        self.get_logger().info('My Robot Node initialized')

    def command_callback(self, msg):
        self.get_logger().info(f'Received command: {msg.data}')
        # Process the command here

    def timer_callback(self):
        msg = String()
        msg.data = f'Robot status: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    my_robot_node = MyRobotNode()

    try:
        rclpy.spin(my_robot_node)
    except KeyboardInterrupt:
        pass
    finally:
        my_robot_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Example Launch File

```python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # Nodes
    my_robot_node = Node(
        package='my_robot_package',
        executable='my_robot_node',
        name='my_robot_node',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # Create launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_use_sim_time)

    # Add nodes
    ld.add_action(my_robot_node)

    return ld
```

## Usage Instructions

1. **Copy the template**: Copy this entire structure to your new package directory
2. **Replace placeholders**: Replace `my_robot_package` with your actual package name
3. **Update metadata**: Modify package.xml with your specific information
4. **Customize code**: Modify the example nodes to implement your functionality
5. **Add dependencies**: Update package.xml and CMakeLists.txt with your dependencies
6. **Build and test**: Use `colcon build` to build your package

## Best Practices

- Follow ROS 2 naming conventions (lowercase with underscores)
- Use descriptive names for nodes, topics, and parameters
- Include proper error handling and logging
- Add unit tests for your components
- Document your code and APIs
- Use parameters for configurable values
- Follow the single responsibility principle for nodes

## Common Customizations

### For Different Robot Types

Modify the template based on your robot type:
- **Mobile robots**: Add navigation and mapping components
- **Manipulators**: Add arm control and kinematics
- **Sensors**: Add sensor processing pipelines
- **Simulation**: Add Gazebo integration

### For Different Applications

Adapt the template for specific use cases:
- **SLAM**: Add mapping and localization nodes
- **Perception**: Add computer vision processing
- **Control**: Add motion control algorithms
- **Planning**: Add path planning components