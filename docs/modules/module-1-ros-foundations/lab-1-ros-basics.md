---
title: Lab 1 - ROS Basics Exercise
sidebar_position: 4
week: 3
module: module-1-ros-foundations
learningObjectives:
  - Set up a ROS 2 workspace and create custom packages
  - Implement a publisher and subscriber node
  - Create and use custom message types
  - Configure Quality of Service settings
  - Use ROS 2 command-line tools for debugging
prerequisites:
  - Week 1-3 content: ROS 2 Foundations
  - ROS 2 installed and configured
  - Basic Python or C++ programming knowledge
description: Hands-on lab to practice ROS 2 basics including nodes, topics, and custom messages
---

# Lab 1: ROS Basics Exercise

## Learning Objectives

After completing this lab, you will be able to:
- Set up a ROS 2 workspace and create custom packages
- Implement a publisher and subscriber node
- Create and use custom message types
- Configure Quality of Service settings
- Use ROS 2 command-line tools for debugging

## Lab Duration

Estimated time: 90 minutes

## Prerequisites

- ROS 2 Humble Hawksbill installed
- Basic knowledge of Linux command line
- Python or C++ programming experience

## Scenario: Simple Robot Communication System

You are developing a simple robot communication system where a sensor node publishes environmental data and a controller node subscribes to this data to make decisions. The system will use custom messages and appropriate QoS settings.

## Step 1: Create a Workspace and Package

First, create a new workspace for this lab:

```bash
# Create workspace directory
mkdir -p ~/ros2_lab_ws/src
cd ~/ros2_lab_ws

# Source ROS 2 environment
source /opt/ros/humble/setup.bash
```

Now create a package for our robot communication system:

```bash
cd ~/ros2_lab_ws/src
ros2 pkg create --build-type ament_python robot_communication --dependencies rclpy std_msgs geometry_msgs
```

## Step 2: Create Custom Message Types

Create a directory for custom messages:

```bash
mkdir -p ~/ros2_lab_ws/src/robot_communication/robot_communication/msg
```

Create a custom message file `SensorReading.msg`:

```bash
nano ~/ros2_lab_ws/src/robot_communication/robot_communication/msg/SensorReading.msg
```

Add the following content:

```
# Custom sensor reading message
string sensor_name
float64 value
float64 min_range
float64 max_range
bool is_valid
```

Update the package's `setup.py` to include the message:

```python
import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'robot_communication'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Robot communication package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_node = robot_communication.sensor_node:main',
            'controller_node = robot_communication.controller_node:main',
        ],
    },
)
```

## Step 3: Create the Publisher Node (Sensor Node)

Create the sensor node file:

```bash
nano ~/ros2_lab_ws/src/robot_communication/robot_communication/sensor_node.py
```

Add the following content:

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Header
import random
from robot_communication.msg import SensorReading


class SensorNode(Node):

    def __init__(self):
        super().__init__('sensor_node')

        # Create QoS profile for sensor data
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.publisher = self.create_publisher(SensorReading, 'sensor_data', qos_profile)

        # Timer to publish data every 2 seconds
        self.timer = self.create_timer(2.0, self.publish_sensor_data)

        # Simulate different sensors
        self.sensors = ['front_ultrasonic', 'left_ultrasonic', 'right_ultrasonic', 'temperature']
        self.get_logger().info('Sensor node initialized')

    def publish_sensor_data(self):
        for sensor_name in self.sensors:
            msg = SensorReading()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = sensor_name

            msg.sensor_name = sensor_name

            # Generate simulated sensor values
            if 'ultrasonic' in sensor_name:
                msg.value = random.uniform(0.1, 3.0)  # Distance in meters
                msg.min_range = 0.02
                msg.max_range = 4.0
                msg.is_valid = True
            elif 'temperature' in sensor_name:
                msg.value = random.uniform(18.0, 30.0)  # Temperature in Celsius
                msg.min_range = -10.0
                msg.max_range = 50.0
                msg.is_valid = True

            self.publisher.publish(msg)
            self.get_logger().info(f'Published sensor data: {sensor_name} = {msg.value}')


def main(args=None):
    rclpy.init(args=args)
    sensor_node = SensorNode()

    try:
        rclpy.spin(sensor_node)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Step 4: Create the Subscriber Node (Controller Node)

Create the controller node file:

```bash
nano ~/ros2_lab_ws/src/robot_communication/robot_communication/controller_node.py
```

Add the following content:

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from robot_communication.msg import SensorReading


class ControllerNode(Node):

    def __init__(self):
        super().__init__('controller_node')

        # Create QoS profile matching the publisher
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.subscription = self.create_subscription(
            SensorReading,
            'sensor_data',
            self.sensor_callback,
            qos_profile
        )

        self.get_logger().info('Controller node initialized')

    def sensor_callback(self, msg):
        self.get_logger().info(
            f'Received sensor data: {msg.sensor_name} = {msg.value} '
            f'(Range: {msg.min_range} - {msg.max_range}, Valid: {msg.is_valid})'
        )

        # Simple decision logic based on sensor data
        if 'ultrasonic' in msg.sensor_name and msg.value < 0.5:
            self.get_logger().warn(f'OBSTACLE DETECTED by {msg.sensor_name}! Distance: {msg.value}m')
        elif 'temperature' in msg.sensor_name and msg.value > 28.0:
            self.get_logger().info(f'High temperature detected: {msg.value}°C')


def main(args=None):
    rclpy.init(args=args)
    controller_node = ControllerNode()

    try:
        rclpy.spin(controller_node)
    except KeyboardInterrupt:
        pass
    finally:
        controller_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Step 5: Update package.xml

Update the package.xml file to include message dependencies:

```bash
nano ~/ros2_lab_ws/src/robot_communication/package.xml
```

Update the content to:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>robot_communication</name>
  <version>0.0.0</version>
  <description>Robot communication package</description>
  <maintainer email="your.email@example.com">Your Name</maintainer>
  <license>Apache License 2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

## Step 6: Build the Package

Go back to the workspace root and build the package:

```bash
cd ~/ros2_lab_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select robot_communication
```

Source the workspace:

```bash
source install/setup.bash
```

## Step 7: Run the Nodes

Open a new terminal and run the sensor node:

```bash
cd ~/ros2_lab_ws
source install/setup.bash
ros2 run robot_communication sensor_node
```

Open another terminal and run the controller node:

```bash
cd ~/ros2_lab_ws
source install/setup.bash
ros2 run robot_communication controller_node
```

## Step 8: Use ROS 2 Tools for Monitoring

While the nodes are running, open a third terminal to monitor the system:

```bash
# List all topics
ros2 topic list

# Echo messages from the sensor_data topic
ros2 topic echo /sensor_data robot_communication/msg/SensorReading

# Show information about the sensor_data topic
ros2 topic info /sensor_data

# Show node information
ros2 node list
ros2 node info /sensor_node
ros2 node info /controller_node
```

## Step 9: Advanced Exercise - Add Parameters

Modify the sensor node to use parameters for configuration. Add this to the `__init__` method of the SensorNode class:

```python
# Declare parameters with default values
self.declare_parameter('publish_rate', 2.0)
self.declare_parameter('sensor_range_min', 0.02)
self.declare_parameter('sensor_range_max', 4.0)

# Get parameter values
publish_rate = self.get_parameter('publish_rate').value
self.get_logger().info(f'Setting publish rate to {publish_rate} Hz')
```

Then update the timer creation:

```python
self.timer = self.create_timer(1.0/publish_rate, self.publish_sensor_data)
```

Run the node with custom parameters:

```bash
ros2 run robot_communication sensor_node --ros-args -p publish_rate:=1.0
```

## Troubleshooting Tips

1. **Node not found**: Make sure you've sourced the workspace (`source install/setup.bash`)
2. **Import errors**: Check that your Python files have proper permissions and syntax
3. **Topic not connecting**: Verify that both nodes are using compatible QoS settings
4. **Build errors**: Ensure all dependencies are properly declared in package.xml and setup.py

## Expected Output

When running both nodes, you should see:
- Sensor node publishing simulated sensor readings every 2 seconds
- Controller node receiving and processing the sensor data
- Warning messages when simulated obstacles are detected

## Practice Exercises

### Exercise 1: Extend the Sensor System
1. Add a new sensor type to your `SensorReading.msg` (e.g., humidity, light level).
2. Modify the sensor node to publish this new sensor type with realistic simulated values.
3. Update the controller node to process the new sensor data appropriately.
4. Test that the extended system works correctly with both old and new sensor types.

### Exercise 2: QoS Experimentation
1. Change the QoS settings in both publisher and subscriber to use `RELIABLE` reliability instead of `BEST_EFFORT`.
2. Compare the behavior of the system with different QoS configurations.
3. Document the differences in message delivery and system performance.
4. Experiment with different history policies (`KEEP_ALL` vs `KEEP_LAST`) and observe the effects.

### Exercise 3: Parameter Configuration
1. Add more parameters to your nodes to control sensor behavior (e.g., sensor range limits, warning thresholds).
2. Create a YAML configuration file to set these parameters at launch time.
3. Launch your nodes using the configuration file instead of command-line parameters.
4. Verify that the parameters are loaded correctly by checking the node logs.

### Exercise 4: Multiple Publishers and Subscribers
1. Create a second sensor node that publishes data from a different set of sensors.
2. Modify the controller node to handle messages from both sensor nodes.
3. Add logic to distinguish between sensor data from different sources.
4. Use topic remapping to ensure both sensor nodes can publish to the same controller without conflicts.

### Discussion Questions
1. What are the advantages and disadvantages of using custom message types versus standard message types?
2. How do Quality of Service settings impact the real-time performance of a robotic system?
3. What are the best practices for structuring ROS 2 packages with custom messages?
4. How can you ensure backward compatibility when modifying custom message definitions?

### Challenge Exercise
Extend the robot communication system to include multiple controllers:
- A safety controller that monitors for dangerous conditions and can send emergency stop commands
- A navigation controller that uses sensor data to plan robot movement
- A logging controller that records all sensor data for later analysis

Implement a service-based communication system that allows these controllers to coordinate with each other. Use appropriate QoS settings for each type of communication and implement proper error handling for cases where some nodes become unavailable.

## Key Takeaways

- You've created a complete ROS 2 communication system with custom messages
- You've implemented both publisher and subscriber nodes
- You've configured QoS settings appropriately for sensor data
- You've used ROS 2 command-line tools for monitoring and debugging
- You've learned how to structure a ROS 2 package with custom message types

## References

[ROS Bibliography](/docs/references/ros-bibliography.md)