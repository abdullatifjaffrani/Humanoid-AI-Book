---
title: Week 3 - ROS Topics and Messages
sidebar_position: 3
week: 3
module: module-1-ros-foundations
learningObjectives:
  - Understand the publish/subscribe communication pattern in ROS 2
  - Create and use custom message types
  - Implement topic-based communication between nodes
  - Configure Quality of Service (QoS) settings for topics
  - Understand message serialization and data types
prerequisites:
  - Week 1 content: Introduction to ROS 2
  - Week 2 content: ROS Nodes and Services
  - Basic Python or C++ programming knowledge
description: Deep dive into ROS 2 topics and messages, including custom message types and QoS settings
---

# Week 3: ROS Topics and Messages

## Learning Objectives

- Understand the publish/subscribe communication pattern in ROS 2
- Create and use custom message types
- Implement topic-based communication between nodes
- Configure Quality of Service (QoS) settings for topics
- Understand message serialization and data types

## Overview

Topics in ROS 2 implement a publish/subscribe communication pattern where nodes can publish messages to a topic and subscribe to messages from a topic. This asynchronous communication pattern is fundamental to ROS and enables loose coupling between nodes. Messages are the data structures that are passed between nodes through topics.

## Publish/Subscribe Pattern

### Concept

The publish/subscribe pattern in ROS 2 works as follows:

1. **Publisher**: A node that sends messages to a topic
2. **Subscriber**: A node that receives messages from a topic
3. **Topic**: A named channel through which messages are sent

This pattern enables asynchronous communication where publishers and subscribers don't need to know about each other directly.

### Basic Example in Python

Publisher:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Subscriber:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Message Types

### Standard Messages

ROS 2 comes with a variety of standard message types in packages like:

- `std_msgs`: Basic data types (String, Int32, Float64, etc.)
- `geometry_msgs`: Geometric primitives (Point, Pose, Twist, etc.)
- `sensor_msgs`: Sensor data (LaserScan, Image, JointState, etc.)
- `nav_msgs`: Navigation-specific messages (Odometry, Path, OccupancyGrid, etc.)

### Message Definition Syntax

Messages are defined using `.msg` files with the following syntax:

```
# Comment
string name
int32 age
float64 height
bool active
---
# Additional comments
```

### Custom Message Example

Create a file `CustomMessage.msg`:
```
# Custom robot status message
string robot_name
int32 battery_level
bool is_operational
geometry_msgs/Point current_position
```

## Quality of Service (QoS)

### QoS Settings

QoS settings allow you to configure the delivery guarantees for messages. Key settings include:

- **Reliability**: Best effort vs reliable delivery
- **Durability**: Volatile vs transient local
- **History**: Keep all vs keep last N messages
- **Deadline**: Maximum time between messages
- **Liveliness**: How to detect if a publisher is alive

### Python QoS Example

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String

class QoSPublisher(Node):

    def __init__(self):
        super().__init__('qos_publisher')

        # Create a QoS profile
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.publisher_ = self.create_publisher(String, 'qos_topic', qos_profile)
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'QoS Message: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
```

### C++ QoS Example

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/qos.hpp"

class QoSPublisher : public rclcpp::Node
{
public:
  QoSPublisher()
  : Node("qos_publisher"), count_(0)
  {
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos_profile.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);

    publisher_ = this->create_publisher<std_msgs::msg::String>("qos_topic", qos_profile);
    timer_ = this->create_wall_timer(
      1s, std::bind(&QoSPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "QoS Message: " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};
```

## Creating Custom Messages

### Package Structure

To create custom messages, you need to:

1. Create a package for your messages
2. Add the message definitions to `msg/` directory
3. Update `CMakeLists.txt` and `package.xml`

### Example Package Structure

```
my_robot_msgs/
├── CMakeLists.txt
├── package.xml
└── msg/
    ├── RobotStatus.msg
    └── SensorArray.msg
```

### CMakeLists.txt Configuration

```cmake
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotStatus.msg"
  "msg/SensorArray.msg"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)
```

### package.xml Configuration

```xml
<buildtool_depend>ament_cmake</buildtool_depend>
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

## Message Serialization

### Serialization Process

When messages are sent over topics, they are serialized into a binary format. The serialization process includes:

1. **Type Information**: Message type identification
2. **Data Encoding**: Converting data to binary format
3. **Transport**: Sending over the middleware (DDS)
4. **Deserialization**: Converting back to usable format

### Performance Considerations

- **Message Size**: Large messages can impact performance
- **Frequency**: High-frequency topics need efficient serialization
- **Complexity**: Complex nested messages take more processing time

## Advanced Topics

### Topic Remapping

Topics can be remapped at runtime:

```bash
# Remap topic at launch
ros2 run package_name node_name --ros-args --remap old_topic:=new_topic

# In code
from rclpy.node import Node

class RemappedNode(Node):
    def __init__(self):
        super().__init__('remapped_node')
        # This will subscribe to 'remapped_topic' instead of 'original_topic'
        self.subscription = self.create_subscription(
            String,
            'original_topic',
            self.callback,
            10,
            callback_group=None)
```

### Topic Monitoring

Monitor topic activity:

```bash
# List all topics
ros2 topic list

# Echo messages from a topic
ros2 topic echo /topic_name std_msgs/msg/String

# Show topic info
ros2 topic info /topic_name

# Publish to a topic from command line
ros2 topic pub /topic_name std_msgs/msg/String "data: 'Hello'"
```

## Best Practices

### Topic Design

1. **Naming Conventions**: Use consistent, descriptive names
2. **Message Frequency**: Balance between responsiveness and performance
3. **Message Size**: Keep messages as small as possible while maintaining necessary information
4. **Topic Organization**: Group related topics with common prefixes
5. **QoS Selection**: Choose appropriate QoS settings based on requirements

### Message Design

1. **Data Types**: Use appropriate data types for your needs
2. **Extensibility**: Design messages that can be extended without breaking compatibility
3. **Documentation**: Document message fields clearly
4. **Validation**: Consider field validation requirements

## Key Takeaways

- Topics implement the publish/subscribe communication pattern
- Messages are the data structures passed between nodes
- Custom messages can be created to represent domain-specific data
- QoS settings allow fine-tuning of communication guarantees
- Proper message design is crucial for system performance

## Practice Exercises

### Exercise 1: Custom Message Creation
1. Create a new package called `my_custom_msgs` for your custom message definitions.
2. Define a custom message called `RobotSensorData.msg` that includes fields for temperature (float64), humidity (float64), and sensor_status (string).
3. Build the package with `colcon build --packages-select my_custom_msgs`.
4. Create a publisher node that publishes messages of your custom type with simulated sensor data.
5. Create a subscriber node that receives and displays the custom message data.

### Exercise 2: QoS Configuration and Testing
1. Create two publisher nodes: one with reliable QoS settings and one with best-effort settings.
2. Create corresponding subscriber nodes for each publisher.
3. Test the behavior of each combination under network stress or when subscribers start after publishers.
4. Document the differences in message delivery between the two QoS configurations.

### Exercise 3: Topic Remapping and Monitoring
1. Create a publisher node that publishes messages to a topic called `sensor_data`.
2. Launch the node with a remapped topic name using the command line argument `--ros-args --remap sensor_data:=robot_sensor_data`.
3. Use `ros2 topic echo` to verify that the messages are being published to the remapped topic.
4. Practice using other topic monitoring commands like `ros2 topic info` and `ros2 topic hz`.

### Exercise 4: Advanced Topic Communication
1. Implement a sensor fusion system with multiple publishers (simulating different sensors) publishing to separate topics.
2. Create a subscriber node that subscribes to all sensor topics and publishes a fused result to another topic.
3. Use appropriate QoS settings for each type of sensor data based on its criticality.
4. Add logging to track the timing and sequence of messages from different sensors.

### Discussion Questions
1. When would you choose best-effort QoS over reliable QoS for a topic? Provide specific use cases.
2. How do QoS settings affect the real-time performance of a ROS 2 system?
3. What are the trade-offs between message size and frequency in topic communication?
4. How can topic remapping be used effectively in different deployment scenarios?

### Challenge Exercise
Design and implement a complete sensor network simulation:
- Create 3-4 different sensor message types (e.g., temperature, proximity, camera, IMU)
- Implement publisher nodes that simulate realistic sensor data
- Create a central monitoring node that subscribes to all sensor topics
- Implement a data logging node that records all sensor data with timestamps
- Use appropriate QoS settings for each sensor type based on its criticality
- Add error handling for cases where sensor publishers are not available

Ensure your system can handle dynamic addition/removal of sensor nodes and maintain reliable communication for critical sensors while optimizing performance for non-critical ones.

## References

[ROS Bibliography](/docs/references/ros-bibliography.md)