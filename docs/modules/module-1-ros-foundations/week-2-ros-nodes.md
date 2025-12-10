---
title: Week 2 - ROS Nodes and Services
sidebar_position: 2
week: 2
module: module-1-ros-foundations
learningObjectives:
  - Understand the concept and implementation of ROS 2 nodes
  - Create and run nodes using both Python and C++
  - Implement service-based communication between nodes
  - Use parameter servers for node configuration
  - Understand node lifecycle management
prerequisites:
  - Week 1 content: Introduction to ROS 2
  - Basic Python or C++ programming knowledge
  - Understanding of object-oriented programming
description: Deep dive into ROS 2 nodes and services, including implementation and communication
---

# Week 2: ROS Nodes and Services

## Learning Objectives

- Understand the concept and implementation of ROS 2 nodes
- Create and run nodes using both Python and C++
- Implement service-based communication between nodes
- Use parameter servers for node configuration
- Understand node lifecycle management

## Overview

In ROS 2, nodes are the fundamental building blocks of a ROS system. They are processes that perform computation and communicate with other nodes through topics, services, actions, and parameters. This week, we'll explore how to create, configure, and manage nodes, as well as implement service-based communication patterns.

## ROS 2 Nodes

### What is a Node?

A node is an executable that uses ROS 2 to communicate with other nodes. Nodes are organized into packages that can be shared with other users. Nodes can publish messages to topics, subscribe to topics to receive messages, provide services, make requests to services, and more.

### Node Structure

Every ROS 2 node typically includes:

1. **Initialization**: Setting up the ROS context and node
2. **Communication interfaces**: Publishers, subscribers, services, etc.
3. **Main loop or callbacks**: Processing logic
4. **Cleanup**: Proper shutdown procedures

### Creating Nodes in Python

Here's a basic Python node example:

```python
import rclpy
from rclpy.node import Node

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

### Creating Nodes in C++

Here's the equivalent C++ node:

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello World: " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclpy::init(argc, argv);
  rclpy::spin(std::make_shared<MinimalPublisher>());
  rclpy::shutdown();
  return 0;
}
```

## Services

### Service-Based Communication

Services provide a request/response communication pattern in ROS 2. A service client sends a request to a service server, which processes the request and sends back a response. This is synchronous communication.

### Service Definition

Services are defined using `.srv` files with the following structure:
```
# Request
string name
int32 age
---
# Response
bool success
string message
```

### Creating a Service Server in Python

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Creating a Service Client in Python

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main():
    rclpy.init()
    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(1, 2)
    minimal_client.get_logger().info(
        'Result of add_two_ints: %d' % response.sum)
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Parameters

### Parameter Server

ROS 2 includes a parameter server that allows nodes to store and retrieve configuration values. Parameters can be:

- Declared during node initialization
- Set at runtime
- Remapped from the command line
- Loaded from configuration files

### Using Parameters in Python

```python
import rclpy
from rclpy.node import Node

class ParameterNode(Node):

    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with default values
        self.declare_parameter('my_parameter', 'default_value')
        self.declare_parameter('another_parameter', 10)

        # Get parameter values
        my_param = self.get_parameter('my_parameter').value
        another_param = self.get_parameter('another_parameter').value

        self.get_logger().info(f'My parameter: {my_param}')
        self.get_logger().info(f'Another parameter: {another_param}')

def main(args=None):
    rclpy.init(args=args)
    parameter_node = ParameterNode()
    rclpy.spin(parameter_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Node Lifecycle

### Lifecycle Nodes

ROS 2 provides lifecycle nodes that have a well-defined state machine with states like:

- Unconfigured
- Inactive
- Active
- Finalized

This is useful for complex systems that need proper initialization, cleanup, and state management.

### Basic Lifecycle

```python
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn

class LifecyclePublisher(LifecycleNode):

    def __init__(self):
        super().__init__('lifecycle_publisher')
        self.pub = None

    def on_configure(self, state):
        self.pub = self.create_publisher(String, 'lifecycle_chatter', 10)
        self.get_logger().info(f'Configured lifecycle publisher in state {state.id}')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.pub.activate()
        self.get_logger().info(f'Activated lifecycle publisher in state {state.id}')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        self.pub.deactivate()
        self.get_logger().info(f'Deactivated lifecycle publisher in state {state.id}')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        self.destroy_publisher(self.pub)
        self.get_logger().info(f'Cleaned up lifecycle publisher in state {state.id}')
        return TransitionCallbackReturn.SUCCESS
```

## Best Practices

### Node Design Principles

1. **Single Responsibility**: Each node should have a clear, single purpose
2. **Error Handling**: Implement proper error handling and recovery
3. **Logging**: Use appropriate logging levels (info, warn, error, debug)
4. **Resource Management**: Properly clean up resources on shutdown
5. **Configuration**: Use parameters for configurable values
6. **Testing**: Write unit tests for your nodes

### Communication Best Practices

1. **Topic Names**: Use descriptive, consistent naming conventions
2. **Message Types**: Choose appropriate message types for your data
3. **QoS Settings**: Configure Quality of Service settings appropriately
4. **Service Usage**: Use services for synchronous, request-response communication
5. **Parameter Validation**: Validate parameters at runtime

## Key Takeaways

- Nodes are the fundamental building blocks of ROS 2 systems
- Services provide synchronous request/response communication
- Parameters enable configurable node behavior
- Lifecycle nodes provide structured state management
- Proper design and error handling are crucial for robust systems

## Practice Exercises

### Exercise 1: Create a Simple Publisher Node
1. Create a new ROS 2 package called `my_nodes` using `ros2 pkg create --build-type ament_python my_nodes`.
2. Implement a Python publisher node that publishes your name and current timestamp to a topic called `my_info`.
3. Build your package using `colcon build --packages-select my_nodes`.
4. Run your publisher node and verify it's publishing messages using `ros2 topic echo /my_info`.

### Exercise 2: Create a Service Server and Client
1. Create a custom service definition file called `CalculateArea.srv` that takes two floats (length and width) and returns one float (area).
2. Implement a service server that calculates the area of a rectangle.
3. Implement a service client that sends requests to your server with different length and width values.
4. Test your service by running the server and client nodes and verifying the calculations.

### Exercise 3: Parameter Configuration
1. Create a node that declares at least 3 different parameter types (string, integer, boolean).
2. Launch your node with different parameter values using a launch file.
3. Create a configuration file (YAML) that sets parameter values for your node.
4. Run your node using the configuration file and verify the parameters are loaded correctly.

### Exercise 4: Lifecycle Node Implementation
1. Implement a lifecycle node that manages a simple counter.
2. Use the lifecycle states to control when the counter increments.
3. Use the lifecycle manager to transition your node through different states.
4. Observe how the node behaves differently in each state.

### Discussion Questions
1. What are the advantages of using lifecycle nodes over regular nodes? When would you choose one over the other?
2. Compare and contrast the different communication patterns in ROS 2 (topics, services, parameters). When would you use each one?
3. How do parameters improve the reusability and configurability of ROS 2 nodes?
4. What are the key differences between synchronous service calls and asynchronous topic publishing in terms of system design?

### Challenge Exercise
Create a complete ROS 2 system with multiple nodes that simulate a simple robot patrol system:
- A patrol coordinator node that manages patrol routes
- A sensor node that publishes simulated sensor readings
- A decision node that processes sensor data and sends alerts if obstacles are detected
- A logging node that records patrol events

Use appropriate topics, services, and parameters to coordinate between the nodes. Include error handling and proper shutdown procedures.

## References

[ROS Bibliography](/docs/references/ros-bibliography.md)