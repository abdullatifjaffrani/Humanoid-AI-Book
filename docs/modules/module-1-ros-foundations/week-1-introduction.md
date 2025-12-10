---
title: Week 1 - Introduction to ROS 2
sidebar_position: 1
week: 1
module: module-1-ros-foundations
learningObjectives:
  - Understand the fundamental concepts of Robot Operating System (ROS 2)
  - Identify the key differences between ROS 1 and ROS 2
  - Explain the architecture and communication patterns in ROS 2
  - Set up a basic ROS 2 development environment
prerequisites:
  - Basic programming knowledge in Python or C++
  - Understanding of Linux command line
  - Familiarity with version control systems (Git)
description: Introduction to ROS 2, covering its architecture, communication patterns, and setup
---

# Week 1: Introduction to ROS 2

## Learning Objectives

- Understand the fundamental concepts of Robot Operating System (ROS 2)
- Identify the key differences between ROS 1 and ROS 2
- Explain the architecture and communication patterns in ROS 2
- Set up a basic ROS 2 development environment

## Overview

The Robot Operating System 2 (ROS 2) is not an operating system but rather a flexible framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms and configurations.

ROS 2 builds upon the success of ROS 1 while addressing its limitations, particularly in areas of security, real-time performance, and multi-robot systems. It provides a distributed computing framework that enables multiple processes (potentially running on different machines) to interact seamlessly.

## Key Concepts

### What is ROS 2?

ROS 2 is a middleware framework that provides services designed for a heterogeneous computer cluster including:

- Hardware abstraction
- Device drivers
- Libraries
- Visualizers
- Message-passing
- Package management
- Real-time capabilities
- Security features

### ROS 1 vs ROS 2

The transition from ROS 1 to ROS 2 introduced several significant improvements:

1. **Middleware Layer**: ROS 2 uses DDS (Data Distribution Service) as its middleware, providing better scalability and reliability.

2. **Security**: ROS 2 includes built-in security features, supporting authentication, access control, and encryption.

3. **Real-time Support**: Better real-time performance and deterministic behavior.

4. **Multi-robot Systems**: Improved support for multi-robot scenarios.

5. **Quality of Service (QoS)**: Configurable delivery guarantees for messages.

6. **Lifecycle Management**: Better control over node lifecycle and state management.

7. **Official Windows and macOS Support**: No longer Linux-only.

### ROS 2 Architecture

The ROS 2 architecture consists of several layers:

- **Client Libraries**: rclcpp (C++), rclpy (Python), and others
- **ROS Client Library (rcl)**: Thin wrapper around middleware
- **Middleware**: DDS implementation (Fast DDS, Cyclone DDS, RTI Connext)
- **Operating System**: Linux, Windows, macOS

![ROS 2 Architecture Overview](/img/ros_architecture.png)

*Figure 1: The layered architecture of ROS 2 showing the relationship between client libraries, ROS client library, middleware, and operating system.*

### Communication Patterns

ROS 2 supports several communication patterns:

1. **Topics**: Publish/subscribe pattern for asynchronous communication
2. **Services**: Request/response pattern for synchronous communication
3. **Actions**: Goal/feedback/result pattern for long-running tasks
4. **Parameters**: Key/value storage for configuration

## Setting Up ROS 2

### System Requirements

- Ubuntu 22.04 (Jammy) or Windows 10/11 or macOS
- Python 3.8 or higher
- At least 4GB RAM recommended
- At least 10GB free disk space

### Installation

For Ubuntu 22.04 (Humble Hawksbill):

```bash
# Set locale
locale  # check for UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 apt repository
sudo apt update && sudo apt install curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 packages
sudo apt update
sudo apt install ros-humble-desktop
```

### Environment Setup

```bash
# Source the ROS 2 setup script
source /opt/ros/humble/setup.bash

# Add to your shell's startup script
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

## Basic ROS 2 Concepts

### Nodes

A node is an executable that uses ROS 2 to communicate with other nodes. Nodes are organized into packages that can be shared with other users.

### Packages

A package is the fundamental unit of organization in ROS 2. It contains nodes, libraries, and other resources.

### Workspaces

A workspace is a directory where you modify and build ROS 2 packages.

## Creating Your First Workspace

```bash
# Create a workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Build the workspace
colcon build

# Source the workspace
source install/setup.bash
```

## Key Takeaways

- ROS 2 is a middleware framework for robotics development
- It addresses limitations of ROS 1 with DDS middleware, security, and real-time support
- The architecture includes client libraries, rcl, middleware, and OS layers
- Communication patterns include topics, services, actions, and parameters
- Proper environment setup is crucial for development

## Cross-References

This introduction to ROS 2 provides the foundational knowledge needed for:
- [Week 4: Simulation Basics](/docs/modules/module-2-gazebo-unity/week-4-simulation-basics.md) - where ROS 2 nodes communicate with Gazebo simulation
- [Week 6: Isaac Platform Overview](/docs/modules/module-3-nvidia-isaac/week-6-isaac-platform.md) - where ROS 2 integrates with NVIDIA's robotics platform
- [Week 8: Vision Processing](/docs/modules/module-4-vla-systems/week-8-vision-processing.md) - where ROS 2 topics carry image data for computer vision
- [Week 9: Language Integration](/docs/modules/module-4-vla-systems/week-9-language-integration.md) - where ROS 2 services handle natural language processing

## Practice Exercises

### Exercise 1: ROS 2 Environment Setup
1. Install ROS 2 Humble Hawksbill on your development machine following the installation steps provided in this chapter.
2. Verify your installation by running `ros2 --version` in your terminal.
3. Create a new workspace named `my_first_workspace` and build it using `colcon build`.

### Exercise 2: Understanding ROS 2 Architecture
1. Explain in your own words the difference between ROS 1 and ROS 2, focusing on the middleware layer.
2. List the four main communication patterns in ROS 2 and provide an example use case for each.
3. Describe the purpose of Quality of Service (QoS) settings in ROS 2 and why they are important for robotics applications.

### Exercise 3: Basic Commands Practice
1. Use the `ros2 node list` command to see what nodes are currently running on your system.
2. Use the `ros2 topic list` command to see what topics are currently available.
3. Create a simple publisher and subscriber pair using the `talker` and `listener` demo nodes:
   ```bash
   # Terminal 1
   ros2 run demo_nodes_cpp talker

   # Terminal 2
   ros2 run demo_nodes_py listener
   ```

### Discussion Questions
1. What are the main advantages of using DDS as the middleware layer in ROS 2 compared to the custom middleware in ROS 1?
2. Why is security an important consideration for modern robotics systems, and how does ROS 2 address this?
3. How does the layered architecture of ROS 2 (client libraries, rcl, middleware, OS) benefit robotics development?

### Challenge Exercise
Create a simple ROS 2 package called `my_first_package` that contains a basic publisher node that publishes a "Hello World" message to a topic called `my_topic` every second. Include both C++ and Python implementations.

## References

[ROS Bibliography](/docs/references/ros-bibliography.md)