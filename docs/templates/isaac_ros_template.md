---
title: Isaac ROS Package Template
sidebar_position: 2
---

# Isaac ROS Package Template

## Overview

This template provides a starting point for creating new Isaac ROS packages that leverage NVIDIA's hardware acceleration capabilities. It includes the basic structure, configuration files, and example nodes optimized for GPU-accelerated robotics applications.

## Package Structure

```
isaac_robot_package/
├── CMakeLists.txt              # Build configuration for C++ packages
├── package.xml                 # Package metadata and dependencies
├── setup.py                    # Build configuration for Python packages
├── setup.cfg                   # Installation configuration
├── isaac_robot_package/        # Python package directory
│   ├── __init__.py            # Python package initialization
│   ├── accelerated_node.py    # Example accelerated Python node
│   └── gpu_processor.py       # Example GPU processor component
├── src/                       # C++ source files
│   ├── accelerated_node.cpp   # Example accelerated C++ node
│   └── cuda_processor.cu      # Example CUDA kernel
├── include/isaac_robot_package/ # C++ header files
│   ├── accelerated_node.hpp   # Example accelerated node header
│   └── cuda_processor.cuh     # Example CUDA header
├── launch/                    # Launch files
│   └── accelerated_launch.py  # Example launch file with Isaac nodes
├── config/                    # Configuration files
│   ├── accelerated_params.yaml # Example accelerated parameters
│   └── gpu_config.yaml        # GPU-specific configuration
├── isaac/                     # Isaac-specific files
│   ├── extensions/            # Isaac extension files
│   └── schemas/               # Isaac schema definitions
├── test/                      # Test files
│   ├── test_accelerated.py    # Example accelerated test
│   └── test_gpu.cpp           # Example GPU test
└── scripts/                   # Utility scripts
    └── gpu_monitor.sh         # GPU monitoring script
```

## Template Files

### package.xml

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>isaac_robot_package</name>
  <version>0.0.0</version>
  <description>Isaac Robot Package with GPU Acceleration</description>
  <maintainer email="maintainer@todo.todo">Maintainer Name</maintainer>
  <license>TODO License Declaration</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>ament_python</buildtool_depend>

  <!-- Standard ROS 2 dependencies -->
  <depend>rclpy</depend>
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>nav_msgs</depend>

  <!-- Isaac ROS dependencies -->
  <depend>isaac_ros_common</depend>
  <depend>isaac_ros_visual_slam</depend>
  <depend>isaac_ros_apriltag</depend>
  <depend>isaac_ros_pointcloud_utils</depend>

  <!-- GPU-related dependencies -->
  <depend>cuda_dev_tools</depend>

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
project(isaac_robot_package)

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
find_package(nav_msgs REQUIRED)

# Find Isaac ROS packages
find_package(isaac_ros_common REQUIRED)
find_package(isaac_ros_visual_slam REQUIRED)
find_package(isaac_ros_apriltag REQUIRED)

# Find CUDA
find_package(CUDA REQUIRED)

# Enable CUDA
enable_language(CUDA)

# Include directories
include_directories(include)

# Example accelerated C++ executable
add_executable(accelerated_node
  src/accelerated_node.cpp
  src/cuda_processor.cu
)
target_include_directories(accelerated_node PRIVATE ${CUDA_INCLUDE_DIRS})
target_link_libraries(accelerated_node ${CUDA_LIBRARIES})

# Link Isaac ROS libraries
ament_target_dependencies(accelerated_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
)

# Install targets
install(TARGETS
  accelerated_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install launch files
install(DIRECTORY
  launch
  config
  isaac
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

### Example Accelerated Python Node

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import numpy as np
import cv2
from cv_bridge import CvBridge
import time


class IsaacAcceleratedNode(Node):
    def __init__(self):
        super().__init__('isaac_accelerated_node')

        # Initialize CvBridge for image conversion
        self.bridge = CvBridge()

        # Publishers and subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.gpu_usage_pub = self.create_publisher(
            Float32,
            '/performance/gpu_usage',
            10
        )

        # Performance tracking
        self.processing_times = []
        self.frame_count = 0
        self.last_process_time = time.time()

        # Camera info
        self.camera_matrix = None
        self.distortion_coeffs = None

        # Timers
        self.gpu_monitor_timer = self.create_timer(1.0, self.monitor_gpu)

        self.get_logger().info('Isaac Accelerated Node initialized')

    def image_callback(self, msg):
        """Process incoming image with potential GPU acceleration."""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            # Start timing for performance measurement
            start_time = time.time()

            # Process image (in real implementation, this would use GPU acceleration)
            processed_image = self.accelerated_image_processing(cv_image)

            # Calculate processing time
            process_time = time.time() - start_time
            self.processing_times.append(process_time)

            # Limit the list size to last 100 measurements
            if len(self.processing_times) > 100:
                self.processing_times.pop(0)

            # Calculate average processing time
            avg_time = sum(self.processing_times) / len(self.processing_times) if self.processing_times else 0
            fps = 1.0 / avg_time if avg_time > 0 else 0

            self.get_logger().info(f'Processed frame {self.frame_count}, '
                                 f'Avg time: {avg_time*1000:.2f}ms, '
                                 f'FPS: {fps:.2f}')
            self.frame_count += 1

        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def camera_info_callback(self, msg):
        """Store camera calibration information."""
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.distortion_coeffs = np.array(msg.d)

    def accelerated_image_processing(self, image):
        """
        Placeholder for GPU-accelerated image processing.
        In a real Isaac implementation, this would use CUDA kernels
        or TensorRT for acceleration.
        """
        # Simulate some processing (in real implementation, use GPU)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)

        return edges

    def monitor_gpu(self):
        """Monitor GPU usage (placeholder - in real system would query GPU)."""
        # In a real implementation, this would query actual GPU usage
        # For now, we'll simulate GPU usage
        import random
        gpu_usage = Float32()
        gpu_usage.data = random.uniform(30.0, 90.0)  # Simulated GPU usage %
        self.gpu_usage_pub.publish(gpu_usage)


def main(args=None):
    """Main function to run the Isaac accelerated node."""
    rclpy.init(args=args)
    node = IsaacAcceleratedNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Isaac-Specific Launch File

```python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    enable_gpu = LaunchConfiguration('enable_gpu', default='true')

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_enable_gpu = DeclareLaunchArgument(
        'enable_gpu',
        default_value='true',
        description='Enable GPU acceleration'
    )

    # Isaac Visual SLAM node
    visual_slam_node = Node(
        package='isaac_ros_visual_slam',
        executable='visual_slam_node',
        parameters=[{
            'enable_slam_3d': False,
            'use_sim_time': use_sim_time,
            'enable_gpu_acceleration': enable_gpu
        }],
        remappings=[
            ('/visual_slam/camera/left/image_rect', '/camera/image_raw'),
            ('/visual_slam/camera/left/camera_info', '/camera/camera_info'),
        ],
        output='screen'
    )

    # Isaac accelerated node
    accelerated_node = Node(
        package='isaac_robot_package',
        executable='accelerated_node',
        name='accelerated_node',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # Create launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_enable_gpu)

    # Add nodes
    ld.add_action(visual_slam_node)
    ld.add_action(accelerated_node)

    return ld
```

## Isaac-Specific Configuration

### Accelerated Parameters

```yaml
accelerated_node:
  ros__parameters:
    # GPU acceleration settings
    enable_gpu_acceleration: true
    gpu_device_id: 0
    max_gpu_memory: 4096  # MB

    # Processing settings
    processing_rate: 30.0  # Hz
    image_width: 640
    image_height: 480

    # Performance monitoring
    enable_performance_monitoring: true
    performance_publish_rate: 10.0  # Hz
```

## GPU Monitoring Script

```bash
#!/bin/bash
# gpu_monitor.sh - Monitor GPU usage for Isaac applications

# Check if nvidia-smi is available
if ! command -v nvidia-smi &> /dev/null; then
    echo "nvidia-smi not found. GPU monitoring unavailable."
    exit 1
fi

# Monitor GPU usage
echo "Monitoring GPU usage for Isaac applications..."
while true; do
    # Get GPU utilization percentage
    gpu_util=$(nvidia-smi --query-gpu=utilization.gpu --format=csv,noheader,nounits)
    gpu_memory=$(nvidia-smi --query-gpu=memory.used,memory.total --format=csv,noheader,nounits)

    echo "GPU Utilization: ${gpu_util}%"
    echo "GPU Memory: ${gpu_memory}"
    echo "---"

    sleep 2
done
```

## Usage Instructions

1. **Copy the template**: Copy this structure to your new Isaac ROS package directory
2. **Replace placeholders**: Replace `isaac_robot_package` with your actual package name
3. **Update metadata**: Modify package.xml with your specific information and Isaac dependencies
4. **Customize code**: Modify the example nodes to implement your accelerated functionality
5. **Configure GPU**: Update parameters for your specific GPU hardware
6. **Build and test**: Use `colcon build` to build your Isaac package

## Isaac-Specific Best Practices

- **GPU Memory Management**: Efficiently allocate and deallocate GPU memory
- **Pipeline Optimization**: Minimize data transfers between CPU and GPU
- **Batch Processing**: Process multiple frames simultaneously when possible
- **Asynchronous Execution**: Use asynchronous processing where applicable
- **Real-time Constraints**: Ensure deterministic processing times for time-critical applications
- **Hardware Compatibility**: Verify compatibility with supported NVIDIA GPUs
- **Performance Monitoring**: Continuously monitor GPU utilization and performance

## Common Isaac Customizations

### For Different Applications

- **SLAM**: Use Isaac Visual SLAM with GPU acceleration
- **Perception**: Implement accelerated computer vision algorithms
- **Navigation**: Use GPU-accelerated path planning
- **Manipulation**: Accelerate grasp planning and control

### For Different Hardware

- **Jetson Platforms**: Optimize for embedded GPU constraints
- **Data Center GPUs**: Leverage high-performance compute capabilities
- **Mobile GPUs**: Balance performance with power consumption