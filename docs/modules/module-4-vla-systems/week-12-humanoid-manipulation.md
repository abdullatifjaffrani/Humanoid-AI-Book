---
title: Week 12 - Humanoid Manipulation and Grasping
sidebar_position: 5
week: 12
module: module-4-vla-systems
learningObjectives:
  - Design and implement grasping strategies for humanoid robots
  - Integrate vision-based object detection with manipulation planning
  - Implement multi-fingered hand control for dexterous manipulation
  - Apply force control and tactile feedback in manipulation tasks
prerequisites:
  - Week 1-11 content: Complete textbook modules
  - Understanding of kinematics and dynamics
  - Knowledge of computer vision and object detection
  - Experience with ROS 2 control systems
description: Advanced manipulation and grasping techniques for humanoid robots with dexterous hands
---

# Week 12: Humanoid Manipulation and Grasping

## Learning Objectives

By the end of this week, students will be able to:
- Explain the principles of robotic grasping and manipulation
- Implement vision-based grasp planning algorithms
- Design control strategies for dexterous manipulation
- Integrate tactile feedback with manipulation systems
- Apply force control techniques for safe and robust grasping

## Overview

Humanoid manipulation represents one of the most challenging and important aspects of humanoid robotics. This week explores the fundamental principles and practical implementations of manipulation and grasping for humanoid robots, including dexterous hand control, vision-based grasp planning, and force control techniques.

## Fundamentals of Robotic Grasping

### Grasp Types and Taxonomy

Robotic grasps can be classified into several categories:

1. **Power Grasps**: Provide maximum stability and force transmission
   - Cylindrical grasp: Wrapping fingers around cylindrical objects
   - Spherical grasp: Encircling spherical objects
   - Hook grasp: Using fingertips to grasp objects

2. **Precision Grasps**: Enable fine manipulation with minimal contact
   - Tip pinch: Grasping with fingertips
   - Lateral pinch: Grasping between thumb and side of index finger
   - Tripod grasp: Grasping with thumb, index, and middle fingers

3. **Intermediate Grasps**: Balance between power and precision

### Grasp Quality Metrics

Evaluating grasp quality is essential for successful manipulation:

```python
import numpy as np
from scipy.spatial.transform import Rotation as R

def calculate_grasp_quality(contact_points, normals, forces, object_mass, gravity=9.81):
    """
    Calculate various grasp quality metrics.

    Args:
        contact_points: Array of contact points [N x 3]
        normals: Array of surface normals at contact points [N x 3]
        forces: Array of applied forces at contact points [N x 3]
        object_mass: Mass of the object being grasped

    Returns:
        Dictionary of quality metrics
    """
    # Calculate grasp matrix (for 3D objects)
    grasp_matrix = calculate_grasp_matrix(contact_points, normals)

    # Minimum singular value (related to grasp stability)
    min_singular_value = np.min(np.linalg.svd(grasp_matrix, compute_uv=False))

    # Force closure test
    force_closure = check_force_closure(contact_points, normals)

    # Grasp wrench space volume
    wrench_space_volume = calculate_wrench_space_volume(contact_points, normals)

    # Gravitational stability (can grasp hold object against gravity)
    gravity_force = object_mass * gravity
    vertical_support = calculate_vertical_support(forces, gravity_force)

    return {
        'min_singular_value': min_singular_value,
        'force_closure': force_closure,
        'wrench_space_volume': wrench_space_volume,
        'vertical_support_ratio': vertical_support / gravity_force if gravity_force > 0 else 0,
        'grasp_stability_index': min_singular_value * wrench_space_volume
    }

def calculate_grasp_matrix(contact_points, normals):
    """
    Calculate the grasp matrix for a given set of contact points and normals.
    The grasp matrix relates joint torques to wrenches applied to the object.
    """
    n_contacts = len(contact_points)
    grasp_matrix = np.zeros((6, 2*n_contacts))  # 6 DOF wrench, 2 forces per contact

    for i, (point, normal) in enumerate(zip(contact_points, normals)):
        # Position vector from object center to contact point
        r = point

        # Normal and tangential directions
        n = normal / np.linalg.norm(normal)  # Normal vector
        # Create tangential vectors (simplified)
        t1 = np.array([1, 0, 0]) if abs(n[0]) < 0.9 else np.array([0, 1, 0])
        t1 = t1 - np.dot(t1, n) * n
        t1 = t1 / np.linalg.norm(t1)
        t2 = np.cross(n, t1)

        # Wrench due to normal force
        grasp_matrix[0:3, 2*i] = n      # Force
        grasp_matrix[3:6, 2*i] = np.cross(r, n)  # Torque

        # Wrench due to tangential force (friction)
        grasp_matrix[0:3, 2*i+1] = t1
        grasp_matrix[3:6, 2*i+1] = np.cross(r, t1)

    return grasp_matrix

def check_force_closure(contact_points, normals, friction_coeff=0.5):
    """
    Check if a grasp provides force closure (can resist any external wrench).
    Simplified implementation using linear programming approach.
    """
    # This is a simplified version - full implementation would use linear programming
    # to check if origin is in interior of convex hull of primitive wrenches
    n_contacts = len(contact_points)

    # For 3D objects, force closure requires at least 7 contact points
    # with appropriate geometry
    if n_contacts < 7:
        return False

    # Additional checks would go here
    # This is a simplified geometric check
    return True

def calculate_wrench_space_volume(contact_points, normals):
    """
    Calculate the volume of the grasp wrench space.
    This is a simplified implementation.
    """
    grasp_matrix = calculate_grasp_matrix(contact_points, normals)
    # Volume related to determinant of grasp matrix
    try:
        # For rectangular matrices, we use the product of singular values
        singular_values = np.linalg.svd(grasp_matrix, compute_uv=False)
        volume = np.prod(singular_values)
        return volume
    except:
        return 0.0

def calculate_vertical_support(forces, required_force):
    """
    Calculate how much vertical force the grasp can support.
    """
    total_vertical_force = sum(f[2] for f in forces)  # Assuming z is up
    return total_vertical_force
```

## Vision-Based Grasp Planning

### Object Detection and Pose Estimation

Integrating vision with grasp planning:

```python
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R

class VisionBasedGraspPlanner:
    def __init__(self):
        # Initialize object detection model
        self.detection_model = self.load_detection_model()
        # Initialize pose estimation
        self.pose_estimator = self.load_pose_estimator()

    def load_detection_model(self):
        """Load pre-trained object detection model."""
        # This would load a model like YOLO, SSD, or Faster R-CNN
        # For this example, we'll create a placeholder
        return None

    def load_pose_estimator(self):
        """Load object pose estimation model."""
        # This would load a model for 6D pose estimation
        return None

    def detect_and_estimate_pose(self, image):
        """
        Detect objects in image and estimate their 3D poses.

        Args:
            image: Input image (numpy array)

        Returns:
            List of objects with bounding boxes, 3D poses, and properties
        """
        # Perform object detection
        detections = self.perform_object_detection(image)

        objects = []
        for detection in detections:
            obj = {
                'bbox': detection['bbox'],
                'class': detection['class'],
                'confidence': detection['confidence'],
                'pose': self.estimate_object_pose(image, detection['bbox']),
                'dimensions': self.estimate_object_dimensions(detection['bbox'])
            }
            objects.append(obj)

        return objects

    def perform_object_detection(self, image):
        """Perform object detection on image."""
        # Placeholder implementation
        # In practice, this would use a deep learning model
        # Return list of detections with bbox [x, y, w, h], class, confidence
        return []

    def estimate_object_pose(self, image, bbox):
        """Estimate 6D pose (position and orientation) of object."""
        # Placeholder implementation
        # This would use a pose estimation algorithm
        # Return [x, y, z, qx, qy, qz, qw] (position + quaternion)
        return [0, 0, 0, 0, 0, 0, 1]

    def estimate_object_dimensions(self, bbox):
        """Estimate 3D dimensions of object from 2D bounding box."""
        # Placeholder implementation
        # This would use additional information or assumptions
        return [0.1, 0.1, 0.1]  # width, height, depth

    def plan_grasp(self, object_info, hand_model):
        """
        Plan grasp for detected object based on its properties.

        Args:
            object_info: Dictionary with object information
            hand_model: Model of the robotic hand

        Returns:
            Grasp configuration (joint angles, contact points, etc.)
        """
        obj_pose = object_info['pose']
        obj_dims = object_info['dimensions']
        obj_class = object_info['class']

        # Determine appropriate grasp type based on object properties
        grasp_type = self.select_grasp_type(obj_class, obj_dims)

        # Calculate grasp pose relative to object
        grasp_pose = self.calculate_grasp_pose(obj_pose, obj_dims, grasp_type)

        # Generate hand configuration for grasp
        hand_config = self.generate_hand_configuration(grasp_pose, grasp_type, hand_model)

        return {
            'grasp_pose': grasp_pose,
            'hand_configuration': hand_config,
            'grasp_type': grasp_type,
            'quality_score': self.estimate_grasp_quality(grasp_pose, obj_dims)
        }

    def select_grasp_type(self, obj_class, obj_dims):
        """Select appropriate grasp type based on object class and dimensions."""
        # Define rules for grasp selection
        aspect_ratio = max(obj_dims) / min(obj_dims)

        if obj_class in ['bottle', 'cup', 'cylinder']:
            return 'cylindrical'
        elif obj_class in ['box', 'book', 'phone'] and aspect_ratio < 3:
            return 'parallel'
        elif obj_class in ['pen', 'pencil', 'stick']:
            return 'cylindrical'
        elif aspect_ratio > 5:  # Thin objects
            return 'tip_pinch'
        else:
            return 'tripod'  # Default precision grasp

    def calculate_grasp_pose(self, obj_pose, obj_dims, grasp_type):
        """Calculate the grasp pose relative to the object."""
        # Extract object position and orientation
        obj_pos = obj_pose[:3]
        obj_rot = R.from_quat(obj_pose[3:])

        # Calculate grasp position based on grasp type
        if grasp_type == 'cylindrical':
            # Grasp around the cylinder
            grasp_pos = obj_pos.copy()
            grasp_pos[2] += max(obj_dims[2], 0.05)  # Lift slightly above center
        elif grasp_type == 'parallel':
            # Grasp from the side
            grasp_pos = obj_pos.copy()
            grasp_pos[0] += obj_dims[0] / 2 + 0.02  # 2cm from surface
        elif grasp_type == 'tip_pinch':
            # Grasp at the end
            grasp_pos = obj_pos.copy()
            grasp_pos[0] += obj_dims[0] / 2  # At the end
        else:  # tripod or other precision grasp
            grasp_pos = obj_pos.copy()
            grasp_pos[2] += obj_dims[2] / 2 + 0.01  # Slightly above

        # Calculate grasp orientation (fingers pointing toward object center)
        approach_dir = np.array([1, 0, 0])  # Default approach direction
        grasp_rot = obj_rot * R.from_rotvec(approach_dir)  # Adjust based on object orientation

        # Combine position and orientation
        grasp_pose = np.zeros(7)
        grasp_pose[:3] = grasp_pos
        grasp_pose[3:] = grasp_rot.as_quat()

        return grasp_pose

    def generate_hand_configuration(self, grasp_pose, grasp_type, hand_model):
        """Generate hand joint configuration for the grasp."""
        # This would interface with inverse kinematics and hand model
        # For now, return a placeholder configuration
        if grasp_type == 'cylindrical':
            # Configuration for cylindrical grasp
            return [0.5, 0.5, 0.5, 0.5, 0.5]  # Example joint angles
        elif grasp_type == 'parallel':
            # Configuration for parallel grasp
            return [0.2, 0.2, 0.2, 0.2, 0.2]
        elif grasp_type == 'tip_pinch':
            # Configuration for tip pinch
            return [0.8, 0.0, 0.0, 0.0, 0.8]
        else:
            # Default tripod grasp
            return [0.3, 0.3, 0.3, 0.0, 0.7]

    def estimate_grasp_quality(self, grasp_pose, obj_dims):
        """Estimate the quality of the planned grasp."""
        # Simplified quality estimation
        # In practice, this would use more sophisticated models
        return 0.8  # Placeholder quality score
```

## Dexterous Hand Control

### Multi-Fingered Hand Kinematics

Controlling a dexterous hand with multiple fingers:

```python
class DexterousHandController:
    def __init__(self, hand_model):
        self.hand_model = hand_model
        self.finger_names = hand_model.finger_names
        self.joint_names = hand_model.joint_names
        self.num_fingers = len(self.finger_names)
        self.num_joints = len(self.joint_names)

    def control_fingers(self, finger_positions, finger_velocities=None, finger_efforts=None):
        """
        Control the hand fingers to achieve desired positions.

        Args:
            finger_positions: Desired joint positions for each finger
            finger_velocities: Desired joint velocities (optional)
            finger_efforts: Desired joint efforts (optional)
        """
        # Create trajectory message for hand control
        trajectory_msg = self.create_hand_trajectory(
            finger_positions, finger_velocities, finger_efforts
        )
        # Publish to hand controller (implementation depends on hardware)
        pass

    def create_hand_trajectory(self, positions, velocities=None, efforts=None):
        """Create trajectory message for hand control."""
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = positions

        if velocities is not None:
            point.velocities = velocities
        else:
            point.velocities = [0.0] * len(positions)

        if efforts is not None:
            point.effort = efforts

        point.time_from_start.sec = 1
        point.time_from_start.nanosec = 0

        trajectory.points.append(point)
        return trajectory

    def execute_predefined_grasp(self, grasp_type):
        """Execute a predefined grasp pattern."""
        predefined_grasps = {
            'cylindrical': [0.5, 0.5, 0.5, 0.5, 0.5],
            'tripod': [0.3, 0.3, 0.3, 0.0, 0.7],
            'tip_pinch': [0.8, 0.0, 0.0, 0.0, 0.8],
            'lateral': [0.0, 0.7, 0.7, 0.7, 0.7],
            'sphere': [0.4, 0.4, 0.4, 0.4, 0.4]
        }

        if grasp_type in predefined_grasps:
            self.control_fingers(predefined_grasps[grasp_type])
            return True
        return False

    def adaptive_grasp(self, object_shape, object_size, object_weight):
        """
        Adaptively adjust grasp based on object properties.

        Args:
            object_shape: Shape descriptor (sphere, cylinder, box, etc.)
            object_size: Size of object (width, height, depth)
            object_weight: Weight of object
        """
        # Calculate appropriate grasp force based on object weight
        grasp_force = self.calculate_grasp_force(object_weight)

        # Adjust finger positions based on object size
        finger_positions = self.calculate_size_adaptive_positions(
            object_size, grasp_force
        )

        # Execute the adaptive grasp
        self.control_fingers(finger_positions)

    def calculate_grasp_force(self, object_weight, safety_factor=2.0):
        """Calculate required grasp force based on object weight."""
        # Grasp force should be at least safety_factor * object_weight
        # to ensure stable grasp with margin for acceleration
        required_force = object_weight * 9.81 * safety_factor
        return min(required_force, 100.0)  # Limit to maximum force

    def calculate_size_adaptive_positions(self, object_size, grasp_force):
        """Calculate finger positions adapted to object size."""
        # Calculate how wide to open fingers based on object size
        max_span = max(object_size[:2])  # Use width and height, not depth
        finger_spread = min(max_span / 2, 0.1)  # Max 10cm span

        # Convert to joint angles based on hand kinematics
        # This is a simplified mapping - real implementation would use inverse kinematics
        thumb_angle = self.size_to_joint_angle(finger_spread)
        other_fingers = [self.size_to_joint_angle(finger_spread * 0.8) for _ in range(4)]

        return [thumb_angle] + other_fingers

    def size_to_joint_angle(self, size_cm):
        """Convert object size to appropriate joint angle."""
        # Linear mapping from size to joint angle (simplified)
        # In reality, this would depend on the specific hand kinematics
        max_size_cm = 10.0  # Maximum graspable size
        max_angle = 1.57    # Maximum joint angle (90 degrees)

        normalized_size = min(size_cm / max_size_cm, 1.0)
        return max_angle * (1 - normalized_size)  # Inverse relationship
```

## Force Control and Tactile Feedback

### Impedance Control

Implementing compliant behavior for safe manipulation:

```python
class ImpedanceController:
    def __init__(self, mass=1.0, damping=10.0, stiffness=100.0):
        self.mass = mass
        self.damping = damping
        self.stiffness = stiffness
        self.position_error = 0.0
        self.velocity_error = 0.0
        self.previous_error = 0.0

    def compute_impedance_force(self, desired_pos, current_pos, desired_vel=0.0, current_vel=0.0):
        """
        Compute force based on impedance model.

        Args:
            desired_pos: Desired position
            current_pos: Current position
            desired_vel: Desired velocity
            current_vel: Current velocity

        Returns:
            Force to apply to achieve desired impedance behavior
        """
        self.position_error = desired_pos - current_pos
        self.velocity_error = desired_vel - current_vel

        # Impedance model: F = M*a + B*v + K*x
        acceleration = self.stiffness * self.position_error + self.damping * self.velocity_error
        force = self.mass * acceleration

        return force

    def adapt_impedance(self, contact_detected, environment_stiffness):
        """
        Adapt impedance parameters based on contact and environment.

        Args:
            contact_detected: Boolean indicating if contact is made
            environment_stiffness: Estimated stiffness of environment
        """
        if contact_detected:
            # Reduce stiffness for compliant contact
            self.stiffness = min(self.stiffness, environment_stiffness / 2)
        else:
            # Higher stiffness for free space movement
            self.stiffness = 100.0

class TactileFeedbackController:
    def __init__(self, hand_controller):
        self.hand_controller = hand_controller
        self.tactile_thresholds = {
            'slip': 0.8,
            'pressure': 50.0,
            'temperature': 40.0
        }

    def process_tactile_data(self, tactile_sensors):
        """
        Process tactile sensor data and adjust grasp accordingly.

        Args:
            tactile_sensors: Dictionary with tactile sensor readings
        """
        slip_detected = self.detect_slip(tactile_sensors)
        pressure_too_high = self.check_pressure(tactile_sensors)
        temperature_warning = self.check_temperature(tactile_sensors)

        adjustments = {}
        if slip_detected:
            adjustments['increase_force'] = True
        if pressure_too_high:
            adjustments['decrease_force'] = True
        if temperature_warning:
            adjustments['reduce_activity'] = True

        return adjustments

    def detect_slip(self, tactile_sensors):
        """Detect slip based on tactile sensor readings."""
        # Simplified slip detection
        # In practice, this would use more sophisticated algorithms
        for finger, sensors in tactile_sensors.items():
            if 'slip' in sensors:
                if sensors['slip'] > self.tactile_thresholds['slip']:
                    return True
        return False

    def check_pressure(self, tactile_sensors):
        """Check if pressure is too high."""
        for finger, sensors in tactile_sensors.items():
            if 'pressure' in sensors:
                if sensors['pressure'] > self.tactile_thresholds['pressure']:
                    return True
        return False

    def check_temperature(self, tactile_sensors):
        """Check for temperature warnings."""
        for finger, sensors in tactile_sensors.items():
            if 'temperature' in sensors:
                if sensors['temperature'] > self.tactile_thresholds['temperature']:
                    return True
        return False

    def adjust_grasp_based_on_feedback(self, tactile_data):
        """Adjust grasp based on tactile feedback."""
        adjustments = self.process_tactile_data(tactile_data)

        if 'increase_force' in adjustments:
            # Increase grasp force slightly
            current_config = self.hand_controller.get_current_configuration()
            new_config = [angle * 1.05 for angle in current_config]  # 5% increase
            self.hand_controller.control_fingers(new_config)

        if 'decrease_force' in adjustments:
            # Decrease grasp force
            current_config = self.hand_controller.get_current_configuration()
            new_config = [angle * 0.95 for angle in current_config]  # 5% decrease
            self.hand_controller.control_fingers(new_config)
```

## ROS 2 Manipulation Framework

### Manipulation Node Integration

Creating a ROS 2 node that integrates all manipulation components:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState, PointCloud2
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import String, Float64MultiArray
from trajectory_msgs.msg import JointTrajectory
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import cv2
from cv_bridge import CvBridge

class ManipulationNode(Node):
    def __init__(self):
        super().__init__('manipulation_node')

        # Initialize components
        self.grasp_planner = VisionBasedGraspPlanner()
        self.hand_controller = DexterousHandController(hand_model=None)  # Placeholder
        self.impedance_controller = ImpedanceController()
        self.tactile_controller = TactileFeedbackController(self.hand_controller)
        self.bridge = CvBridge()

        # Publishers
        self.grasp_pose_pub = self.create_publisher(Pose, '/grasp_pose', 10)
        self.hand_trajectory_pub = self.create_publisher(
            JointTrajectory, '/hand_controller/joint_trajectory', 10
        )
        self.visualization_pub = self.create_publisher(MarkerArray, '/manipulation_markers', 10)
        self.status_pub = self.create_publisher(String, '/manipulation_status', 10)

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/rgb/image_raw', self.image_callback, 10
        )
        self.pointcloud_sub = self.create_subscription(
            PointCloud2, '/camera/depth/points', self.pointcloud_callback, 10
        )
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        self.tactile_sub = self.create_subscription(
            Float64MultiArray, '/tactile_sensors', self.tactile_callback, 10
        )

        # Service servers
        self.grasp_service = self.create_service(
            String, '/execute_grasp', self.execute_grasp_callback
        )

        # Timers
        self.manipulation_timer = self.create_timer(0.1, self.manipulation_loop)

        # State variables
        self.latest_image = None
        self.latest_pointcloud = None
        self.joint_states = None
        self.tactile_data = None
        self.current_object = None
        self.grasp_in_progress = False

        self.get_logger().info('Manipulation Node initialized')

    def image_callback(self, msg):
        """Handle incoming camera images for object detection."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_image = cv_image
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')

    def pointcloud_callback(self, msg):
        """Handle incoming point cloud data."""
        # Process point cloud for 3D object information
        self.latest_pointcloud = msg

    def joint_state_callback(self, msg):
        """Handle joint state updates."""
        self.joint_states = msg

    def tactile_callback(self, msg):
        """Handle tactile sensor data."""
        # Convert Float64MultiArray to structured tactile data
        # This would depend on your specific tactile sensor setup
        self.tactile_data = msg.data

    def manipulation_loop(self):
        """Main manipulation control loop."""
        if self.latest_image is not None and not self.grasp_in_progress:
            # Detect objects in the scene
            objects = self.grasp_planner.detect_and_estimate_pose(self.latest_image)

            if objects:
                # Select the most suitable object for grasping
                target_object = self.select_target_object(objects)
                self.current_object = target_object

                # Plan a grasp for the target object
                grasp_plan = self.grasp_planner.plan_grasp(
                    target_object, self.hand_controller.hand_model
                )

                if grasp_plan and grasp_plan['quality_score'] > 0.5:
                    # Publish grasp pose for visualization
                    self.publish_grasp_pose(grasp_plan['grasp_pose'])

                    # Execute the grasp
                    self.execute_grasp(grasp_plan)

    def select_target_object(self, objects):
        """Select the most suitable object for grasping."""
        # For simplicity, select the closest object
        # In practice, you might consider object type, graspability, etc.
        if not objects:
            return None

        # Sort by distance (assuming z-axis is forward)
        objects.sort(key=lambda obj: obj['pose'][2])  # Sort by z (depth)
        return objects[0]

    def publish_grasp_pose(self, grasp_pose):
        """Publish grasp pose for visualization."""
        pose_msg = Pose()
        pose_msg.position.x = grasp_pose[0]
        pose_msg.position.y = grasp_pose[1]
        pose_msg.position.z = grasp_pose[2]
        pose_msg.orientation.x = grasp_pose[3]
        pose_msg.orientation.y = grasp_pose[4]
        pose_msg.orientation.z = grasp_pose[5]
        pose_msg.orientation.w = grasp_pose[6]

        self.grasp_pose_pub.publish(pose_msg)

        # Publish visualization markers
        marker_array = MarkerArray()

        # Grasp approach direction marker
        approach_marker = Marker()
        approach_marker.header.frame_id = "camera_link"  # Adjust to your frame
        approach_marker.header.stamp = self.get_clock().now().to_msg()
        approach_marker.id = 1
        approach_marker.type = Marker.ARROW
        approach_marker.action = Marker.ADD
        approach_marker.pose = pose_msg
        approach_marker.scale.x = 0.1  # Shaft diameter
        approach_marker.scale.y = 0.15  # Head diameter
        approach_marker.scale.z = 0.05  # Head length
        approach_marker.color.a = 1.0  # Alpha
        approach_marker.color.r = 1.0  # Red
        approach_marker.color.g = 0.0  # Green
        approach_marker.color.b = 0.0  # Blue

        marker_array.markers.append(approach_marker)
        self.visualization_pub.publish(marker_array)

    def execute_grasp(self, grasp_plan):
        """Execute the planned grasp."""
        self.grasp_in_progress = True

        # Move hand to pre-grasp position
        pre_grasp_pose = self.calculate_pre_grasp_pose(grasp_plan['grasp_pose'])
        self.move_to_pose(pre_grasp_pose)

        # Open hand to appropriate configuration
        self.hand_controller.execute_predefined_grasp(grasp_plan['grasp_type'])

        # Move to grasp pose
        self.move_to_pose(grasp_plan['grasp_pose'])

        # Close hand to grasp object
        self.close_hand_for_grasp(grasp_plan)

        # Lift object
        self.lift_object(grasp_plan['grasp_pose'])

        self.grasp_in_progress = False

        status_msg = String()
        status_msg.data = "Grasp completed successfully"
        self.status_pub.publish(status_msg)

    def calculate_pre_grasp_pose(self, grasp_pose):
        """Calculate pre-grasp pose (slightly above the grasp pose)."""
        pre_grasp = grasp_pose.copy()
        # Move 5cm above the grasp point
        pre_grasp[2] += 0.05
        return pre_grasp

    def move_to_pose(self, pose):
        """Move hand to specified pose."""
        # This would interface with the arm controller
        # For now, this is a placeholder
        pass

    def close_hand_for_grasp(self, grasp_plan):
        """Close hand with appropriate force for the grasp."""
        # Execute the grasp configuration
        self.hand_controller.control_fingers(grasp_plan['hand_configuration'])

        # Apply impedance control for compliant grasp
        self.impedance_controller.adapt_impedance(contact_detected=True, environment_stiffness=500.0)

    def lift_object(self, grasp_pose):
        """Lift the grasped object."""
        # Move upward from the grasp pose
        lift_pose = grasp_pose.copy()
        lift_pose[2] += 0.1  # Lift 10cm
        self.move_to_pose(lift_pose)

    def execute_grasp_callback(self, request, response):
        """Service callback to manually trigger a grasp."""
        # Parse request for object type or specific grasp
        object_type = request.data if request.data else "any"

        # Perform grasp based on request
        if self.current_object:
            grasp_plan = self.grasp_planner.plan_grasp(
                self.current_object, self.hand_controller.hand_model
            )
            if grasp_plan:
                self.execute_grasp(grasp_plan)
                response.data = "Grasp executed"
            else:
                response.data = "No valid grasp found"
        else:
            response.data = "No object detected"

        return response
```

## Advanced Manipulation Strategies

### Multi-Modal Grasp Planning

Combining vision, touch, and prior knowledge:

```python
class MultiModalGraspPlanner:
    def __init__(self):
        self.vision_grasp_planner = VisionBasedGraspPlanner()
        self.prior_knowledge_base = self.load_prior_knowledge()
        self.tactile_adaptation = TactileFeedbackController(None)

    def load_prior_knowledge(self):
        """Load prior knowledge about object grasping."""
        # This would load a database of successful grasps for different object types
        return {
            'mug': {'grasp_type': 'cylindrical', 'approach_angle': 90},
            'book': {'grasp_type': 'parallel', 'approach_angle': 0},
            'ball': {'grasp_type': 'sphere', 'approach_angle': 45}
        }

    def plan_grasp_multimodal(self, object_info, tactile_data=None):
        """
        Plan grasp using multiple modalities.

        Args:
            object_info: Information from vision system
            tactile_data: Tactile feedback (if available)

        Returns:
            Optimized grasp plan
        """
        # Start with vision-based grasp
        vision_grasp = self.vision_grasp_planner.plan_grasp(
            object_info, hand_model=None
        )

        # Refine with prior knowledge
        refined_grasp = self.refine_with_prior_knowledge(
            vision_grasp, object_info['class']
        )

        # Adapt based on tactile feedback if available
        if tactile_data:
            adapted_grasp = self.adapt_to_tactile_feedback(
                refined_grasp, tactile_data
            )
            return adapted_grasp

        return refined_grasp

    def refine_with_prior_knowledge(self, grasp_plan, object_class):
        """Refine grasp based on prior knowledge of object class."""
        if object_class in self.prior_knowledge_base:
            prior = self.prior_knowledge_base[object_class]

            # Adjust grasp type if different from vision plan
            if prior['grasp_type'] != grasp_plan['grasp_type']:
                grasp_plan['grasp_type'] = prior['grasp_type']
                # Recalculate hand configuration
                grasp_plan['hand_configuration'] = self.adjust_hand_configuration(
                    grasp_plan['grasp_pose'], prior['grasp_type']
                )

        return grasp_plan

    def adjust_hand_configuration(self, grasp_pose, grasp_type):
        """Adjust hand configuration for specific grasp type."""
        # Placeholder implementation
        return [0.5, 0.5, 0.5, 0.5, 0.5]

    def adapt_to_tactile_feedback(self, grasp_plan, tactile_data):
        """Adapt grasp based on tactile feedback."""
        # Analyze tactile data for slip, pressure, etc.
        feedback = self.tactile_adaptation.process_tactile_data(tactile_data)

        # Adjust grasp plan based on feedback
        if 'increase_force' in feedback:
            # Adjust hand configuration to apply more force
            grasp_plan['hand_configuration'] = [
                angle * 1.1 for angle in grasp_plan['hand_configuration']
            ]

        return grasp_plan
```

## Quality Assessment and Evaluation

### Grasp Success Metrics

Evaluating manipulation performance:

```python
class ManipulationEvaluator:
    def __init__(self):
        self.success_count = 0
        self.attempt_count = 0
        self.grasp_quality_history = []
        self.execution_times = []

    def evaluate_grasp_attempt(self, grasp_plan, object_properties, result):
        """
        Evaluate a grasp attempt.

        Args:
            grasp_plan: Planned grasp configuration
            object_properties: Properties of the object
            result: Result of the grasp attempt ('success', 'slip', 'drop', etc.)

        Returns:
            Evaluation metrics
        """
        metrics = {
            'success': result == 'success',
            'grasp_quality': grasp_plan.get('quality_score', 0),
            'object_weight': object_properties.get('weight', 0),
            'object_shape': object_properties.get('shape', 'unknown')
        }

        # Update statistics
        self.attempt_count += 1
        if result == 'success':
            self.success_count += 1

        self.grasp_quality_history.append(grasp_plan.get('quality_score', 0))

        # Calculate success rate
        success_rate = self.success_count / self.attempt_count if self.attempt_count > 0 else 0

        metrics['success_rate'] = success_rate
        metrics['average_quality'] = np.mean(self.grasp_quality_history)

        return metrics

    def get_performance_report(self):
        """Generate performance report."""
        if self.attempt_count == 0:
            return "No grasp attempts recorded."

        success_rate = self.success_count / self.attempt_count
        avg_quality = np.mean(self.grasp_quality_history) if self.grasp_quality_history else 0
        std_quality = np.std(self.grasp_quality_history) if self.grasp_quality_history else 0

        report = f"""
        Manipulation Performance Report:
        - Total attempts: {self.attempt_count}
        - Successful grasps: {self.success_count}
        - Success rate: {success_rate:.2%}
        - Average grasp quality: {avg_quality:.3f}
        - Grasp quality std dev: {std_quality:.3f}
        """

        return report
```

## Safety Considerations

### Safe Manipulation Practices

```python
class SafeManipulationController:
    def __init__(self, max_force=50.0, max_velocity=0.5):
        self.max_force = max_force
        self.max_velocity = max_velocity
        self.emergency_stop = False

    def check_safety_constraints(self, planned_force, planned_velocity):
        """Check if planned actions are within safety limits."""
        if abs(planned_force) > self.max_force:
            return False, f"Force limit exceeded: {planned_force} > {self.max_force}"

        if abs(planned_velocity) > self.max_velocity:
            return False, f"Velocity limit exceeded: {planned_velocity} > {self.max_velocity}"

        return True, "Safe"

    def emergency_stop_procedure(self):
        """Execute emergency stop."""
        self.emergency_stop = True
        # Send zero commands to all joints
        # Log the emergency stop event
        print("EMERGENCY STOP: Manipulation system halted for safety")
```

## Best Practices

### Design Principles

1. **Gradual Complexity**: Start with simple grasps and increase complexity
2. **Robust Perception**: Ensure reliable object detection and pose estimation
3. **Compliant Control**: Use impedance control for safe interaction
4. **Feedback Integration**: Incorporate tactile and force feedback
5. **Safety First**: Implement comprehensive safety checks

### Implementation Guidelines

1. **Calibration**: Properly calibrate hand and camera systems
2. **Testing**: Extensively test with various object types and sizes
3. **Documentation**: Document grasp configurations and parameters
4. **Modularity**: Design components as independent modules
5. **Monitoring**: Continuously monitor grasp quality and success

## Key Takeaways

- Grasp planning combines vision, kinematics, and object properties
- Dexterous manipulation requires precise hand control and tactile feedback
- Force control and impedance control enable safe interaction
- Multi-modal approaches improve grasp success rates
- Safety considerations are paramount in manipulation systems

## Practice Exercises

### Exercise 1: Grasp Planning Algorithm Implementation
1. Implement a grasp quality evaluation function for different grasp types (cylindrical, tripod, pinch).
2. Test the algorithm with various object shapes and sizes.
3. Compare the performance of different grasp quality metrics.
4. Analyze the relationship between grasp quality scores and actual grasp success rates.

### Exercise 2: Vision-Based Object Detection and Grasping
1. Integrate a pre-trained object detection model with a grasp planning system.
2. Test the system with various everyday objects.
3. Evaluate the accuracy of pose estimation for grasping.
4. Implement error handling for failed object detections.

### Exercise 3: Dexterous Hand Control
1. Implement a controller for a multi-fingered robotic hand.
2. Create predefined grasp configurations for different object types.
3. Test the hand controller with simulated or real hardware.
4. Evaluate the precision and repeatability of the grasping motions.

### Exercise 4: Force Control and Compliance
1. Implement an impedance control system for safe manipulation.
2. Test the compliance behavior with different stiffness and damping parameters.
3. Integrate tactile feedback for slip detection and prevention.
4. Evaluate the system's ability to handle unexpected contacts and disturbances.

### Discussion Questions
1. What are the main challenges in designing a universal grasping system that works for all object types?
2. How does the number of degrees of freedom in a robotic hand affect grasping performance?
3. What are the trade-offs between precision grasping and power grasping in humanoid robots?
4. How can tactile sensing improve the robustness of robotic manipulation?

### Challenge Exercise
Design and implement a complete manipulation system for a humanoid robot:
- Integrate vision-based object detection and pose estimation
- Implement grasp planning with quality assessment
- Create a compliant hand control system with tactile feedback
- Develop a complete pick-and-place pipeline
- Test the system with various objects of different shapes, sizes, and materials
- Evaluate the system's performance in terms of success rate, speed, and safety
- Document the system architecture, parameters, and experimental results

## References

[Manipulation Systems Bibliography](/docs/references/manipulation-bibliography.md)