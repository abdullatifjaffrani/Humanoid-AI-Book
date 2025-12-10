#!/usr/bin/env python3
"""
Advanced VLA (Vision-Language-Action) Integration Demo

This comprehensive example demonstrates the integration of Vision, Language, and Action
systems for a humanoid robot, showcasing the concepts covered in Modules 1-4 of the
Physical AI & Humanoid Robotics Textbook.

The demo includes:
- Vision processing with deep learning models
- Natural language understanding
- Action planning and execution
- Sensor fusion and multimodal integration
- Real-time control and safety systems
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, PointCloud2, CameraInfo, LaserScan, Imu
from geometry_msgs.msg import Pose, PoseStamped, Twist, Vector3
from std_msgs.msg import String, Bool, Float64MultiArray
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from vision_msgs.msg import Detection2DArray, Detection2D
from tf2_ros import TransformListener, Buffer
import numpy as np
import cv2
from cv_bridge import CvBridge
import torch
import torch.nn as nn
from transformers import AutoTokenizer, AutoModelForSequenceClassification
import threading
import time
import json
from dataclasses import dataclass
from typing import List, Dict, Tuple, Optional
import queue


@dataclass
class RobotState:
    """Data class to represent the robot's state."""
    position: np.ndarray
    orientation: np.ndarray
    joint_positions: List[float]
    joint_velocities: List[float]
    battery_level: float
    system_health: bool


@dataclass
class ObjectInfo:
    """Data class to represent detected objects."""
    id: str
    class_name: str
    position: np.ndarray
    confidence: float
    bounding_box: Tuple[int, int, int, int]  # x, y, width, height


class AdvancedVisionProcessor(Node):
    """
    Advanced vision processing system that demonstrates concepts from Module 4:
    Vision-Language-Action Systems.
    """
    def __init__(self):
        super().__init__('advanced_vision_processor')

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Vision processing parameters
        self.confidence_threshold = 0.5
        self.nms_threshold = 0.4
        self.object_classes = [
            'person', 'bottle', 'cup', 'chair', 'table',
            'monitor', 'laptop', 'phone', 'book', 'box'
        ]

        # Publishers and subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/rgb/image_raw', self.image_callback, 10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/rgb/camera_info', self.camera_info_callback, 10
        )

        self.detection_pub = self.create_publisher(
            Detection2DArray, '/vision/detections', 10
        )

        self.object_pose_pub = self.create_publisher(
            PoseStamped, '/vision/object_pose', 10
        )

        # Internal state
        self.camera_info = None
        self.latest_image = None
        self.image_queue = queue.Queue(maxsize=2)  # Prevent backlog

        # Initialize deep learning model (using a simple CNN as placeholder)
        self.vision_model = self.initialize_vision_model()

        self.get_logger().info('Advanced Vision Processor initialized')

    def initialize_vision_model(self):
        """
        Initialize a vision model for object detection and recognition.
        In a real implementation, this would load a pre-trained model like YOLOv8,
        Detectron2, or a custom model.
        """
        # Placeholder for a real vision model
        # This would typically be a PyTorch or TensorFlow model
        class SimpleVisionModel(nn.Module):
            def __init__(self):
                super().__init__()
                # This is a simplified placeholder - real models would be much more complex
                self.feature_extractor = nn.Sequential(
                    nn.Conv2d(3, 32, 3, padding=1),
                    nn.ReLU(),
                    nn.MaxPool2d(2),
                    nn.Conv2d(32, 64, 3, padding=1),
                    nn.ReLU(),
                    nn.MaxPool2d(2),
                )
                self.classifier = nn.Linear(64, len(self.object_classes))

            def forward(self, x):
                features = self.feature_extractor(x)
                # Simplified classification
                return torch.randn(1, len(self.object_classes))  # Placeholder

        return SimpleVisionModel()

    def camera_info_callback(self, msg):
        """Store camera calibration information."""
        self.camera_info = msg

    def image_callback(self, msg):
        """Process incoming images and detect objects."""
        try:
            # Only process the latest image to avoid backlog
            if self.image_queue.full():
                self.image_queue.get()  # Remove oldest image

            self.image_queue.put(msg)

            # Process images in a separate thread to avoid blocking
            threading.Thread(target=self.process_image_thread, daemon=True).start()

        except Exception as e:
            self.get_logger().error(f'Error in image callback: {str(e)}')

    def process_image_thread(self):
        """Process images in a separate thread."""
        try:
            if not self.image_queue.empty():
                msg = self.image_queue.get()
                cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

                # Perform object detection
                detections = self.detect_objects(cv_image)

                # Publish detections
                self.publish_detections(detections, msg.header)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def detect_objects(self, image):
        """Detect objects in the image using the vision model."""
        # Convert image to tensor format expected by the model
        # This is a simplified version - real implementation would use actual model
        height, width = image.shape[:2]

        # Simulate detection results (in real implementation, this would use the model)
        detections = []

        # Simulate detecting some objects
        for i in range(np.random.randint(1, 4)):  # 1-3 random objects
            x = np.random.randint(0, width - 100)
            y = np.random.randint(0, height - 100)
            w = np.random.randint(50, 150)
            h = np.random.randint(50, 150)

            class_idx = np.random.randint(0, len(self.object_classes))
            class_name = self.object_classes[class_idx]
            confidence = np.random.uniform(0.6, 0.95)

            detection = ObjectInfo(
                id=f"obj_{i}",
                class_name=class_name,
                position=np.array([x + w/2, y + h/2, 0.0]),  # 2D position, Z unknown
                confidence=confidence,
                bounding_box=(x, y, w, h)
            )

            detections.append(detection)

        return detections

    def publish_detections(self, detections, header):
        """Publish detection results as ROS 2 messages."""
        detection_array = Detection2DArray()
        detection_array.header = header

        for detection in detections:
            if detection.confidence > self.confidence_threshold:
                detection_msg = Detection2D()
                detection_msg.header = header

                # Set bounding box center and size
                detection_msg.bbox.center.x = detection.position[0]
                detection_msg.bbox.center.y = detection.position[1]
                detection_msg.bbox.size_x = detection.bounding_box[2]
                detection_msg.bbox.size_y = detection.bounding_box[3]

                # Add classification result
                from vision_msgs.msg import ObjectHypothesisWithPose
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = detection.class_name
                hypothesis.hypothesis.score = detection.confidence
                detection_msg.results.append(hypothesis)

                detection_array.detections.append(detection_msg)

        self.detection_pub.publish(detection_array)


class NaturalLanguageUnderstandingNode(Node):
    """
    Natural Language Understanding system that processes human commands
    and converts them to robot actions, demonstrating concepts from Module 4.
    """
    def __init__(self):
        super().__init__('natural_language_understanding')

        # Publishers and subscribers
        self.command_sub = self.create_subscription(
            String, '/voice_commands', self.command_callback, 10
        )

        self.action_pub = self.create_publisher(
            String, '/parsed_actions', 10
        )

        self.status_pub = self.create_publisher(
            String, '/nlu_status', 10
        )

        # Initialize NLU model (using transformers as placeholder)
        self.tokenizer = None  # Would initialize with actual model
        self.nlu_model = self.initialize_nlu_model()

        # Command mapping
        self.command_keywords = {
            'navigation': ['go to', 'move to', 'navigate to', 'walk to', 'go', 'move'],
            'manipulation': ['pick up', 'grasp', 'take', 'grab', 'lift', 'place', 'put'],
            'interaction': ['greet', 'hello', 'hi', 'wave', 'follow', 'stop', 'wait'],
            'query': ['where', 'what', 'how', 'find', 'locate', 'search']
        }

        self.get_logger().info('Natural Language Understanding Node initialized')

    def initialize_nlu_model(self):
        """
        Initialize Natural Language Understanding model.
        In a real implementation, this would load a pre-trained transformer model.
        """
        # Placeholder for a real NLU model
        # This would typically be a BERT, GPT, or similar model fine-tuned for robotics
        class SimpleNLUModel(nn.Module):
            def __init__(self):
                super().__init__()
                # Simplified model structure
                self.embedding = nn.Embedding(10000, 128)  # Vocabulary size, embedding dim
                self.lstm = nn.LSTM(128, 64, batch_first=True)
                self.classifier = nn.Linear(64, 10)  # 10 action classes

            def forward(self, x):
                # Simplified forward pass
                return torch.randn(1, 10)  # Placeholder output

        return SimpleNLUModel()

    def command_callback(self, msg):
        """Process natural language commands."""
        try:
            command_text = msg.data.lower().strip()
            self.get_logger().info(f'Received command: {command_text}')

            # Parse the command
            parsed_action = self.parse_command(command_text)

            if parsed_action:
                # Publish parsed action
                action_msg = String()
                action_msg.data = json.dumps(parsed_action)
                self.action_pub.publish(action_msg)

                status_msg = String()
                status_msg.data = f'Command parsed: {parsed_action["action_type"]}'
                self.status_pub.publish(status_msg)

                self.get_logger().info(f'Parsed action: {parsed_action}')
            else:
                self.get_logger().warn(f'Could not parse command: {command_text}')

        except Exception as e:
            self.get_logger().error(f'Error processing command: {str(e)}')

    def parse_command(self, command_text: str) -> Optional[Dict]:
        """
        Parse natural language command into structured action.

        Args:
            command_text: Natural language command

        Returns:
            Parsed action as dictionary, or None if parsing fails
        """
        # Convert command to lowercase for processing
        command_lower = command_text.lower()

        # Identify action type based on keywords
        action_type = None
        target_object = None
        location = None

        # Check for navigation commands
        for keyword in self.command_keywords['navigation']:
            if keyword in command_lower:
                action_type = 'navigation'
                # Extract location if mentioned
                location = self.extract_location(command_lower, keyword)
                break

        # Check for manipulation commands
        if action_type is None:
            for keyword in self.command_keywords['manipulation']:
                if keyword in command_lower:
                    action_type = 'manipulation'
                    # Extract target object if mentioned
                    target_object = self.extract_object(command_lower, keyword)
                    break

        # Check for interaction commands
        if action_type is None:
            for keyword in self.command_keywords['interaction']:
                if keyword in command_lower:
                    action_type = 'interaction'
                    break

        # Check for query commands
        if action_type is None:
            for keyword in self.command_keywords['query']:
                if keyword in command_lower:
                    action_type = 'query'
                    break

        if action_type:
            return {
                'action_type': action_type,
                'command': command_text,
                'target_object': target_object,
                'location': location,
                'timestamp': time.time()
            }

        return None

    def extract_location(self, command: str, keyword: str) -> Optional[str]:
        """Extract location from navigation command."""
        # Simple location extraction (in real implementation, would use NER)
        # This is a basic example - real implementation would be more sophisticated
        location_keywords = ['kitchen', 'living room', 'bedroom', 'office', 'table', 'shelf', 'door']

        for loc in location_keywords:
            if loc in command:
                return loc

        return None

    def extract_object(self, command: str, keyword: str) -> Optional[str]:
        """Extract target object from manipulation command."""
        # Simple object extraction
        object_keywords = ['bottle', 'cup', 'book', 'phone', 'box', 'apple', 'toy']

        for obj in object_keywords:
            if obj in command:
                return obj

        return None


class ActionPlanningNode(Node):
    """
    Action planning system that creates executable plans from high-level commands,
    demonstrating concepts from Module 3: NVIDIA Isaac Platform and Module 4: VLA Systems.
    """
    def __init__(self):
        super().__init__('action_planning_node')

        # Publishers and subscribers
        self.action_sub = self.create_subscription(
            String, '/parsed_actions', self.action_callback, 10
        )

        self.plan_pub = self.create_publisher(
            String, '/action_plan', 10
        )

        self.vision_sub = self.create_subscription(
            Detection2DArray, '/vision/detections', self.vision_callback, 10
        )

        # Internal state
        self.detected_objects = []
        self.current_robot_state = RobotState(
            position=np.array([0.0, 0.0, 0.0]),
            orientation=np.array([0.0, 0.0, 0.0, 1.0]),  # quaternion
            joint_positions=[0.0] * 12,  # Example: 12 joints
            joint_velocities=[0.0] * 12,
            battery_level=100.0,
            system_health=True
        )

        self.get_logger().info('Action Planning Node initialized')

    def vision_callback(self, msg):
        """Update detected objects from vision system."""
        self.detected_objects = []
        for detection in msg.detections:
            if detection.results:
                best_result = max(detection.results, key=lambda x: x.hypothesis.score)
                obj_info = ObjectInfo(
                    id=f"obj_{len(self.detected_objects)}",
                    class_name=best_result.hypothesis.class_id,
                    position=np.array([detection.bbox.center.x, detection.bbox.center.y, 0.0]),
                    confidence=best_result.hypothesis.score,
                    bounding_box=(
                        int(detection.bbox.center.x - detection.bbox.size_x/2),
                        int(detection.bbox.center.y - detection.bbox.size_y/2),
                        int(detection.bbox.size_x),
                        int(detection.bbox.size_y)
                    )
                )
                self.detected_objects.append(obj_info)

    def action_callback(self, msg):
        """Process parsed actions and create execution plans."""
        try:
            action_data = json.loads(msg.data)
            plan = self.create_plan_for_action(action_data)

            if plan:
                plan_msg = String()
                plan_msg.data = json.dumps(plan)
                self.plan_pub.publish(plan_msg)

                self.get_logger().info(f'Created plan: {plan["plan_type"]}')
        except Exception as e:
            self.get_logger().error(f'Error creating plan: {str(e)}')

    def create_plan_for_action(self, action_data: Dict) -> Optional[Dict]:
        """
        Create an executable plan based on the parsed action.

        Args:
            action_data: Parsed action from NLU system

        Returns:
            Execution plan as dictionary, or None if planning fails
        """
        action_type = action_data['action_type']

        if action_type == 'navigation':
            return self.create_navigation_plan(action_data)
        elif action_type == 'manipulation':
            return self.create_manipulation_plan(action_data)
        elif action_type == 'interaction':
            return self.create_interaction_plan(action_data)
        elif action_type == 'query':
            return self.create_query_plan(action_data)

        return None

    def create_navigation_plan(self, action_data: Dict) -> Dict:
        """Create navigation plan."""
        # Find target location in the environment
        target_location = action_data.get('location', 'default')

        # This would interface with navigation stack to plan path
        # For demo, we'll create a simple plan
        plan = {
            'plan_type': 'navigation',
            'target_location': target_location,
            'waypoints': self.calculate_navigation_path(target_location),
            'execution_steps': [
                {'step': 'localize', 'description': 'Localize robot in environment'},
                {'step': 'plan_path', 'description': f'Plan path to {target_location}'},
                {'step': 'execute_navigation', 'description': 'Execute navigation'}
            ],
            'safety_checks': ['obstacle_avoidance', 'kinect_check', 'battery_level'],
            'timeout': 60.0  # seconds
        }

        return plan

    def calculate_navigation_path(self, target_location: str) -> List[np.ndarray]:
        """Calculate navigation path to target location."""
        # This would use a path planning algorithm (A*, RRT, etc.)
        # For demo, return a simple path
        current_pos = self.current_robot_state.position
        target_pos = self.get_location_coordinates(target_location)

        # Simple straight-line path (in real implementation, would use proper path planning)
        path = [current_pos, target_pos]
        return path

    def get_location_coordinates(self, location_name: str) -> np.ndarray:
        """Get coordinates for a named location."""
        # This would interface with a map of known locations
        location_map = {
            'kitchen': np.array([2.0, 1.0, 0.0]),
            'living room': np.array([0.0, 0.0, 0.0]),
            'bedroom': np.array([3.0, -1.0, 0.0]),
            'office': np.array([-1.0, 2.0, 0.0]),
            'default': np.array([0.0, 0.0, 0.0])
        }

        return location_map.get(location_name, np.array([0.0, 0.0, 0.0]))

    def create_manipulation_plan(self, action_data: Dict) -> Dict:
        """Create manipulation plan."""
        target_object = action_data.get('target_object', 'object')

        # Find the object in detected objects
        target_obj_info = self.find_object_by_name(target_object)

        if not target_obj_info:
            self.get_logger().warn(f'Object {target_object} not found in environment')
            return {
                'plan_type': 'search',
                'target_object': target_object,
                'execution_steps': [
                    {'step': 'search', 'description': f'Search for {target_object}'}
                ]
            }

        plan = {
            'plan_type': 'manipulation',
            'target_object': target_object,
            'object_info': {
                'position': target_obj_info.position.tolist(),
                'class': target_obj_info.class_name,
                'confidence': target_obj_info.confidence
            },
            'execution_steps': [
                {'step': 'approach', 'description': f'Approach {target_object}'},
                {'step': 'grasp_planning', 'description': 'Plan grasp for object'},
                {'step': 'execute_grasp', 'description': 'Execute grasp'},
                {'step': 'lift', 'description': 'Lift object'}
            ],
            'safety_checks': ['joint_limits', 'force_feedback', 'collision_check'],
            'timeout': 30.0
        }

        return plan

    def find_object_by_name(self, name: str) -> Optional[ObjectInfo]:
        """Find object by name in detected objects."""
        for obj in self.detected_objects:
            if name.lower() in obj.class_name.lower():
                return obj
        return None

    def create_interaction_plan(self, action_data: Dict) -> Dict:
        """Create interaction plan."""
        plan = {
            'plan_type': 'interaction',
            'interaction_type': action_data['command'],
            'execution_steps': [
                {'step': 'detect_human', 'description': 'Detect human in environment'},
                {'step': 'orient_towards_human', 'description': 'Orient towards human'},
                {'step': 'execute_interaction', 'description': 'Execute interaction behavior'}
            ],
            'timeout': 10.0
        }

        return plan

    def create_query_plan(self, action_data: Dict) -> Dict:
        """Create query plan."""
        plan = {
            'plan_type': 'query',
            'query_type': action_data['command'],
            'execution_steps': [
                {'step': 'analyze_environment', 'description': 'Analyze current environment'},
                {'step': 'search_objects', 'description': 'Search for requested objects'},
                {'step': 'report_findings', 'description': 'Report findings to user'}
            ],
            'timeout': 15.0
        }

        return plan


class ControlExecutionNode(Node):
    """
    Control execution system that executes action plans on the robot hardware,
    demonstrating concepts from Module 1: ROS 2 Foundations and Module 3: NVIDIA Isaac Platform.
    """
    def __init__(self):
        super().__init__('control_execution_node')

        # Publishers and subscribers
        self.plan_sub = self.create_subscription(
            String, '/action_plan', self.plan_callback, 10
        )

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist, '/cmd_vel', 10
        )

        self.joint_trajectory_pub = self.create_publisher(
            JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10
        )

        self.status_pub = self.create_publisher(
            String, '/execution_status', 10
        )

        # Internal state
        self.current_pose = None
        self.current_twist = None
        self.imu_data = None
        self.is_executing = False
        self.execution_thread = None

        self.get_logger().info('Control Execution Node initialized')

    def odom_callback(self, msg):
        """Update robot pose from odometry."""
        self.current_pose = msg.pose.pose
        self.current_twist = msg.twist.twist

    def imu_callback(self, msg):
        """Update IMU data."""
        self.imu_data = msg

    def plan_callback(self, msg):
        """Execute action plan."""
        if self.is_executing:
            self.get_logger().warn('Already executing a plan, skipping new plan')
            return

        try:
            plan_data = json.loads(msg.data)
            self.is_executing = True

            # Execute plan in separate thread to avoid blocking
            self.execution_thread = threading.Thread(
                target=self.execute_plan, args=(plan_data,), daemon=True
            )
            self.execution_thread.start()

        except Exception as e:
            self.get_logger().error(f'Error executing plan: {str(e)}')
            self.is_executing = False

    def execute_plan(self, plan_data: Dict):
        """Execute the action plan."""
        try:
            plan_type = plan_data['plan_type']

            status_msg = String()
            status_msg.data = f'Executing {plan_type} plan'
            self.status_pub.publish(status_msg)

            if plan_type == 'navigation':
                self.execute_navigation_plan(plan_data)
            elif plan_type == 'manipulation':
                self.execute_manipulation_plan(plan_data)
            elif plan_type == 'interaction':
                self.execute_interaction_plan(plan_data)
            elif plan_type == 'query':
                self.execute_query_plan(plan_data)
            elif plan_type == 'search':
                self.execute_search_plan(plan_data)

            # Plan completed
            status_msg.data = f'Completed {plan_type} plan'
            self.status_pub.publish(status_msg)

        except Exception as e:
            self.get_logger().error(f'Error during plan execution: {str(e)}')
            status_msg = String()
            status_msg.data = f'Plan execution failed: {str(e)}'
            self.status_pub.publish(status_msg)

        finally:
            self.is_executing = False

    def execute_navigation_plan(self, plan_data: Dict):
        """Execute navigation plan."""
        waypoints = plan_data.get('waypoints', [])

        for i, waypoint in enumerate(waypoints):
            self.get_logger().info(f'Navigating to waypoint {i+1}/{len(waypoints)}')

            # Simple navigation to waypoint (in real implementation, would use Navigation2)
            success = self.navigate_to_pose(waypoint)

            if not success:
                self.get_logger().error(f'Failed to reach waypoint {i+1}')
                break

            # Check for timeout
            if self.check_timeout(plan_data.get('timeout', 60.0)):
                break

    def navigate_to_pose(self, target_pose: np.ndarray) -> bool:
        """Navigate to target pose."""
        # Calculate direction to target
        if self.current_pose:
            current_pos = np.array([
                self.current_pose.position.x,
                self.current_pose.position.y,
                self.current_pose.position.z
            ])

            direction = target_pose - current_pos
            distance = np.linalg.norm(direction[:2])  # 2D distance

            if distance > 0.1:  # If more than 10cm away
                # Normalize direction
                direction = direction / distance
                direction = direction[:2]  # Use 2D direction

                # Create velocity command
                cmd_vel = Twist()
                cmd_vel.linear.x = min(0.2, distance * 0.5)  # Scale with distance
                cmd_vel.linear.y = 0.0
                cmd_vel.angular.z = np.arctan2(direction[1], direction[0]) * 0.5

                # Publish command
                self.cmd_vel_pub.publish(cmd_vel)

                # Wait briefly
                time.sleep(0.1)
                return True

        return False

    def execute_manipulation_plan(self, plan_data: Dict):
        """Execute manipulation plan."""
        target_object = plan_data.get('target_object', 'object')
        object_info = plan_data.get('object_info', {})

        self.get_logger().info(f'Executing manipulation for {target_object}')

        # Approach object
        if 'position' in object_info:
            object_pos = np.array(object_info['position'])
            self.approach_object(object_pos)

        # Execute grasp (simplified)
        self.execute_grasp()

        # Lift object
        self.lift_object()

    def approach_object(self, object_pos: np.ndarray):
        """Approach the target object."""
        # Navigate close to the object
        approach_pos = object_pos.copy()
        approach_pos[0] -= 0.3  # Stay 30cm away for safety

        # Use navigation function
        self.navigate_to_pose(approach_pos)

    def execute_grasp(self):
        """Execute grasp action."""
        # This would send commands to the manipulator
        # For demo, just simulate the action
        self.get_logger().info('Executing grasp action')

        # Create joint trajectory for grasping
        trajectory = JointTrajectory()
        trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'gripper']  # Example names

        point = JointTrajectoryPoint()
        point.positions = [0.5, 0.3, -0.2, 0.8]  # Example positions
        point.velocities = [0.0, 0.0, 0.0, 0.0]
        point.time_from_start.sec = 2
        point.time_from_start.nanosec = 0

        trajectory.points.append(point)

        self.joint_trajectory_pub.publish(trajectory)

        time.sleep(2)  # Simulate execution time

    def lift_object(self):
        """Lift the grasped object."""
        self.get_logger().info('Lifting object')
        # Simulate lifting action
        time.sleep(1)

    def execute_interaction_plan(self, plan_data: Dict):
        """Execute interaction plan."""
        self.get_logger().info('Executing interaction plan')
        # Simulate interaction behavior
        time.sleep(2)

    def execute_query_plan(self, plan_data: Dict):
        """Execute query plan."""
        self.get_logger().info('Executing query plan')
        # Simulate environment analysis
        time.sleep(3)

    def execute_search_plan(self, plan_data: Dict):
        """Execute search plan."""
        target_object = plan_data.get('target_object', 'object')
        self.get_logger().info(f'Searching for {target_object}')

        # Simulate search behavior
        search_positions = [
            np.array([1.0, 0.0, 0.0]),
            np.array([0.0, 1.0, 0.0]),
            np.array([-1.0, 0.0, 0.0]),
            np.array([0.0, -1.0, 0.0])
        ]

        for pos in search_positions:
            self.navigate_to_pose(pos)
            time.sleep(1)  # Simulate looking around

    def check_timeout(self, timeout: float) -> bool:
        """Check if execution has timed out."""
        # This would track execution time and compare to timeout
        return False  # Simplified - no actual timeout check


class SafetyMonitorNode(Node):
    """
    Safety monitoring system that ensures safe operation of the robot,
    demonstrating safety concepts throughout the textbook.
    """
    def __init__(self):
        super().__init__('safety_monitor_node')

        # Publishers and subscribers
        self.emergency_stop_pub = self.create_publisher(
            Bool, '/emergency_stop', 10
        )

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )

        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10
        )

        self.joint_state_sub = self.create_subscription(
            Float64MultiArray, '/joint_states', self.joint_state_callback, 10
        )

        # Internal state
        self.scan_data = None
        self.imu_data = None
        self.joint_states = None
        self.emergency_stop_activated = False

        # Safety parameters
        self.min_obstacle_distance = 0.3  # meters
        self.max_lean_angle = 0.3  # radians
        self.max_joint_velocity = 2.0  # rad/s

        # Start safety monitoring
        self.safety_timer = self.create_timer(0.1, self.safety_check)  # 10 Hz

        self.get_logger().info('Safety Monitor Node initialized')

    def scan_callback(self, msg):
        """Update laser scan data."""
        self.scan_data = msg

    def imu_callback(self, msg):
        """Update IMU data for balance monitoring."""
        self.imu_data = msg

    def joint_state_callback(self, msg):
        """Update joint state data for safety checks."""
        self.joint_states = msg

    def safety_check(self):
        """Perform periodic safety checks."""
        if self.emergency_stop_activated:
            return

        # Check for obstacles
        if self.scan_data and self.check_for_obstacles():
            self.trigger_emergency_stop("Obstacle detected too close")
            return

        # Check balance (from IMU)
        if self.imu_data and self.check_balance():
            self.trigger_emergency_stop("Balance compromised")
            return

        # Check joint limits
        if self.joint_states and self.check_joint_limits():
            self.trigger_emergency_stop("Joint limit exceeded")
            return

    def check_for_obstacles(self) -> bool:
        """Check if there are obstacles too close."""
        if not self.scan_data:
            return False

        # Check distances in the front sector
        front_distances = self.scan_data.ranges[
            len(self.scan_data.ranges)//2 - 10 : len(self.scan_data.ranges)//2 + 10
        ]

        min_distance = min((d for d in front_distances if 0 < d < float('inf')), default=float('inf'))

        return min_distance < self.min_obstacle_distance

    def check_balance(self) -> bool:
        """Check if robot is tilting too much."""
        if not self.imu_data:
            return False

        # Extract orientation from IMU
        orientation = self.imu_data.orientation
        # Convert quaternion to roll/pitch (simplified)
        # In real implementation, would properly convert quaternion to euler angles

        # Check if absolute values exceed limits (simplified)
        return (abs(orientation.x) > self.max_lean_angle or
                abs(orientation.y) > self.max_lean_angle)

    def check_joint_limits(self) -> bool:
        """Check if joint velocities exceed limits."""
        if not self.joint_states or not self.joint_states.data:
            return False

        # This would check actual joint velocities against limits
        # For demo, return False (no limit exceeded)
        return False

    def trigger_emergency_stop(self, reason: str):
        """Trigger emergency stop."""
        self.emergency_stop_activated = True

        # Publish emergency stop command
        stop_msg = Bool()
        stop_msg.data = True
        self.emergency_stop_pub.publish(stop_msg)

        self.get_logger().fatal(f'EMERGENCY STOP: {reason}')


def main(args=None):
    """
    Main function to run the complete VLA integration demo.

    This demonstrates the integration of all major components:
    - Vision processing
    - Natural language understanding
    - Action planning
    - Control execution
    - Safety monitoring
    """
    rclpy.init(args=args)

    # Create nodes for each component
    vision_node = AdvancedVisionProcessor()
    nlu_node = NaturalLanguageUnderstandingNode()
    planning_node = ActionPlanningNode()
    control_node = ControlExecutionNode()
    safety_node = SafetyMonitorNode()

    # Create multi-threaded executor to run all nodes
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(vision_node)
    executor.add_node(nlu_node)
    executor.add_node(planning_node)
    executor.add_node(control_node)
    executor.add_node(safety_node)

    print("VLA Integration Demo started!")
    print("Nodes running:")
    print(f"- Vision Processor: {vision_node.get_name()}")
    print(f"- NLU System: {nlu_node.get_name()}")
    print(f"- Action Planner: {planning_node.get_name()}")
    print(f"- Control Executor: {control_node.get_name()}")
    print(f"- Safety Monitor: {safety_node.get_name()}")
    print("\nSystem is ready to process commands and execute actions.")
    print("Send commands to /voice_commands topic to test the system.")

    try:
        executor.spin()
    except KeyboardInterrupt:
        print("\nShutting down VLA Integration Demo...")
    finally:
        # Cleanup
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()