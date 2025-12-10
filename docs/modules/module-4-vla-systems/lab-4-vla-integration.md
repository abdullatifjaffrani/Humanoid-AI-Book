---
title: Lab 4 - VLA Integration Exercise
sidebar_position: 3
week: 9
module: module-4-vla-systems
learningObjectives:
  - Implement a complete Vision-Language-Action system
  - Integrate computer vision with natural language processing
  - Create a multimodal robot interface
  - Test VLA system in simulated environment
  - Evaluate performance of integrated system
prerequisites:
  - Week 1-9 content: Complete textbook modules including vision and language
  - Understanding of ROS 2 message passing
  - Python programming experience
  - Basic knowledge of deep learning frameworks
description: Hands-on lab to implement and test a complete Vision-Language-Action system
---

# Lab 4: VLA Integration Exercise

## Learning Objectives

After completing this lab, you will be able to:
- Implement a complete Vision-Language-Action system for robotics
- Integrate computer vision and natural language processing components
- Create a multimodal interface for robot interaction
- Test the integrated system in simulation
- Evaluate the performance of the VLA system

## Lab Duration

Estimated time: 180 minutes

## Prerequisites

- Complete Week 1-9 content (all textbook modules)
- ROS 2 Humble Hawksbill installed
- Gazebo simulation environment
- Basic understanding of deep learning frameworks
- Python programming experience

## Scenario: Multimodal Robot Assistant

You are developing a multimodal robot assistant that can understand natural language commands, perceive its environment through vision, and execute appropriate actions. In this lab, you'll create a complete Vision-Language-Action system that can process commands like "Find the red cup and bring it to the kitchen."

## Step 1: Create VLA Workspace

First, create a workspace for the VLA system:

```bash
mkdir -p ~/vla_robot_ws/src
cd ~/vla_robot_ws
source /opt/ros/humble/setup.bash
```

## Step 2: Create VLA Package

Create a package for the VLA system:

```bash
cd ~/vla_robot_ws/src
ros2 pkg create --build-type ament_python vla_robot_system --dependencies rclpy std_msgs geometry_msgs sensor_msgs vision_msgs cv_bridge openai transformers torch
```

## Step 3: Create Vision Processing Component

Create the vision processing directory:

```bash
mkdir -p ~/vla_robot_ws/src/vla_robot_system/vla_robot_system/vision
```

Create vision processor `vision_processor.py`:

```bash
nano ~/vla_robot_ws/src/vla_robot_system/vla_robot_system/vision/vision_processor.py
```

Add the following content:

```python
#!/usr/bin/env python3

"""
Vision Processor for VLA System

This module handles computer vision processing for the Vision-Language-Action system.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray, ObjectHypothesisWithPose
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO  # Using YOLO for object detection


class VisionProcessor(Node):
    def __init__(self):
        super().__init__('vision_processor')

        # Initialize CvBridge
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

        self.detection_pub = self.create_publisher(
            Detection2DArray,
            '/vla/vision/detections',
            10
        )

        # Object detection model
        try:
            self.detector = YOLO('yolov8n.pt')  # You might need to download this model
        except:
            self.get_logger().warn('YOLO model not found, using dummy detector')
            self.detector = None

        # Camera parameters
        self.camera_info = None
        self.intrinsic_matrix = None

        # Processing parameters
        self.confidence_threshold = 0.5
        self.max_objects = 10

        self.get_logger().info('Vision Processor initialized')

    def camera_info_callback(self, msg):
        """Store camera calibration information."""
        self.camera_info = msg
        self.intrinsic_matrix = np.array(msg.k).reshape(3, 3)

    def image_callback(self, msg):
        """Process incoming image and publish detections."""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            # Perform object detection
            if self.detector:
                results = self.detector(cv_image)
                detections = self.process_yolo_results(results, cv_image.shape)
            else:
                # Fallback to simple color-based detection
                detections = self.simple_color_detection(cv_image)

            # Publish detections
            self.publish_detections(detections, msg.header)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def process_yolo_results(self, results, image_shape):
        """Process YOLO detection results."""
        detections = []
        height, width = image_shape[:2]

        for result in results:
            for detection in result.boxes:
                if detection.conf[0] > self.confidence_threshold:
                    # Get bounding box coordinates
                    x1, y1, x2, y2 = detection.xyxy[0].cpu().numpy()

                    # Calculate center and size
                    center_x = (x1 + x2) / 2
                    center_y = (y1 + y2) / 2
                    box_width = x2 - x1
                    box_height = y2 - y1

                    # Get class name
                    class_id = int(detection.cls[0])
                    class_name = result.names[class_id]

                    detection_info = {
                        'class_name': class_name,
                        'confidence': float(detection.conf[0]),
                        'bbox': {
                            'center_x': float(center_x),
                            'center_y': float(center_y),
                            'width': float(box_width),
                            'height': float(box_height)
                        }
                    }
                    detections.append(detection_info)

        return detections[:self.max_objects]

    def simple_color_detection(self, image):
        """Fallback simple color-based detection."""
        # Convert BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define color ranges (simplified for red, blue, green)
        color_ranges = {
            'red': ([0, 50, 50], [10, 255, 255]),
            'red2': ([170, 50, 50], [180, 255, 255]),  # Red wraps around in HSV
            'blue': ([100, 50, 50], [130, 255, 255]),
            'green': ([40, 50, 50], [80, 255, 255])
        }

        detections = []
        for color_name, (lower, upper) in color_ranges.items():
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))

            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 1000:  # Filter small areas
                    # Get bounding box
                    x, y, w, h = cv2.boundingRect(contour)

                    detection_info = {
                        'class_name': f'{color_name}_object',
                        'confidence': 0.7,  # Fixed confidence for simplicity
                        'bbox': {
                            'center_x': float(x + w/2),
                            'center_y': float(y + h/2),
                            'width': float(w),
                            'height': float(h)
                        }
                    }
                    detections.append(detection_info)

        return detections[:self.max_objects]

    def publish_detections(self, detections, header):
        """Publish detection results as ROS messages."""
        detection_array = Detection2DArray()
        detection_array.header = header

        for detection in detections:
            detection_msg = Detection2D()
            detection_msg.header = header

            # Set bounding box
            detection_msg.bbox.center.x = detection['bbox']['center_x']
            detection_msg.bbox.center.y = detection['bbox']['center_y']
            detection_msg.bbox.size_x = detection['bbox']['width']
            detection_msg.bbox.size_y = detection['bbox']['height']

            # Add hypothesis
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.id = detection['class_name']
            hypothesis.score = detection['confidence']
            detection_msg.results.append(hypothesis)

            detection_array.detections.append(detection_msg)

        self.detection_pub.publish(detection_array)


def main(args=None):
    """Main function to run the vision processor."""
    rclpy.init(args=args)
    vision_processor = VisionProcessor()

    try:
        rclpy.spin(vision_processor)
    except KeyboardInterrupt:
        vision_processor.get_logger().info('Vision processor interrupted')
    finally:
        vision_processor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Step 4: Create Language Processing Component

Create the language processing directory:

```bash
mkdir -p ~/vla_robot_ws/src/vla_robot_system/vla_robot_system/language
```

Create language processor `language_processor.py`:

```bash
nano ~/vla_robot_ws/src/vla_robot_system/vla_robot_system/language/language_processor.py
```

Add the following content:

```python
#!/usr/bin/env python3

"""
Language Processor for VLA System

This module handles natural language processing for the Vision-Language-Action system.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
import spacy
import numpy as np
import json


class LanguageProcessor(Node):
    def __init__(self):
        super().__init__('language_processor')

        # Publishers and subscribers
        self.command_sub = self.create_subscription(
            String,
            '/vla/command',
            self.command_callback,
            10
        )

        self.parsed_command_pub = self.create_publisher(
            String,
            '/vla/parsed_command',
            10
        )

        # Initialize spaCy model
        try:
            self.nlp = spacy.load("en_core_web_sm")
        except OSError:
            self.get_logger().error("spaCy English model not found. Install with: python -m spacy download en_core_web_sm")
            self.nlp = None

        # Define robot-specific vocabulary
        self.robot_actions = {
            'move': ['go', 'move', 'navigate', 'travel', 'drive', 'go to'],
            'grasp': ['pick', 'grasp', 'take', 'grab', 'lift', 'pick up'],
            'place': ['place', 'put', 'set', 'drop', 'release', 'put down'],
            'find': ['find', 'locate', 'search', 'look for', 'detect'],
            'follow': ['follow', 'accompany', 'escort', 'go after'],
            'bring': ['bring', 'carry', 'deliver', 'take to']
        }

        # Spatial relations
        self.spatial_relations = {
            'directional': ['left', 'right', 'front', 'back', 'forward', 'backward'],
            'relative': ['near', 'far', 'close', 'next to', 'beside', 'around'],
            'topological': ['on', 'in', 'under', 'above', 'below', 'inside', 'outside'],
            'distance': ['near', 'far', 'close', 'far away', 'right', 'next']
        }

        self.get_logger().info('Language Processor initialized')

    def command_callback(self, msg):
        """Process incoming natural language command."""
        try:
            if self.nlp is None:
                self.get_logger().error("spaCy model not loaded")
                return

            # Parse the command
            parsed_command = self.parse_command(msg.data)

            # Publish parsed command
            parsed_msg = String()
            parsed_msg.data = json.dumps(parsed_command)
            self.parsed_command_pub.publish(parsed_msg)

            self.get_logger().info(f'Parsed command: {parsed_command}')

        except Exception as e:
            self.get_logger().error(f'Error processing command: {str(e)}')

    def parse_command(self, command):
        """
        Parse a natural language command into structured representation.

        Args:
            command (str): Natural language command

        Returns:
            dict: Parsed command structure
        """
        doc = self.nlp(command.lower())

        # Extract action
        action = self.extract_action(doc)

        # Extract objects
        objects = self.extract_objects(doc)

        # Extract spatial relations
        spatial_info = self.extract_spatial_info(doc)

        # Extract locations
        locations = self.extract_locations(doc)

        # Extract quantities
        quantities = self.extract_quantities(doc)

        return {
            'original_command': command,
            'action': action,
            'objects': objects,
            'spatial_relations': spatial_info,
            'locations': locations,
            'quantities': quantities,
            'confidence': 0.8  # Simplified confidence
        }

    def extract_action(self, doc):
        """Extract the main action from the command."""
        for token in doc:
            if token.pos_ == "VERB":
                # Check if it matches known robot actions
                for action, synonyms in self.robot_actions.items():
                    if token.lemma_ in synonyms or str(token) in synonyms:
                        return action
                return token.lemma_
        return None

    def extract_objects(self, doc):
        """Extract objects mentioned in the command."""
        objects = []
        for token in doc:
            if token.pos_ in ["NOUN", "PROPN"] and token.dep_ != "ROOT":
                # Skip spatial relation words
                if token.text not in [word for sublist in self.spatial_relations.values() for word in sublist]:
                    # Check if it's not a location or spatial term
                    if token.text not in ['kitchen', 'living room', 'bedroom', 'office', 'table', 'chair', 'couch']:
                        objects.append({
                            'text': token.text,
                            'lemma': token.lemma_,
                            'pos': token.pos_
                        })
        return objects

    def extract_spatial_info(self, doc):
        """Extract spatial relationships from the command."""
        spatial_info = []
        for token in doc:
            for category, relations in self.spatial_relations.items():
                if token.text in relations:
                    # Find what the spatial relation applies to
                    for child in token.children:
                        spatial_info.append({
                            'relation': token.text,
                            'category': category,
                            'reference': child.text
                        })
                    # Also check parent
                    if token.head.text not in [token.text for token in doc if token.pos_ == "VERB"]:
                        spatial_info.append({
                            'relation': token.text,
                            'category': category,
                            'reference': token.head.text
                        })
        return spatial_info

    def extract_locations(self, doc):
        """Extract location references from the command."""
        locations = []
        for ent in doc.ents:
            if ent.label_ in ["GPE", "LOC", "FAC", "PERSON"]:  # Geographic, location, facility, person
                locations.append({
                    'text': ent.text,
                    'label': ent.label_
                })

        # Also check for common location nouns
        for token in doc:
            if token.text in ['kitchen', 'living room', 'bedroom', 'office', 'table', 'chair', 'couch', 'door', 'window']:
                locations.append({
                    'text': token.text,
                    'label': 'LOCATION'
                })

        return locations

    def extract_quantities(self, doc):
        """Extract quantity information from the command."""
        quantities = []
        for token in doc:
            if token.pos_ == "NUM" or token.like_num:
                quantities.append({
                    'value': token.text,
                    'type': 'number'
                })
        return quantities


def main(args=None):
    """Main function to run the language processor."""
    rclpy.init(args=args)
    language_processor = LanguageProcessor()

    try:
        rclpy.spin(language_processor)
    except KeyboardInterrupt:
        language_processor.get_logger().info('Language processor interrupted')
    finally:
        language_processor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Step 5: Create Action Planning Component

Create the action planning directory:

```bash
mkdir -p ~/vla_robot_ws/src/vla_robot_system/vla_robot_system/action_planning
```

Create action planner `action_planner.py`:

```bash
nano ~/vla_robot_ws/src/vla_robot_system/vla_robot_system/action_planning/action_planner.py
```

Add the following content:

```python
#!/usr/bin/env python3

"""
Action Planner for VLA System

This module handles action planning based on parsed language commands and vision inputs.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point
from vision_msgs.msg import Detection2DArray
from action_msgs.msg import GoalStatus
import json
import numpy as np


class ActionPlanner(Node):
    def __init__(self):
        super().__init__('action_planner')

        # Publishers and subscribers
        self.parsed_command_sub = self.create_subscription(
            String,
            '/vla/parsed_command',
            self.parsed_command_callback,
            10
        )

        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/vla/vision/detections',
            self.detection_callback,
            10
        )

        self.action_sequence_pub = self.create_publisher(
            String,
            '/vla/action_sequence',
            10
        )

        # Store detected objects and environment state
        self.detected_objects = {}
        self.environment_state = {
            'object_locations': {},
            'robot_pose': Pose(),
            'locations': {
                'kitchen': Point(x=2.0, y=1.0, z=0.0),
                'living_room': Point(x=0.0, y=0.0, z=0.0),
                'bedroom': Point(x=-2.0, y=1.0, z=0.0),
                'office': Point(x=0.0, y=2.0, z=0.0)
            }
        }

        self.current_command = None
        self.current_detections = []

        self.get_logger().info('Action Planner initialized')

    def parsed_command_callback(self, msg):
        """Process parsed command and generate action sequence."""
        try:
            parsed_data = json.loads(msg.data)
            self.current_command = parsed_data

            # Plan actions based on command and current environment state
            action_sequence = self.plan_actions(parsed_data)

            # Publish action sequence
            action_msg = String()
            action_msg.data = json.dumps(action_sequence)
            self.action_sequence_pub.publish(action_msg)

            self.get_logger().info(f'Planned actions: {action_sequence}')

        except Exception as e:
            self.get_logger().error(f'Error planning actions: {str(e)}')

    def detection_callback(self, msg):
        """Update environment state with new detections."""
        detected_objects = []
        for detection in msg.detections:
            if detection.results:
                best_result = max(detection.results, key=lambda x: x.score)
                detected_objects.append({
                    'class': best_result.id,
                    'confidence': best_result.score,
                    'position': {
                        'x': detection.bbox.center.x,
                        'y': detection.bbox.center.y,
                        'width': detection.bbox.size_x,
                        'height': detection.bbox.size_y
                    }
                })

        self.current_detections = detected_objects

        # Update environment state with object locations
        for obj in detected_objects:
            self.environment_state['object_locations'][obj['class']] = obj['position']

    def plan_actions(self, parsed_command):
        """
        Plan sequence of actions based on parsed command and environment state.

        Args:
            parsed_command (dict): Parsed natural language command

        Returns:
            list: Sequence of executable actions
        """
        action_sequence = []
        action = parsed_command.get('action')
        objects = parsed_command.get('objects', [])
        locations = parsed_command.get('locations', [])
        spatial_relations = parsed_command.get('spatial_relations', [])

        if action == 'find':
            if objects:
                # Find the specified object
                object_name = objects[0]['text'] if objects else 'object'
                action_sequence.extend(self.find_object_actions(object_name))

        elif action == 'grasp':
            if objects:
                # Find and grasp the specified object
                object_name = objects[0]['text'] if objects else 'object'
                action_sequence.extend(self.find_object_actions(object_name))
                action_sequence.append({
                    'action': 'grasp_object',
                    'object': object_name,
                    'location': self.find_object_location(object_name)
                })

        elif action == 'move':
            if locations:
                # Navigate to specified location
                location_name = locations[0]['text'] if locations else 'location'
                action_sequence.append({
                    'action': 'navigate_to',
                    'location': location_name,
                    'coordinates': self.get_location_coordinates(location_name)
                })

        elif action == 'place':
            if locations:
                # Place carried object at specified location
                location_name = locations[0]['text'] if locations else 'location'
                action_sequence.append({
                    'action': 'place_object',
                    'location': location_name,
                    'coordinates': self.get_location_coordinates(location_name)
                })

        elif action == 'bring':
            if objects and locations:
                # Find object, grasp it, then navigate to location and place it
                object_name = objects[0]['text']
                location_name = locations[0]['text']

                action_sequence.extend(self.find_object_actions(object_name))
                action_sequence.append({
                    'action': 'grasp_object',
                    'object': object_name,
                    'location': self.find_object_location(object_name)
                })
                action_sequence.append({
                    'action': 'navigate_to',
                    'location': location_name,
                    'coordinates': self.get_location_coordinates(location_name)
                })
                action_sequence.append({
                    'action': 'place_object',
                    'location': location_name,
                    'coordinates': self.get_location_coordinates(location_name)
                })

        # Add error handling and validation
        action_sequence = self.validate_action_sequence(action_sequence)

        return {
            'command': parsed_command['original_command'],
            'actions': action_sequence,
            'timestamp': self.get_clock().now().to_msg()
        }

    def find_object_actions(self, object_name):
        """Generate actions to find a specific object."""
        actions = []

        # Look for object in current detections
        found_object = self.find_object_in_detections(object_name)

        if found_object:
            # Object is visible, move closer if needed
            actions.append({
                'action': 'move_to_object',
                'object': object_name,
                'location': found_object['position']
            })
        else:
            # Object not visible, need to search
            actions.append({
                'action': 'search_for_object',
                'object': object_name
            })

        return actions

    def find_object_in_detections(self, object_name):
        """Check if specified object is in current detections."""
        for detection in self.current_detections:
            if object_name.lower() in detection['class'].lower():
                return detection
        return None

    def find_object_location(self, object_name):
        """Find the location of a specific object."""
        if object_name in self.environment_state['object_locations']:
            return self.environment_state['object_locations'][object_name]
        return None

    def get_location_coordinates(self, location_name):
        """Get coordinates for a named location."""
        if location_name in self.environment_state['locations']:
            return {
                'x': self.environment_state['locations'][location_name].x,
                'y': self.environment_state['locations'][location_name].y,
                'z': self.environment_state['locations'][location_name].z
            }
        return {'x': 0.0, 'y': 0.0, 'z': 0.0}  # Default coordinates

    def validate_action_sequence(self, action_sequence):
        """Validate and refine the action sequence."""
        # Check for logical consistency
        validated_sequence = []

        for action in action_sequence:
            # Skip actions that don't have required information
            if self.is_action_valid(action):
                validated_sequence.append(action)

        return validated_sequence

    def is_action_valid(self, action):
        """Check if an action has all required information."""
        required_fields = {
            'navigate_to': ['location'],
            'grasp_object': ['object'],
            'place_object': ['location'],
            'move_to_object': ['object']
        }

        action_type = action.get('action')
        if action_type in required_fields:
            for field in required_fields[action_type]:
                if field not in action or action[field] is None:
                    return False

        return True


def main(args=None):
    """Main function to run the action planner."""
    rclpy.init(args=args)
    action_planner = ActionPlanner()

    try:
        rclpy.spin(action_planner)
    except KeyboardInterrupt:
        action_planner.get_logger().info('Action planner interrupted')
    finally:
        action_planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Step 6: Create VLA Integration Node

Create the main integration node:

```bash
nano ~/vla_robot_ws/src/vla_robot_system/vla_robot_system/vla_integration.py
```

Add the following content:

```python
#!/usr/bin/env python3

"""
VLA Integration Node

This node integrates Vision, Language, and Action components into a complete system.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Twist
import json


class VLAIntegrationNode(Node):
    def __init__(self):
        super().__init__('vla_integration')

        # Publishers and subscribers
        self.command_sub = self.create_subscription(
            String,
            '/vla/command',
            self.command_callback,
            10
        )

        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/vla/vision/detections',
            self.detection_callback,
            10
        )

        self.action_sequence_sub = self.create_subscription(
            String,
            '/vla/action_sequence',
            self.action_sequence_callback,
            10
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.status_pub = self.create_publisher(
            String,
            '/vla/status',
            10
        )

        # System state
        self.system_state = {
            'current_command': None,
            'current_detections': [],
            'current_action_sequence': [],
            'current_action_index': 0,
            'executing_action': False,
            'carrying_object': False
        }

        # Action execution timers
        self.action_timer = None
        self.current_action_start_time = None

        self.get_logger().info('VLA Integration Node initialized')

    def command_callback(self, msg):
        """Handle new command input."""
        self.system_state['current_command'] = msg.data
        self.system_state['current_action_index'] = 0
        self.system_state['executing_action'] = False
        self.get_logger().info(f'Received command: {msg.data}')

    def detection_callback(self, msg):
        """Update system with new detections."""
        detections = []
        for detection in msg.detections:
            if detection.results:
                best_result = max(detection.results, key=lambda x: x.score)
                detections.append({
                    'class': best_result.id,
                    'confidence': best_result.score,
                    'bbox': {
                        'center_x': detection.bbox.center.x,
                        'center_y': detection.bbox.center.y,
                        'size_x': detection.bbox.size_x,
                        'size_y': detection.bbox.size_y
                    }
                })

        self.system_state['current_detections'] = detections

    def action_sequence_callback(self, msg):
        """Handle new action sequence."""
        try:
            action_data = json.loads(msg.data)
            self.system_state['current_action_sequence'] = action_data['actions']
            self.system_state['current_action_index'] = 0
            self.system_state['executing_action'] = False

            self.get_logger().info(f'Received action sequence with {len(action_data["actions"])} actions')

            # Start executing the sequence
            self.execute_next_action()

        except Exception as e:
            self.get_logger().error(f'Error processing action sequence: {str(e)}')

    def execute_next_action(self):
        """Execute the next action in the sequence."""
        if (self.system_state['current_action_index'] <
            len(self.system_state['current_action_sequence'])):

            action = self.system_state['current_action_sequence'][self.system_state['current_action_index']]

            self.get_logger().info(f'Executing action: {action}')

            # Execute the action based on its type
            success = self.execute_action(action)

            if success:
                # Move to next action
                self.system_state['current_action_index'] += 1

                # If there are more actions, execute the next one
                if self.system_state['current_action_index'] < len(self.system_state['current_action_sequence']):
                    self.execute_next_action()
                else:
                    # All actions completed
                    self.publish_status('completed')
            else:
                # Action failed
                self.publish_status('failed')
        else:
            # All actions completed
            self.publish_status('completed')

    def execute_action(self, action):
        """
        Execute a single action.

        Args:
            action (dict): Action to execute

        Returns:
            bool: True if action completed successfully
        """
        action_type = action.get('action')

        if action_type == 'navigate_to':
            return self.execute_navigation(action)
        elif action_type == 'grasp_object':
            return self.execute_grasp(action)
        elif action_type == 'place_object':
            return self.execute_place(action)
        elif action_type == 'move_to_object':
            return self.execute_move_to_object(action)
        elif action_type == 'search_for_object':
            return self.execute_search(action)
        else:
            self.get_logger().warn(f'Unknown action type: {action_type}')
            return False

    def execute_navigation(self, action):
        """Execute navigation action."""
        try:
            target_location = action.get('coordinates', {'x': 0.0, 'y': 0.0, 'z': 0.0})

            # Simple navigation: move toward target
            twist = Twist()
            twist.linear.x = 0.2  # Move forward at 0.2 m/s
            twist.angular.z = 0.0  # No rotation for simplicity

            # Publish command for 2 seconds (this is simplified)
            for _ in range(20):  # 20 iterations at 10Hz = 2 seconds
                self.cmd_vel_pub.publish(twist)
                self.get_clock().sleep_for(rclpy.time.Duration(seconds=0.1))

            # Stop robot
            stop_twist = Twist()
            self.cmd_vel_pub.publish(stop_twist)

            return True
        except Exception as e:
            self.get_logger().error(f'Navigation error: {str(e)}')
            return False

    def execute_grasp(self, action):
        """Execute object grasping action."""
        try:
            object_name = action.get('object', 'unknown')
            self.get_logger().info(f'Attempting to grasp {object_name}')

            # Simulate grasping action
            # In a real system, this would interface with the robot's gripper
            self.system_state['carrying_object'] = True

            # Wait for grasp completion
            self.get_clock().sleep_for(rclpy.time.Duration(seconds=1.0))

            self.get_logger().info(f'Successfully grasped {object_name}')
            return True
        except Exception as e:
            self.get_logger().error(f'Grasp error: {str(e)}')
            return False

    def execute_place(self, action):
        """Execute object placement action."""
        try:
            location_name = action.get('location', 'unknown')
            self.get_logger().info(f'Attempting to place object at {location_name}')

            # Simulate placement action
            # In a real system, this would interface with the robot's gripper
            self.system_state['carrying_object'] = False

            # Wait for placement completion
            self.get_clock().sleep_for(rclpy.time.Duration(seconds=1.0))

            self.get_logger().info(f'Successfully placed object at {location_name}')
            return True
        except Exception as e:
            self.get_logger().error(f'Placement error: {str(e)}')
            return False

    def execute_move_to_object(self, action):
        """Execute move to object action."""
        try:
            object_name = action.get('object', 'unknown')
            self.get_logger().info(f'Moving toward {object_name}')

            # Simple movement toward object
            twist = Twist()
            twist.linear.x = 0.1  # Move forward slowly
            twist.angular.z = 0.0

            # Move for 1 second (simplified)
            for _ in range(10):  # 10 iterations at 10Hz = 1 second
                self.cmd_vel_pub.publish(twist)
                self.get_clock().sleep_for(rclpy.time.Duration(seconds=0.1))

            # Stop robot
            stop_twist = Twist()
            self.cmd_vel_pub.publish(stop_twist)

            return True
        except Exception as e:
            self.get_logger().error(f'Move to object error: {str(e)}')
            return False

    def execute_search(self, action):
        """Execute search action."""
        try:
            object_name = action.get('object', 'unknown')
            self.get_logger().info(f'Searching for {object_name}')

            # Simple search pattern: rotate and look around
            twist = Twist()
            twist.angular.z = 0.5  # Rotate at 0.5 rad/s

            # Rotate for 2 seconds (simplified search)
            for _ in range(20):  # 20 iterations at 10Hz = 2 seconds
                self.cmd_vel_pub.publish(twist)
                self.get_clock().sleep_for(rclpy.time.Duration(seconds=0.1))

            # Stop rotation
            stop_twist = Twist()
            self.cmd_vel_pub.publish(stop_twist)

            return True
        except Exception as e:
            self.get_logger().error(f'Search error: {str(e)}')
            return False

    def publish_status(self, status):
        """Publish system status."""
        status_msg = String()
        status_msg.data = f'Status: {status}, Command: {self.system_state["current_command"]}'
        self.status_pub.publish(status_msg)


def main(args=None):
    """Main function to run the VLA integration."""
    rclpy.init(args=args)
    vla_node = VLAIntegrationNode()

    try:
        rclpy.spin(vla_node)
    except KeyboardInterrupt:
        vla_node.get_logger().info('VLA integration interrupted')
    finally:
        # Stop robot if it's moving
        stop_twist = Twist()
        vla_node.cmd_vel_pub.publish(stop_twist)

        vla_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Step 7: Create Launch File

Create a launch directory and file:

```bash
mkdir -p ~/vla_robot_ws/src/vla_robot_system/vla_robot_system/launch
nano ~/vla_robot_ws/src/vla_robot_system/vla_robot_system/launch/vla_system_launch.py
```

Add the following content:

```python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # VLA Integration Node
    vla_integration_node = Node(
        package='vla_robot_system',
        executable='vla_integration',
        name='vla_integration',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # Vision Processor Node
    vision_processor_node = Node(
        package='vla_robot_system',
        executable='vision_processor',
        name='vision_processor',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # Language Processor Node
    language_processor_node = Node(
        package='vla_robot_system',
        executable='language_processor',
        name='language_processor',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # Action Planner Node
    action_planner_node = Node(
        package='vla_robot_system',
        executable='action_planner',
        name='action_planner',
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
    ld.add_action(vla_integration_node)
    ld.add_action(vision_processor_node)
    ld.add_action(language_processor_node)
    ld.add_action(action_planner_node)

    return ld
```

## Step 8: Update Package Configuration

Update the package.xml file:

```bash
nano ~/vla_robot_ws/src/vla_robot_system/package.xml
```

Add the following content:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>vla_robot_system</name>
  <version>0.0.0</version>
  <description>Vision-Language-Action Robot System</description>
  <maintainer email="your.email@example.com">Your Name</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_python</buildtool_depend>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>vision_msgs</depend>
  <depend>cv_bridge</depend>
  <depend>action_msgs</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

Update the setup.py file:

```bash
nano ~/vla_robot_ws/src/vla_robot_system/setup.py
```

Add the following content:

```python
from setuptools import setup
from glob import glob
import os

package_name = 'vla_robot_system'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('vla_robot_system/launch/*')),
        (os.path.join('share', package_name, 'config'), glob('vla_robot_system/config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Vision-Language-Action Robot System',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vla_integration = vla_robot_system.vla_integration:main',
            'vision_processor = vla_robot_system.vision.vision_processor:main',
            'language_processor = vla_robot_system.language.language_processor:main',
            'action_planner = vla_robot_system.action_planning.action_planner:main',
        ],
    },
)
```

## Step 9: Build and Test the VLA System

Build the package:

```bash
cd ~/vla_robot_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select vla_robot_system
```

Source the workspace:

```bash
source install/setup.bash
```

## Step 10: Run the VLA System Test

To test the complete VLA system:

1. Start Gazebo simulation with a test world:
```bash
# Terminal 1
cd ~/vla_robot_ws
source install/setup.bash
ros2 launch gazebo_ros gazebo.launch.py world:=/usr/share/gazebo-11/worlds/empty.world
```

2. Start the VLA system:
```bash
# Terminal 2
cd ~/vla_robot_ws
source install/setup.bash
ros2 launch vla_robot_system vla_system_launch.py
```

3. Send a test command:
```bash
# Terminal 3
cd ~/vla_robot_ws
source install/setup.bash
ros2 topic pub /vla/command std_msgs/String "data: 'find the red cup'"
```

4. Monitor the system status:
```bash
# Terminal 4
cd ~/vla_robot_ws
source install/setup.bash
ros2 topic echo /vla/status
```

## Step 11: Advanced Testing with Real Commands

Test with various commands to validate the VLA system:

```bash
# Test different commands
ros2 topic pub /vla/command std_msgs/String "data: 'go to the kitchen'"
ros2 topic pub /vla/command std_msgs/String "data: 'find the blue ball'"
ros2 topic pub /vla/command std_msgs/String "data: 'bring the book to me'"
ros2 topic pub /vla/command std_msgs/String "data: 'pick up the cup and place it on the table'"
```

## Troubleshooting Tips

1. **Missing dependencies**: Install required packages like `ultralytics`, `spacy`, `transformers`
2. **Model loading**: Download required models (YOLO, spaCy language model)
3. **Topic connections**: Verify all nodes are connected properly
4. **Permission issues**: Ensure all Python files have execute permissions
5. **ROS environment**: Make sure ROS environment is properly sourced

## Expected Results

When running the VLA system, you should observe:
- Vision system detecting objects in the environment
- Language system parsing natural commands
- Action planner generating appropriate action sequences
- Robot executing actions based on commands
- Coordinated behavior between vision, language, and action components

## Performance Evaluation

To evaluate the VLA system performance:

1. **Accuracy**: Measure how often the system correctly interprets commands
2. **Response time**: Measure time from command input to action execution
3. **Success rate**: Measure percentage of tasks completed successfully
4. **Robustness**: Test performance under various conditions

## Key Takeaways

- You've implemented a complete Vision-Language-Action system
- You've integrated computer vision, natural language processing, and action planning
- You've created a multimodal interface for robot interaction
- You understand the challenges and solutions in VLA integration
- You can evaluate and improve VLA system performance

## References

[VLA Bibliography](/docs/references/vla-bibliography.md)