#!/usr/bin/env python3

"""
Basic Gazebo Model Spawner

This script demonstrates how to programmatically spawn a model in Gazebo using ROS 2 services.
"""

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
import time


class GazeboModelSpawner(Node):
    """
    A node that spawns a model in Gazebo using the spawn_entity service.
    """

    def __init__(self):
        super().__init__('gazebo_model_spawner')

        # Create a client for the spawn_entity service
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')

        # Wait for the service to be available
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /spawn_entity not available, waiting again...')

        self.get_logger().info('Connected to spawn_entity service')

    def spawn_model(self, model_name, model_xml, initial_pose):
        """
        Spawn a model in Gazebo.

        Args:
            model_name (str): Name of the model to spawn
            model_xml (str): XML description of the model
            initial_pose: Initial pose for the model (geometry_msgs/Pose)
        """
        request = SpawnEntity.Request()
        request.name = model_name
        request.xml = model_xml
        request.initial_pose = initial_pose

        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Successfully spawned model: {model_name}')
            else:
                self.get_logger().error(f'Failed to spawn model: {response.status_message}')
        else:
            self.get_logger().error('Service call failed')


def main(args=None):
    """
    Main function to demonstrate model spawning.
    """
    rclpy.init(args=args)

    spawner = GazeboModelSpawner()

    # Example: Simple box model
    box_model_xml = """
    <?xml version="1.0"?>
    <sdf version="1.7">
      <model name="demo_box">
        <pose>0 0 0.5 0 0 0</pose>
        <link name="box_link">
          <inertial>
            <mass>1.0</mass>
            <inertia>
              <ixx>0.1</ixx>
              <ixy>0</ixy>
              <ixz>0</ixz>
              <iyy>0.1</iyy>
              <iyz>0</iyz>
              <izz>0.1</izz>
            </inertia>
          </inertial>
          <visual name="visual">
            <geometry>
              <box>
                <size>1 1 1</size>
              </box>
            </geometry>
            <material>
              <ambient>1 0 0 1</ambient>
              <diffuse>1 0 0 1</diffuse>
            </material>
          </visual>
          <collision name="collision">
            <geometry>
              <box>
                <size>1 1 1</size>
              </box>
            </geometry>
          </collision>
        </link>
      </model>
    </sdf>
    """

    # Import Pose from geometry_msgs
    from geometry_msgs.msg import Pose
    initial_pose = Pose()
    initial_pose.position.x = 1.0
    initial_pose.position.y = 0.0
    initial_pose.position.z = 0.5
    initial_pose.orientation.w = 1.0

    # Spawn the model
    spawner.spawn_model("demo_box", box_model_xml, initial_pose)

    # Allow some time for the model to spawn
    time.sleep(1.0)

    spawner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()