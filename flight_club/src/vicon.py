#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np
# from tf_transformations import euler_from_quaternion, quaternion_from_euler


class CameraPoseForward(Node):
    def __init__(self):
        super().__init__('camera_pose_forward')
        self.declare_parameter('sim')
        self.declare_parameter('remap')
        self.sim = self.get_parameter('sim').get_parameter_value().bool_value
        self.remap = self.get_parameter('remap').get_parameter_value().bool_value

        if self.sim:
            self.get_logger().info("vicon: simulation")
            self.subscription = self.create_subscription(
                PoseStamped,
                '/mavros/global_position/local',
                self.pose_callback,
                10
            )
        else:
            self.get_logger().info("vicon: real")
            self.subscription = self.create_subscription(
                PoseStamped,
                '/vicon/ROB498_Drone/ROB498_Drone',
                self.pose_callback,
                10
            )
        self.publisher = self.create_publisher(
            PoseStamped,
            '/mavros/vision_pose/pose',
            10
        )
        
        if self.remap:
            # Initialize state
            self.pose_buffer = []  # Buffer to hold poses for averaging
            self.origin_set = False
            self.origin_position = None
            self.origin_orientation = None
        else:
            self.origin_set = True
            self.origin_position = np.array([0,0,0])
            # self.origin_orientation = np.array([0,0,0,1])

        
        self.get_logger().info("camera_pose_forward node started")

    def pose_callback(self, msg):
        # Store received pose data in buffer
        self.pose_buffer.append(msg.pose)

        if len(self.pose_buffer) >= 10 and not self.origin_set:
            # Compute the average position
            avg_position = np.mean([np.array([pose.position.x, pose.position.y, pose.position.z]) for pose in self.pose_buffer], axis=0)

            # Compute the standard deviation for position
            position_std = np.std([np.array([pose.position.x, pose.position.y, pose.position.z]) for pose in self.pose_buffer], axis=0)

            # Check if the variance is below threshold (3 cm)
            if np.all(position_std < 0.03):
                self.get_logger().info("Origin set successfully.")
                self.origin_set = True
                self.origin_position = avg_position
            else:
                self.get_logger().info("Waiting for stable pose measurements...")

            # Reset buffer for next round
            self.pose_buffer = []

        # If origin is set, publish pose relative to origin
        if self.origin_set:
            # Subtract the origin from the current pose
            relative_position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]) - self.origin_position

            # We do not need to modify orientation, assuming no transformation needed
            relative_orientation = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]

            # Prepare and publish the new pose
            new_message = PoseStamped()
            new_message.header.stamp = msg.header.stamp
            new_message.header.frame_id = msg.header.frame_id
            new_message.pose.position.x, new_message.pose.position.y, new_message.pose.position.z = relative_position
            new_message.pose.orientation.x, new_message.pose.orientation.y, new_message.pose.orientation.z, new_message.pose.orientation.w = relative_orientation
            self.publisher.publish(new_message)

def main(args=None):
    rclpy.init(args=args)
    node = CameraPoseForward()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
