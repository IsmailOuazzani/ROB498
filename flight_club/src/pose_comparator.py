#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates
import numpy as np

class PoseComparator(Node):
    def __init__(self):
        super().__init__('pose_comparator')

        # Subscribe to MAVROS local position
        self.mavros_sub = self.create_subscription(
            PoseStamped, 
            '/mavros/local_position/pose', 
            self.mavros_callback, 
            10
        )

        # Subscribe to Gazebo model states
        self.gazebo_sub = self.create_subscription(
            ModelStates, 
            '/gazebo/model_states', 
            self.gazebo_callback, 
            10
        )

        self.mavros_pose = None
        self.gazebo_pose = None

    def mavros_callback(self, msg):
        self.mavros_pose = msg.pose
        self.compare_poses()

    def gazebo_callback(self, msg):
        try:
            index = msg.name.index('iris')  # Find the index of "iris"
            self.gazebo_pose = msg.pose[index]
            self.compare_poses()
        except ValueError:
            self.get_logger().warn("❌ Model 'iris' not found in Gazebo!")

    def compare_poses(self):
        if self.mavros_pose is None or self.gazebo_pose is None:
            return
        
        # Extract positions
        mavros_pos = np.array([
            self.mavros_pose.position.x, 
            self.mavros_pose.position.y, 
            self.mavros_pose.position.z
        ])
        
        gazebo_pos = np.array([
            self.gazebo_pose.position.x, 
            self.gazebo_pose.position.y, 
            self.gazebo_pose.position.z
        ])
        
        # Compute position error
        pos_error = np.linalg.norm(mavros_pos - gazebo_pos)
        
        # Extract orientations (quaternions)
        q1 = self.mavros_pose.orientation
        q2 = self.gazebo_pose.orientation

        # Quaternion dot product for similarity
        quat_diff = abs(q1.x * q2.x + q1.y * q2.y + q1.z * q2.z + q1.w * q2.w)

        # Tolerances
        pos_tolerance = 0.1  # meters
        quat_tolerance = 0.99  # close to 1 means similar orientation

        # Print results
        self.get_logger().info(f"Position Error: {pos_error:.4f} m")
        self.get_logger().info(f"Quaternion Similarity: {quat_diff:.4f}")

        if pos_error < pos_tolerance and quat_diff > quat_tolerance:
            self.get_logger().info("✅ MAVROS and Gazebo poses MATCH!")
        else:
            self.get_logger().warn("❌ Poses DO NOT match!")

def main(args=None):
    rclpy.init(args=args)
    node = PoseComparator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
