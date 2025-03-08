#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

class OcclusionVisualizer(Node):
    def __init__(self):
        super().__init__("occlusion_visualizer")
        self.publisher = self.create_publisher(MarkerArray, "/occlusion_markers", 10)
        self.timer = self.create_timer(1.0, self.publish_occlusion)
        self.occluded_voxels = np.load("occluded_voxels.npy")  # Load occlusion data
        self.voxel_size = 0.1  # Assuming 10cm resolution

    def publish_occlusion(self):
        marker_array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = Marker.CUBE_LIST  # More efficient than individual cubes
        marker.scale.x = self.voxel_size
        marker.scale.y = self.voxel_size
        marker.scale.z = self.voxel_size
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.5

        # Find all occluded voxels in a single operation
        occluded_indices = np.argwhere(self.occluded_voxels == -1)  # Get indices of occluded cells
        positions = occluded_indices * self.voxel_size  # Convert indices to world positions

        # Convert positions to ROS messages
        for pos in positions:
            point = Point()
            point.x, point.y, point.z = pos
            marker.points.append(point)

        marker_array.markers.append(marker)
        self.publisher.publish(marker_array)

def main():
    rclpy.init()
    node = OcclusionVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
