#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import GetWorldProperties

class RaycastNode(Node):
    def __init__(self):
        super().__init__("raycast_node")
        self.client = self.create_client(GetWorldProperties, "/gazebo/get_world_properties")
        self.ray_directions = self.generate_ray_directions()
        self.seeker_position = np.array([1, 2, 3])

    def generate_ray_directions(self, num_rays=10000):
        """Generate rays in a uniform spherical distribution."""
        directions = []
        for _ in range(num_rays):
            theta = np.arccos(1 - 2 * np.random.rand())  # Polar angle
            phi = 2 * np.pi * np.random.rand()  # Azimuthal angle
            x = np.sin(theta) * np.cos(phi)
            y = np.sin(theta) * np.sin(phi)
            z = np.cos(theta)
            directions.append((x, y, z))
        return np.array(directions)

    def fire_rays(self):
        """Fire rays and get hit points."""
        visible_points = []
        for direction in self.ray_directions:
            hit_point = self.cast_ray(self.seeker_position, direction)
            if hit_point:
                visible_points.append(hit_point)
        np.save("hit_points.npy", np.array(visible_points))

    def cast_ray(self):
        """Simulate raycast in Gazebo"""
        request = GetWorldProperties.Request()
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result():
            return future.result().hit_point
        return None

def main():
    rclpy.init()
    node = RaycastNode()
    node.fire_rays()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
