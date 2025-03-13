#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseArray, Pose
import math
import numpy as np

TOPIC_NAMESPACE = 'rob498_drone_06'

class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_publisher')
        
        # Publisher for waypoints
        self.waypoint_pub = self.create_publisher(PoseArray, f'{TOPIC_NAMESPACE}/comm/waypoints', 10)
        
        # Service to trigger waypoint publishing
        self.srv_send_waypoints = self.create_service(Trigger, f'{TOPIC_NAMESPACE}/comm/send_waypoints', self.send_waypoints_callback)
        
        self.get_logger().info("Waypoint Publisher Node is ready.")

    def send_waypoints_callback(self, request, response):
        self.get_logger().info("Publishing waypoints...")

        waypoints = self.generate_waypoints()
        
        # Publish waypoints
        self.waypoint_pub.publish(waypoints)

        response.success = True
        response.message = "Waypoints sent successfully."
        self.get_logger().info("Waypoints sent successfully.")
        return response

    def generate_waypoints(self):
        waypoints = PoseArray()
        height = 1.5
        positions = [(3*(1-math.cos(t)), 3*(math.sin(t)), height) for t in np.linspace(0, 2*np.pi, 6)]

        for pos in positions:
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = pos
            waypoints.poses.append(pose)
        
        return waypoints

def main(args=None):
    rclpy.init(args=args)
    node = WaypointPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
