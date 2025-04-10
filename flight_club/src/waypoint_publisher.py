#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseArray, Pose
import numpy as np
import os

TOPIC_NAMESPACE = 'rob498_drone_6'

class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_publisher')
        
        # Declare and get the output folder parameter
        self.declare_parameter('output_folder', '/tmp')
        self.output_folder = self.get_parameter('output_folder').get_parameter_value().string_value
        
        # Publisher for waypoints
        self.waypoint_pub = self.create_publisher(PoseArray, f'{TOPIC_NAMESPACE}/comm/waypoints', 10)
        
        # Service to trigger waypoint publishing
        self.srv_send_waypoints = self.create_service(Trigger, f'{TOPIC_NAMESPACE}/comm/send_waypoints', self.send_waypoints_callback)
        
        # Load waypoints and initialize index
        self.waypoints = self.load_waypoints()
        self.current_index = 0

        self.get_logger().info("Waypoint Publisher Node is ready.")

    def send_waypoints_callback(self, request, response):
        if self.waypoints is None or self.current_index >= len(self.waypoints.poses):
            response.success = False
            response.message = "No more waypoints to send."
            self.get_logger().error("No more waypoints to send.")
            return response

        # Publish the current waypoint
        waypoint = PoseArray()
        waypoint.poses.append(self.waypoints.poses[self.current_index])
        self.waypoint_pub.publish(waypoint)

        self.get_logger().info(f"Published waypoint {self.current_index + 1}/{len(self.waypoints.poses)}.")

        # Increment the index for the next call
        self.current_index += 1

        response.success = True
        response.message = f"Waypoint {self.current_index} sent successfully."
        return response

    def load_waypoints(self):
        # Load waypoints from the waypoint.npy file
        waypoint_file = os.path.join(self.output_folder, 'waypoints.npy')
        if not os.path.exists(waypoint_file):
            self.get_logger().error(f"Waypoint file not found: {waypoint_file}")
            return None

        try:
            positions = np.load(waypoint_file)
        except Exception as e:
            self.get_logger().error(f"Error loading waypoint file: {e}")
            return None

        waypoints = PoseArray()
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
