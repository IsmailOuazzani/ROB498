#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class CameraPoseForward(Node):
    def _init_(self):
        super()._init_('camera_pose_forward')
        self.subscription = self.create_subscription(
                PoseStamped,
                '/camera/pose/sample',  # Update if your topic is different
                self.pose_callback,
                10  # QoS depth
                )
        self.publisher = self.create_publisher(
                PoseStamped,
                '/camera/pose/forwarded',
                10
                )
        self.get_logger().info("camera_pose_forward node started")

    def pose_callback(self, msg):
        self.get_logger().info(f"Received pose: {msg}")
        self.publisher.publish(msg)  # Forward the pose message

def main(args=None):
    rclpy.init(args=args)
    node = CameraPoseForward()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if _name_ == '_main_':
    main()