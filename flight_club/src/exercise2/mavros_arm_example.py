#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool
from geometry_msgs.msg import PoseStamped

class OffboardNode(Node):
    def __init__(self):
        super().__init__('offboard_node')

        self.state = State()
        self.pose_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, 10)
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')

        self.timer = self.create_timer(0.1, self.publish_pose)  # Publish at 10 Hz

    def state_callback(self, msg):
        self.state = msg

    def publish_pose(self):
        pose = PoseStamped()
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.0
        self.pose_pub.publish(pose)

        if self.state.mode != "OFFBOARD":
            self.set_offboard_mode()

        if not self.state.armed:
            self.arm_drone()

    def set_offboard_mode(self):
        if self.set_mode_client.wait_for_service(timeout_sec=1.0):
            req = SetMode.Request()
            req.custom_mode = "OFFBOARD"
            future = self.set_mode_client.call_async(req)
            self.get_logger().info("Trying to set OFFBOARD mode")
        else:
            self.get_logger().warn("SetMode service not available")

    def arm_drone(self):
        if self.arm_client.wait_for_service(timeout_sec=1.0):
            req = CommandBool.Request()
            req.value = True
            future = self.arm_client.call_async(req)
            self.get_logger().info("Trying to arm the drone")
        else:
            self.get_logger().warn("Arming service not available")

def main(args=None):
    rclpy.init(args=args)
    node = OffboardNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
