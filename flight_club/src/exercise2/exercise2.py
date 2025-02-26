Share


You said:
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool
from geometry_msgs.msg import PoseStamped
import logging

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

TOPIC_NAMESPACE = 'rob498_drone_06'

class CommNode(Node):
    def __init__(self):
        super().__init__(TOPIC_NAMESPACE)
        self.srv_launch = self.create_service(Trigger, f'{TOPIC_NAMESPACE}/comm/launch', self.launch_callback)
        self.srv_test = self.create_service(Trigger, f'{TOPIC_NAMESPACE}/comm/test', self.test_callback)
        self.srv_land = self.create_service(Trigger, f'{TOPIC_NAMESPACE}/comm/land', self.land_callback)
        self.srv_abort = self.create_service(Trigger, f'{TOPIC_NAMESPACE}/comm/abort', self.abort_callback)
        
        self.should_fly = False
        self.state = State()
        self.pose_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, 10)
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')

        self.timer = self.create_timer(0.1, self.publish_pose)  # Publish at 10 Hz

    def state_callback(self, msg):
        self.state = msg

    def launch_callback(self, request, response):
        self.get_logger().info('Launch Requested. Drone taking off.')
        self.should_fly = True
        response.success = True
        response.message = "Launch command executed."
        return response

    def test_callback(self, request, response):
        self.get_logger().info('Test Requested. Drone performing tasks.')
        response.success = True
        response.message = "Test command executed."
        return response

    def land_callback(self, request, response):
        self.get_logger().info('Land Requested. Drone landing.')
        self.should_fly = False
        self.command_land()
        response.success = True
        response.message = "Land command executed."
        return response

    def abort_callback(self, request, response):
        self.get_logger().warning('Abort Requested. Emergency landing.')
        self.should_fly = False
        self.command_land()
        response.success = True
        response.message = "Abort command executed."
        return response

    def publish_pose(self):
        if self.should_fly:
            pose = PoseStamped()
            pose.pose.position.x = 0.0
            pose.pose.position.y = 0.0
            pose.pose.position.z = 2.0  # Set desired altitude
            self.pose_pub.publish(pose)

            if self.state.mode != "OFFBOARD":
                self.set_offboard_mode()

            if not self.state.armed:
                self.arm_drone()
        elif self.state.armed:
            self.command_land()

    def command_land(self):
        self.get_logger().info('Commanding drone to land.')
        pose = PoseStamped()
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.0  # Set to ground level
        self.pose_pub.publish(pose)

    def set_offboard_mode(self):
        if self.set_mode_client.wait_for_service(timeout_sec=1.0):
            req = SetMode.Request()
            req.custom_mode = "OFFBOARD"
            self.set_mode_client.call_async(req)
            self.get_logger().info("Switching to OFFBOARD mode")
        else:
            self.get_logger().warn("Set mode service not available")

    def arm_drone(self):
        if self.arm_client.wait_for_service(timeout_sec=1.0):
            req = CommandBool.Request()
            req.value = True
            self.arm_client.call_async(req)
            self.get_logger().info("Trying to arm the drone")
        else:
            self.get_logger().warn("Arming service not available")


def main(args=None):
    rclpy.init(args=args)
    node = CommNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
