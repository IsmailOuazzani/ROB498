#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
import logging
import time
import math
import numpy as np

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

TOPIC_NAMESPACE = 'rob498_drone_06'
SET_HEIGHT = 1.5

class CommNode(Node):
    def __init__(self):
        super().__init__(TOPIC_NAMESPACE)
        self.srv_launch = self.create_service(Trigger, f'{TOPIC_NAMESPACE}/comm/launch', self.launch_callback)
        self.srv_test = self.create_service(Trigger, f'{TOPIC_NAMESPACE}/comm/test', self.test_callback)
        self.srv_land = self.create_service(Trigger, f'{TOPIC_NAMESPACE}/comm/land', self.land_callback)
        self.srv_abort = self.create_service(Trigger, f'{TOPIC_NAMESPACE}/comm/abort', self.abort_callback)
        self.srv_set_offboard = self.create_service(Trigger, f'{TOPIC_NAMESPACE}/comm/set_offboard', self.set_offboard_callback)
        self.sub_waypoints = self.create_subscription(PoseArray, f'{TOPIC_NAMESPACE}/comm/waypoints', callback_waypoints, 10)
        
        self.should_offboard = False # Only for simulation
        self.should_fly = False
        self.state = State()
        self.pose_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, 10)
        self.subscription = self.create_subscription(PoseStamped, '/vicon/ROB498_Drone/ROB498_Drone', self.vicon_callback,10)
        self.waypoint_received = False
        
        self.pose = PoseStamped()

        # this is needed because there is some kind of ghost publisher that I 
        # can't see when I do ros2 topic info but it's still publishing somehow
        qos_profile = QoSProfile(
            depth=2,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Try changing to RELIABLE if needed
            durability=QoSDurabilityPolicy.VOLATILE
        )

        self.pose_tracker = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.mavros_pose_callback,
            qos_profile
        )


        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.land_client = self.create_client(CommandTOL, '/mavros/cmd/land')

        self.timer = self.create_timer(0.05, self.publish_pose)

        # vicon information
        self.vicon_poses_to_collect = 30
        self.vicon_poses_collected_so_far = 0
        self.vicon_poses = []
        self.start_pose_calculated = False
        self.start_pose = PoseStamped()
        self.start_pose.pose.position.x = 0.0
        self.start_pose.pose.position.y = 0.0
        self.start_pose.pose.position.z = 0.0

        self.target_tracker = TargetTrackerPath(node=self, height = SET_HEIGHT-self.start_pose.pose.position.z)
        self.target_tracker.spawn_waypoint(*self.target_tracker.target_xyz)

    def state_callback(self, msg):
        self.state = msg

    def mavros_pose_callback(self, msg: PoseStamped):
        self.pose = msg

    def callback_waypoints(msg):
        if self.waypoint_received:
            return
        print('Waypoints Received')
        self.waypoint_received = True
        for pose in msg.poses:
            pos = np.array([pose.position.x, pose.position.y, pose.position.z])
            self.waypoints = np.vstack((self.waypoints, pos))
            # planning
    
    def vicon_callback(self, msg):
        if self.vicon_poses_collected_so_far < self.vicon_poses_to_collect:
            self.get_logger().info(f'Collected {self.vicon_poses_collected_so_far}/{self.vicon_poses_to_collect} Vicon poses')
            # append the pose to the list
            self.vicon_poses_collected_so_far += 1
            self.vicon_poses.append(msg)

        elif self.vicon_poses_collected_so_far >= self.vicon_poses_to_collect and not self.start_pose_calculated:
            self.start_pose_calculated = True
            # calculate the average pose
            self.start_pose.pose.position.x = sum([pose.pose.position.x for pose in self.vicon_poses]) / len(self.vicon_poses)
            self.start_pose.pose.position.y = sum([pose.pose.position.y for pose in self.vicon_poses]) / len(self.vicon_poses)
            self.start_pose.pose.position.z = sum([pose.pose.position.z for pose in self.vicon_poses]) / len(self.vicon_poses)


    def launch_callback(self, request, response):
        self.get_logger().info('Launch Requested. Drone taking off.')
        if self.start_pose_calculated:
          response.message = "Launch command executed."
        else:
          response.message = "Vicon poses not collected yet. Lauching without."
        self.should_fly = True
        response.success = True
        return response

    def test_callback(self, request, response):
        self.get_logger().info('Test Requested. Drone performing tasks.')
        response.success = True
        response.message = "Test command executed."
        return response

    def land_callback(self, request, response):
        self.get_logger().info('Land Requested. Drone landing using CommandTOL service.')
        self.should_fly = False
        self.command_land()
        response.success = True
        response.message = "Land command executed."
        return response

    def abort_callback(self, request, response):
        self.get_logger().warning('Abort Requested. Emergency landing using CommandTOL service.')
        self.should_fly = False
        self.command_land()
        response.success = True
        response.message = "Abort command executed."
        return response
    
    def set_offboard_callback(self, request, response):
        self.get_logger().info('Set Offboard Requested. Setting Offboard mode.')
        self.should_offboard = True
        response.success = True
        response.message = "Will attempt to gain offboard mode."
        return response

    def publish_pose(self):
        self.get_logger().info(f"State: {self.state.mode} | Armed: {self.state.armed}  | Should offboard: {self.should_offboard}  | Should fly: {self.should_fly}")
        if self.should_fly:
            self.target_tracker.update_target(self.pose)
            pose = self.target_tracker.target_pose
            self.get_logger().info(f'Target Pose XYZ: ({pose.pose.position.x}, {pose.pose.position.y}, {pose.pose.position.z})')
            self.pose_pub.publish(pose)
            if self.state.mode != "OFFBOARD" and self.should_offboard:
                self.set_offboard_mode()
            if not self.state.armed and self.state.mode == "OFFBOARD" and self.should_fly:
                self.arm_drone()

    def command_land(self):
        self.get_logger().info('Commanding drone to land using CommandTOL service.')
        if self.land_client.wait_for_service(timeout_sec=1.0):
            req = CommandTOL.Request()
            req.min_pitch = 0.0
            req.yaw = 0.0
            req.latitude = 0.0
            req.longitude = 0.0
            req.altitude = 0.0  # Target altitude for landing
            self.land_client.call_async(req)
            self.get_logger().info("Land command sent via CommandTOL service")
        else:
            self.get_logger().warn("CommandTOL service not available")

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

class TargetTrackerPath():
    def __init__(self, node, height):
        # # square path
        # self.waypoints = [
        #     (0, 0, height),
        #     (1, 0, height),
        #     (1, 1, height),
        #     (0, 1, height),
        #     (0, 0, height),
        # ]

        self.waypoints = [(3*(1-math.cos(t)), 3*(math.sin(t)), height) for t in np.linspace(0, 2*np.pi, 40)]

        #TODO: add orientation tracking --- use planner to get the easiest orientations at each waypoint

        self.node = node

        self.cur_waypoint = 0
        self.target_pose = self.get_target_pose()
        self.waiting = False
        self.wait_time = 0.0 # seconds
        self.wait_end = 0
        self.allowed_pose_error = 0.2
        self.get_logger = node.get_logger

        self.spawn_client = node.create_client(SpawnEntity, '/spawn_entity')
        self.delete_client = node.create_client(DeleteEntity, '/delete_entity')

    @property
    def target_xyz(self):
        """Returns a current target pose as tuple (x, y, z)"""
        return self.waypoints[self.cur_waypoint]

    def get_target_pose(self):
        pose = PoseStamped()
        x, y, z = self.target_xyz
        print("xyz", x, y, z, type(x), type(y), type(z))
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        return pose

    def update_target(self, cur_pose):
        if self.cur_waypoint >= len(self.waypoints):
            self.get_logger().info('All waypoints reached, stopping updates.')
            return
        
        if self.waiting:
            self.get_logger().info(f'Waiting for target: {self.cur_waypoint} at {self.target_xyz}')
            
            if time.time() >= self.wait_end:
                # Next target
                self.cur_waypoint += 1
                if self.cur_waypoint >= len(self.waypoints):
                    self.get_logger().info('All waypoints reached, stopping updates.')
                    return -1
                self.target_pose = self.get_target_pose()
                self.waiting = False
                self.delete_waypoint()
                self.spawn_waypoint(*self.target_xyz)
                self.get_logger().info(f'Moving to next waypoint: {self.cur_waypoint} at {self.target_xyz}')
                return
            
            # Check distance from the target pose
            distance = pose_distance(self.target_pose, cur_pose)
            self.get_logger().info(f'Current distance to target: {distance:.2f}, allowed error: {self.allowed_pose_error}')
            
            if distance > self.allowed_pose_error:
                self.wait_end = time.time() + self.wait_time
                self.get_logger().info(f'Resetting wait timer, next wait end: {self.wait_end}')
            
            return

        # Check if within proximity of our target, if so, switch to waiting
        distance = pose_distance(self.target_pose, cur_pose)
        self.get_logger().info(f'Current distance to target: {distance:.2f}, allowed error: {self.allowed_pose_error}')
        if distance < self.allowed_pose_error:
            self.waiting = True
            self.wait_end = time.time() + self.wait_time
            self.get_logger().info(f'Proximity reached, waiting for {self.wait_time} seconds.')

        return


    def spawn_waypoint(self, x, y, z):
        """Spawns a green sphere at the given (x, y, z) position."""

        sdf = '''<sdf version="1.6"> <model name="cur_waypoint"> <static>true</static> <link name="link">
          <visual name="visual">
            <geometry> <sphere><radius>0.3</radius></sphere> </geometry>
            <material> <ambient>0.0 1.0 0.0 1.0</ambient><diffuse>0.0 1.0 0.0 1.0</diffuse> </material>
          </visual>
          <pose>{0} {1} {2} 0 0 0</pose>
          </link> </model> </sdf>'''.format(x, y, z)

        request = SpawnEntity.Request()
        request.name = "cur_waypoint"
        request.xml = sdf
        future = self.spawn_client.call_async(request)
        # rclpy.spin_once(self.node)
        # if future.result():
        #     self.get_logger().info(f'Successfully spawned waypoint at ({x}, {y}, {z})')
        # else:
        #     self.get_logger().error('Failed to spawn waypoint.')

    def delete_waypoint(self):
        """Deletes the entity named 'cur_waypoint'."""
        request = DeleteEntity.Request()
        request.name = "cur_waypoint"
        future = self.delete_client.call_async(request)
        # rclpy.spin_once(self.node, timeout_sec=0.04)
        # if future.result():
        #     self.get_logger().info('Successfully deleted waypoint')
        # else:
        #     self.get_logger().info('Failed to delete waypoint')

# class TargetTrackerTrajectory():

def pose_distance(pose1: PoseStamped, pose2: PoseStamped) -> float:
    return math.sqrt(
        (pose1.pose.position.x - pose2.pose.position.x) ** 2 +
        (pose1.pose.position.y - pose2.pose.position.y) ** 2 +
        (pose1.pose.position.z - pose2.pose.position.z) ** 2
    )

def main(args=None):
    rclpy.init(args=args)
    node = CommNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
