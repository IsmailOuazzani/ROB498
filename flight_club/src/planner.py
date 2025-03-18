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
import numpy as np
from flight_club.msg import TrajectoryPlan
from path_planning_utils.path_generation import initial_guess
from path_planning_utils.plotting import plot_3d_trajectory
TOPIC_NAMESPACE = 'rob498_drone_6'

class Planner(Node):
    def __init__(self):
        super().__init__('planner')

        qos_profile = QoSProfile(
            depth=2,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Try changing to RELIABLE if needed
            durability=QoSDurabilityPolicy.VOLATILE
        )

        # subscriber
        self.pose = None
        self.waypoint_received = False
        self.srv_planned_waypoints = self.create_subscription(PoseArray, f'{TOPIC_NAMESPACE}/comm/waypoints', self.callback_waypoints, 10)
        self.send_waypoints = self.create_publisher(TrajectoryPlan, f'{TOPIC_NAMESPACE}/comm/trajectory', 10)
        self.pose_tracker = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.mavros_pose_callback,
            qos_profile
        )
        self.get_logger().info("Running...")
    
    def mavros_pose_callback(self, msg: PoseStamped):
        self.pose = msg

    def callback_waypoints(self, msg):
        if self.waypoint_received:
            return
        self.get_logger().info('Received waypoints')
        self.waypoint_received = True
        if self.pose is None:
            self.get_logger().info('No pose')
            return
        self.waypoints = np.array([self.pose.pose.position.x, self.pose.pose.position.y, self.pose.pose.position.z])
        for pose in msg.poses:
            pos = np.array([pose.position.x, pose.position.y, pose.position.z])
            self.waypoints = np.vstack((self.waypoints, pos))
        self.get_logger().info(f'Starting to plan')
        # plan the trajectory
        X0_no_tn, tf, N = initial_guess(self.waypoints)
        # publish the trajectory
        self.get_logger().info('Publishing trajectory')
        msg = TrajectoryPlan()
        msg.n_points = N
        msg.tf = tf
        msg.data = X0_no_tn
        self.send_waypoints.publish(msg)
        X = [tf]
        X.extend(X0_no_tn)
        # plot_3d_trajectory(X, self.waypoints)


def main(args=None):
    rclpy.init(args=args)
    node = Planner()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()




        

