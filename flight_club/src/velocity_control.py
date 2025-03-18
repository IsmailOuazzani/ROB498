#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
from geometry_msgs.msg import PoseStamped, Pose, PoseArray, TwistStamped
from flight_club.msg import TrajectoryPlan
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
import logging
import time
import math
import numpy as np
from path_planning_utils.path_generation import initial_guess
from path_planning_utils.plotting import decompose_X, plan_vs_execute


logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

TOPIC_NAMESPACE = 'rob498_drone_6'
SET_HEIGHT = 1.5

class CommNode(Node):
    def __init__(self):
        super().__init__(TOPIC_NAMESPACE)
        self.srv_launch = self.create_service(Trigger, f'{TOPIC_NAMESPACE}/comm/launch', self.launch_callback)
        self.srv_test = self.create_service(Trigger, f'{TOPIC_NAMESPACE}/comm/test', self.test_callback)
        self.srv_land = self.create_service(Trigger, f'{TOPIC_NAMESPACE}/comm/land', self.land_callback)
        self.srv_abort = self.create_service(Trigger, f'{TOPIC_NAMESPACE}/comm/abort', self.abort_callback)
        self.srv_set_offboard = self.create_service(Trigger, f'{TOPIC_NAMESPACE}/comm/set_offboard', self.set_offboard_callback)
        self.planned_waypoints = self.create_subscription(TrajectoryPlan, f'{TOPIC_NAMESPACE}/comm/trajectory', self.waypoints_callback, 10)
        
        self.should_offboard = False # Only for simulation
        self.should_fly = False
        self.state = State()
        self.pose_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.vel_pub = self.create_publisher(TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10)
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, 10)
        self.waypoint_received = False
        self.trajectory_ended = False
        
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

        self.target_tracker = TargetTrackerPath(node=self, height = SET_HEIGHT)
        self.trajectory_history = []

    def state_callback(self, msg):
        self.state = msg

    def mavros_pose_callback(self, msg: PoseStamped):
        self.pose = msg

    def waypoints_callback(self, msg):
        if self.waypoint_received:
            return
        self.get_logger().info('Received plan')
        self.waypoint_received = True
        N = msg.n_points
        tf = msg.tf
        X0_no_tn = msg.data
        X = [tf]
        X.extend(X0_no_tn)    
        self.X = X    
        qs, qs_dots, us = decompose_X(X, 3, 9)
        # make sure the tracker is aware of the new waypoints
        qs, qs_dots, us = qs.T, qs_dots.T, us.T
        self.get_logger().info(f'Waypoints received: {N}')
        self.target_tracker.qs = qs
        self.target_tracker.qs_dots = qs_dots
        self.target_tracker.N = N
        self.target_tracker.tf = tf
        # start the execution time
        self.trajectory_start_time = time.time()


    def launch_callback(self, request, response):
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
        # self.get_logger().    info(f"State: {self.state.mode} | Armed: {self.state.armed}  | Should offboard: {self.should_offboard}  | Should fly: {self.should_fly}")
        if self.should_fly:
            if self.waypoint_received:
                t = time.time() - self.trajectory_start_time
                qs, qs_dots = self.target_tracker.interpolate(t)
                pose = PoseStamped()
                vel_topic = TwistStamped()
                pose.header.stamp = self.get_clock().now().to_msg()
                vel_topic.header.stamp = self.get_clock().now().to_msg()

                # inflate measured errors from trajectory to artificially increase aggressiveness of tracking 
                pose.pose.position.x = qs[0] + (qs[0] - self.pose.pose.position.x) * 0.2
                pose.pose.position.y = qs[1] + (qs[1] - self.pose.pose.position.y) * 0.2
                pose.pose.position.z = qs[2] + (qs[2] - self.pose.pose.position.z) * 0.8
                if t < self.target_tracker.tf: 
                    if len(self.trajectory_history) > 0:
                        prev_x = self.trajectory_history[-1][1]
                        prev_y = self.trajectory_history[-1][2]
                        prev_z = self.trajectory_history[-1][3]
                        prev_t = self.trajectory_history[-1][0]
                        cur_vel = [(self.pose.pose.position.x - prev_x)/(t - prev_t), (self.pose.pose.position.y - prev_y)/(t - prev_t), (self.pose.pose.position.z - prev_z)/(t - prev_t)]
                    else:
                        
                        cur_vel = [0, 0, 0]
                    self.trajectory_history.append([t, self.pose.pose.position.x, self.pose.pose.position.y, self.pose.pose.position.z, cur_vel[0], cur_vel[1], cur_vel[2]])
                    vel_topic.twist.linear.x = qs_dots[0]
                    vel_topic.twist.linear.y = qs_dots[1]
                    vel_topic.twist.linear.z = qs_dots[2]
                    self.vel_pub.publish(vel_topic)
                else:
                    #plot the data
                    plan_vs_execute(self.X, self.trajectory_history)


                self.pose_pub.publish(pose)

            
            else:
                point = self.target_tracker.qs[0]
                pose = PoseStamped()
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.position.x = float(point[0])
                pose.pose.position.y = float(point[1])
                pose.pose.position.z = float(point[2])

                # self.get_logger().info(f'Target Pose XYZ: ({pose.pose.position.x}, {pose.pose.position.y}, {pose.pose.position.z})')
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
        
        self.qs = [(0, 0, SET_HEIGHT)]
        self.qs_dots = [[0,0,0]]
        self.N = 0
        self.tf = 0
        
        self.node = node

        self.cur_waypoint = 0
        self.waiting = False
        self.wait_time = 0.0 # seconds
        self.wait_end = 0
        self.allowed_pose_error = 0.2
        self.get_logger = node.get_logger


    def interpolate(self, t):
        offset=0
        self.dt = self.tf/self.N
        if t > self.tf:
            return self.qs[-1], None
        # Interpolate
        lower_index = int(t/self.dt)
        lower_index_time = int(t/self.dt)
        upper_index = lower_index_time + 1
        if lower_index_time >= (self.N-1):
            return self.qs[self.N-1], self.qs_dots[self.N-1]
        if upper_index >= (self.N-1):
            return self.qs[lower_index], self.qs_dots[lower_index]
        lower_time = lower_index_time*self.dt
        upper_time = (lower_index_time +1)*self.dt
        
        # look ahead to account for lagging controller, should be less for aggressive motions!!! TODO: adjust as needed
        # velocity look ahead
        lower_index = min(self.N-1, lower_index+int(0.2/self.dt))
        upper_index = min(self.N-1, upper_index+int(0.2/self.dt))
        q_dot = self.qs_dots[lower_index] + (self.qs_dots[upper_index] - self.qs_dots[lower_index])*(t - lower_time)/(upper_time - lower_time)

        # position look ahead
        lower_index = min(self.N-1, lower_index+int(0.4/self.dt))
        upper_index = min(self.N-1, upper_index+int(0.4/self.dt))

        q = self.qs[lower_index] + (self.qs[upper_index] - self.qs[lower_index])*(t - lower_time)/(upper_time - lower_time)
        return q, q_dot

def main(args=None):
    rclpy.init(args=args)
    node = CommNode()
    node.get_logger().info("Running...")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
