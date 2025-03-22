#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Header
from flight_club.msg import TrajectoryPlan
import matplotlib.pyplot as plt
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
import numpy as np
import time
from path_planning_utils.path_generation import initial_guess
from path_planning_utils.plotting import decompose_X, plan_vs_execute


TOPIC_NAMESPACE = 'rob498_drone_6'

class TrajectoryMonitor(Node):
    def __init__(self):
        super().__init__('trajectory_monitor')
        self.declare_parameter('enable_rviz', True)
        self.declare_parameter('enable_plot', True)
        
        self.enable_rviz = self.get_parameter('enable_rviz').value
        self.enable_plot = self.get_parameter('enable_plot').value

        qos_profile = QoSProfile(
            depth=2,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Try changing to RELIABLE if needed
            durability=QoSDurabilityPolicy.VOLATILE
        )

        self.trajectory_sub = self.create_subscription(
            TrajectoryPlan, f'{TOPIC_NAMESPACE}/comm/trajectory', self.waypoints_callback, 10)

        self.pose_sub = self.create_subscription(
            PoseStamped, '/mavros/vision_pose/pose', self.pose_callback, qos_profile)
        
        self.vel_sub = self.create_subscription(
            TwistStamped, '/mavros/setpoint_velocity/cmd_vel', self.velocity_callback, 10)
        
        if self.enable_rviz:
            self.marker_pub = self.create_publisher(MarkerArray, 'trajectory_markers', 10)

        self.trajectory_history = []
        self.X = []
        self.waypoint_received = True
        self.trajectory_start_time = 0.0 #None
        self.get_logger().info("Running...")

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
        qs, qs_dots, us = qs.T, qs_dots.T, us.T
        self.qs_wpt = qs
        self.get_logger().info(f'Waypoints received: {N}')
        
        # self.target_tracker.qs = qs
        # self.target_tracker.qs_dots = qs_dots
        # self.target_tracker.N = N
        # self.target_tracker.tf = tf

        # Start the execution time
        self.trajectory_start_time = time.time()

        # Retain RViz waypoint publishing
        if self.enable_rviz:
            self.publish_waypoints_rviz()

    def pose_callback(self, msg):
        if not self.waypoint_received:
            return

        t = time.time() - self.trajectory_start_time

        if len(self.trajectory_history) > 0:
            prev_x = self.trajectory_history[-1][1]
            prev_y = self.trajectory_history[-1][2]
            prev_z = self.trajectory_history[-1][3]
            prev_t = self.trajectory_history[-1][0]
            cur_vel = [(msg.pose.position.x - prev_x) / (t - prev_t), 
                    (msg.pose.position.y - prev_y) / (t - prev_t), 
                    (msg.pose.position.z - prev_z) / (t - prev_t)]
        else:
            cur_vel = [0, 0, 0]

        self.trajectory_history.append([t, msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, cur_vel[0], cur_vel[1], cur_vel[2]])


    def velocity_callback(self, msg):
        self.get_logger().info(
            f"Velocity | x: {msg.twist.linear.x:.3f}, y: {msg.twist.linear.y:.3f}, z: {msg.twist.linear.z:.3f}"
        )

    def publish_waypoints_rviz(self):
        marker_array = MarkerArray()
        for i in range(0, len(self.X) - 1, 3):
            marker = Marker()
            marker.header = Header()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = "map"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = self.X[i + 1]
            marker.pose.position.y = self.X[i + 2]
            marker.pose.position.z = self.X[i + 3]
            marker.scale.x = marker.scale.y = marker.scale.z = 0.1
            marker.color.r = 1.0
            marker.color.a = 1.0
            marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)

    def plot_trajectory(self):
        self.get_logger().info("plotting...")
        if len(self.trajectory_history) == 0:
            return
        
        traj = np.array(self.trajectory_history)
        plt.figure()
        plt.plot(traj[:, 1], traj[:, 2], label='Executed')
        if len(self.X) > 0:
            x = [float(point[0]) for point in self.qs_wpt]
            y = [float(point[1]) for point in self.qs_wpt]
            plt.scatter(x, y, color='red', label='Planned Waypoints')
        plt.xlabel('X Position')
        plt.ylabel('Y Position')
        plt.legend()
        plt.title('Executed vs Planned Trajectory')
        plt.show()
        plan_vs_execute(self.X, self.trajectory_history)

    def shutdown_callback(self):
        if self.enable_plot:
            self.plot_trajectory()
        self.get_logger().info("Shutting down trajectory monitor.")


def main(args=None):
    rclpy.init(args=args)
    monitor = TrajectoryMonitor()
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.shutdown_callback()
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
