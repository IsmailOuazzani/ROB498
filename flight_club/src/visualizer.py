#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from visualization_msgs.msg import MarkerArray
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
import numpy as np
import time
from mpl_toolkits.mplot3d import Axes3D

TOPIC_NAMESPACE = 'rob498_drone_6'

class TrajectoryMonitor(Node):
    def __init__(self):
        super().__init__('trajectory_monitor')
        self.declare_parameter('enable_rviz', True)
        self.declare_parameter('enable_plot', True)
        
        self.enable_rviz = self.get_parameter('enable_rviz').value
        self.enable_plot = self.get_parameter('enable_plot').value

        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT)

        # Subscribe to the additional topics
        self.pose_sub_vicon = self.create_subscription(
            PoseStamped, '/vicon/ROB498_Drone/ROB498_Drone', self.pose_callback_vicon, 10
        )
        self.pose_sub_camera = self.create_subscription(
            Odometry, '/camera/pose/sample', self.pose_callback_camera, qos_profile
        )
        self.pose_sub_mavros = self.create_subscription(
            PoseStamped, '/mavros/vision_pose/pose', self.pose_callback_mavros, 10
        )

        if self.enable_rviz:
            self.marker_pub = self.create_publisher(MarkerArray, 'trajectory_markers', 10)

        self.pose_history_vicon = []
        self.pose_history_camera = []
        self.pose_history_mavros = []

        self.get_logger().info("Running...")

    def pose_callback_vicon(self, msg):
        # Record the pose from the Vicon system
        t = time.time()
        self.pose_history_vicon.append([t, msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        
    def pose_callback_camera(self, msg):
        # Record the pose from the camera
        t = time.time()
        self.pose_history_camera.append([t, msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])

    def pose_callback_mavros(self, msg):
        # Record the pose from MAVROS
        t = time.time()
        self.pose_history_mavros.append([t, msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

    def plot_trajectory(self):
        self.get_logger().info("Plotting...")

        if len(self.pose_history_vicon) == 0 and len(self.pose_history_camera) == 0 and len(self.pose_history_mavros) == 0:
            return
        
        # Convert to numpy arrays for easier manipulation
        traj_vicon = np.array(self.pose_history_vicon)
        traj_camera = np.array(self.pose_history_camera)
        traj_mavros = np.array(self.pose_history_mavros)
        
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        print(f"{traj_vicon.shape}, {traj_camera.shape},{traj_mavros.shape}")
        # Plot the 3D positions for each source
        ax.plot(traj_vicon[:, 1], traj_vicon[:, 2], traj_vicon[:, 3], label='Vicon', color='blue')
        ax.plot(traj_camera[:, 1], traj_camera[:, 2], traj_camera[:, 3], label='Camera', color='green')
        ax.plot(traj_mavros[:, 1], traj_mavros[:, 2], traj_mavros[:, 3], label='MAVROS', color='red')

        ax.set_xlabel('X Position')
        ax.set_ylabel('Y Position')
        ax.set_zlabel('Z Position')
        ax.legend()

        plt.title('3D Trajectories from Different Sources')
        plt.show()

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

