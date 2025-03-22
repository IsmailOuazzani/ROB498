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
from scipy.spatial.transform import Rotation as R

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
        self.pose_history_vicon.append([t, msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, 
                                        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        
    def pose_callback_camera(self, msg):
        # Record the pose from the camera
        t = time.time()
        self.pose_history_camera.append([t, msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z, 
                                        msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        
    def pose_callback_mavros(self, msg):
        # Record the pose from MAVROS
        t = time.time()
        self.pose_history_mavros.append([t, msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, 
                                        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        
    def plot_trajectory(self):
        self.get_logger().info("Plotting...")

        if len(self.pose_history_vicon) == 0 and len(self.pose_history_camera) == 0 and len(self.pose_history_mavros) == 0:
            return
        
        # Convert to numpy arrays for easier manipulation
        traj_vicon = np.array(self.pose_history_vicon)
        traj_camera = np.array(self.pose_history_camera)
        traj_mavros = np.array(self.pose_history_mavros)

        rpy_vicon = R.from_quat(traj_vicon[:, 4:]).as_euler('xyz', degrees=False)
        rpy_camera = R.from_quat(traj_camera[:, 4:]).as_euler('xyz', degrees=False)
        rpy_mavros = R.from_quat(traj_mavros[:, 4:]).as_euler('xyz', degrees=False)
        
        fig = plt.figure(figsize=(20, 15))

        # Left plot (3D trajectory)
        ax3d = fig.add_subplot(121, projection='3d')
        ax3d.plot(traj_vicon[:, 1], traj_vicon[:, 2], traj_vicon[:, 3], label='Vicon', color='blue')
        ax3d.plot(traj_camera[:, 1], traj_camera[:, 2], traj_camera[:, 3], label='Camera', color='green')
        ax3d.plot(traj_mavros[:, 1], traj_mavros[:, 2], traj_mavros[:, 3], label='MAVROS', color='red')
        ax3d.scatter(traj_mavros[:, 1], traj_mavros[:, 2], traj_mavros[:, 3], label='MAVROS', color='pink')

        ax3d.set_xlabel('X Position')
        ax3d.set_ylabel('Y Position')
        ax3d.set_zlabel('Z Position')
        ax3d.legend()
        ax3d.set_title('3D Trajectories from Different Sources')

        # Right plots (Roll, Pitch, Yaw stacked vertically)
        # Roll
        ax_roll = fig.add_subplot(322)
        ax_roll.plot(traj_vicon[:, 0], rpy_vicon[:, 0], label='Vicon Roll', color='blue')
        ax_roll.plot(traj_camera[:, 0], rpy_camera[:, 0], label='Camera Roll', color='green')
        ax_roll.plot(traj_mavros[:, 0], rpy_mavros[:, 0], label='MAVROS Roll', color='red')
        ax_roll.set_ylabel('Roll (radians)')
        ax_roll.set_title('Roll Comparison')
        ax_roll.legend()

        # Pitch
        ax_pitch = fig.add_subplot(324)
        ax_pitch.plot(traj_vicon[:, 0], rpy_vicon[:, 1], label='Vicon Pitch', color='blue')
        ax_pitch.plot(traj_camera[:, 0], rpy_camera[:, 1], label='Camera Pitch', color='green')
        ax_pitch.plot(traj_mavros[:, 0], rpy_mavros[:, 1], label='MAVROS Pitch', color='red')
        ax_pitch.set_ylabel('Pitch (radians)')
        ax_pitch.set_title('Pitch Comparison')
        ax_pitch.legend()

        # Yaw
        ax_yaw = fig.add_subplot(326)
        ax_yaw.plot(traj_vicon[:, 0], rpy_vicon[:, 2], label='Vicon Yaw', color='blue')
        ax_yaw.plot(traj_camera[:, 0], rpy_camera[:, 2], label='Camera Yaw', color='green')
        ax_yaw.plot(traj_mavros[:, 0], rpy_mavros[:, 2], label='MAVROS Yaw', color='red')
        ax_yaw.set_xlabel('Time (s)')
        ax_yaw.set_ylabel('Yaw (radians)')
        ax_yaw.set_title('Yaw Comparison')
        ax_yaw.legend()

        # Adjust layout
        fig.tight_layout()
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

