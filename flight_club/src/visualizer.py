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
from flight_club.tracker import TargetTrackerPath


TOPIC_NAMESPACE = 'rob498_drone_6'

class TrajectoryMonitor(Node):
    def __init__(self):
        super().__init__('trajectory_monitor')
        self.declare_parameter('enable_rviz', False)
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
            PoseStamped, '/mavros/local_position/pose', self.pose_callback, qos_profile)
        
        self.vel_sub = self.create_subscription(
            TwistStamped, '/mavros/setpoint_velocity/cmd_vel', self.velocity_callback, 10)
        
        if self.enable_rviz:
            self.marker_pub = self.create_publisher(MarkerArray, 'trajectory_markers', 10)

        self.trajectory_history = []
        self.plan_history = []
        self.X = []
        self.waypoint_received = False
        self.trajectory_start_time = None

        self.overall_trajectory_time = 0
        self.target_tracker = TargetTrackerPath(self, (0,0,0))
        self.old_trajectories = []
        self.get_logger().info("Running...")
        # Plotting
        if self.enable_plot:
            self.plot_initialized = True
            self.fig, self.axes = plt.subplots(3, 2, figsize=(10, 15))

            # Position tracking plots
            self.line_plan_x, = self.axes[0, 0].plot([], [], 'r--', label='Planned X')
            self.line_exec_x, = self.axes[0, 0].plot([], [], 'b-', label='Executed X')
            self.axes[0, 0].set_xlabel('Time (s)')
            self.axes[0, 0].set_ylabel('Position X')
            self.axes[0, 0].set_title('Position Tracking - X')
            self.axes[0, 0].legend()

            self.line_plan_y, = self.axes[1, 0].plot([], [], 'r--', label='Planned Y')
            self.line_exec_y, = self.axes[1, 0].plot([], [], 'b-', label='Executed Y')
            self.axes[1, 0].set_xlabel('Time (s)')
            self.axes[1, 0].set_ylabel('Position Y')
            self.axes[1, 0].set_title('Position Tracking - Y')
            self.axes[1, 0].legend()

            self.line_plan_z, = self.axes[2, 0].plot([], [], 'r--', label='Planned Z')
            self.line_exec_z, = self.axes[2, 0].plot([], [], 'b-', label='Executed Z')
            self.axes[2, 0].set_xlabel('Time (s)')
            self.axes[2, 0].set_ylabel('Position Z')
            self.axes[2, 0].set_title('Position Tracking - Z')
            self.axes[2, 0].legend()

            # Velocity tracking plots
            self.line_vel_plan_x, = self.axes[0, 1].plot([], [], 'r--', label='Planned Velocity X')
            self.line_vel_exec_x, = self.axes[0, 1].plot([], [], 'b-', label='Executed Velocity X')
            self.axes[0, 1].set_xlabel('Time (s)')
            self.axes[0, 1].set_ylabel('Velocity X')
            self.axes[0, 1].set_title('Velocity Tracking - X')
            self.axes[0, 1].legend()

            self.line_vel_plan_y, = self.axes[1, 1].plot([], [], 'r--', label='Planned Velocity Y')
            self.line_vel_exec_y, = self.axes[1, 1].plot([], [], 'b-', label='Executed Velocity Y')
            self.axes[1, 1].set_xlabel('Time (s)')
            self.axes[1, 1].set_ylabel('Velocity Y')
            self.axes[1, 1].set_title('Velocity Tracking - Y')
            self.axes[1, 1].legend()

            self.line_vel_plan_z, = self.axes[2, 1].plot([], [], 'r--', label='Planned Velocity Z')
            self.line_vel_exec_z, = self.axes[2, 1].plot([], [], 'b-', label='Executed Velocity Z')
            self.axes[2, 1].set_xlabel('Time (s)')
            self.axes[2, 1].set_ylabel('Velocity Z')
            self.axes[2, 1].set_title('Velocity Tracking - Z')
            self.axes[2, 1].legend()

            plt.tight_layout()
            plt.ion()
            plt.show()

    def update_plot(self):

        traj = np.array(self.trajectory_history)
        t = traj[:, 0]
        x = traj[:, 1]
        y = traj[:, 2]
        z = traj[:, 3]
        vx = traj[:, 4]
        vy = traj[:, 5]
        vz = traj[:, 6]

        self.line_exec_x.set_data(t, x)
        self.line_exec_y.set_data(t, y)
        self.line_exec_z.set_data(t, z)

        self.line_vel_exec_x.set_data(t, vx)
        self.line_vel_exec_y.set_data(t, vy)
        self.line_vel_exec_z.set_data(t, vz)

        if len(self.plan_history) > 0:
            plan = np.array(self.plan_history)
            t_plan = plan[:, 0]
            x_plan = plan[:, 1]
            y_plan = plan[:, 2]
            z_plan = plan[:, 3]
            vx_plan = plan[:, 4]
            vy_plan = plan[:, 5]
            vz_plan = plan[:, 6]

            self.line_plan_x.set_data(t_plan, x_plan)
            self.line_plan_y.set_data(t_plan, y_plan)
            self.line_plan_z.set_data(t_plan, z_plan)

            self.line_vel_plan_x.set_data(t_plan, vx_plan)
            self.line_vel_plan_y.set_data(t_plan, vy_plan)
            self.line_vel_plan_z.set_data(t_plan, vz_plan)            

        for ax in self.axes.flatten():
            ax.relim()
            ax.autoscale_view()

        plt.pause(0.001)

    def waypoints_callback(self, msg):
      
        self.get_logger().info('Received plan')
        if not self.waypoint_received:
            self.overall_trajectory_time = time.time()
        N = msg.n_points
        tf = msg.tf
        X0_no_tn = msg.data
        X = [tf]
        X.extend(X0_no_tn)    
        self.X = X    
        
        qs, qs_dots, us = decompose_X(X, 3, 9)
        qs, qs_dots, us = qs.T, qs_dots.T, us.T
        self.qs_wpt = qs
        self.qs_dots = qs_dots
        self.get_logger().info(f'Waypoints received: {N}')

        # Start the execution time
        self.waypoint_received = True
        self.target_tracker.qs = qs
        self.target_tracker.qs_dots = qs_dots
        self.target_tracker.N = N
        self.target_tracker.tf = tf
        self.trajectory_start_time = time.time()

        # Retain RViz waypoint publishing
        # if self.enable_rviz:
        #     self.publish_waypoints_rviz()

    def pose_callback(self, msg):
        if not self.waypoint_received:
            return

        t = time.time() - self.trajectory_start_time
        plotting_time = time.time() - self.overall_trajectory_time
        qs, qdots = self.target_tracker.interpolate(t)

        if len(self.trajectory_history) > 0:
            prev_x = self.trajectory_history[-1][1]
            prev_y = self.trajectory_history[-1][2]
            prev_z = self.trajectory_history[-1][3]
            prev_t = self.trajectory_history[-1][0]
            cur_vel = [(msg.pose.position.x - prev_x) / (plotting_time - prev_t), 
                    (msg.pose.position.y - prev_y) / (plotting_time - prev_t), 
                    (msg.pose.position.z - prev_z) / (plotting_time - prev_t)]
        else:
            cur_vel = [0, 0, 0]

        self.trajectory_history.append([plotting_time, msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, cur_vel[0], cur_vel[1], cur_vel[2]])
        self.plan_history.append([plotting_time, qs[0], qs[1], qs[2], qdots[0], qdots[1], qdots[2]])
        self.update_plot()

    def velocity_callback(self, msg):
        pass
        # self.get_logger().info(
        #     f"Velocity | x: {msg.twist.linear.x:.3f}, y: {msg.twist.linear.y:.3f}, z: {msg.twist.linear.z:.3f}"
        # )
        

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



    # def shutdown_callback(self):
    #     if self.enable_plot:
    #         self.plot_trajectory()
    #     self.get_logger().info("Shutting down trajectory monitor.")


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
