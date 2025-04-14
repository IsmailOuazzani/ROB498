#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from pynput import keyboard
import numpy as np
import os
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from enum import Enum, auto


from flight_club.msg import TrajectoryPlan, GameInfo
from path_planning_utils.path_generation import initial_guess


TOPIC_NAMESPACE = 'rob498_drone_6'


class RobotState(Enum):
    INIALIZING = auto()
    INITIALIZED = auto()
    EXPECT_MISSION = auto()
    MISSION = auto()
    SEEKER_LOOKING = auto()
    IDLE = auto()
    ABORT = auto()


class SequenceTimerNode(Node):
    def __init__(self):
        super().__init__('sequence_timer')
        self.get_logger().info("Press 1 to start the sequence timer (then 2, 3, space in order)")
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        # Key sequence we're looking for (after '1')
        self.expected_keys = ['2', '3', 'space']
        self.pressed_keys = []

        self.declare_parameter('waypoints', '/src/ros_ws/src/drone_packages/output')  # Default file for waypoints
        self.declare_parameter('occluded_region', '/src/ros_ws/src/drone_packages/output')  # Default file for occluded points
        self.declare_parameter('obstacles', '/src/ros_ws/src/drone_packages/output')  # Default file for obstacles
        self.waypoints = self.get_parameter('waypoints').get_parameter_value().string_value
        self.occluded_region = self.get_parameter('occluded_region').get_parameter_value().string_value
        self.obstacles = self.get_parameter('obstacles').get_parameter_value().string_value
        self.waypoints = self.load_waypoints()
        self.occluded = self.load_occluded()
        self.obstacles = self.load_obstacles()
        self.current_index = 0
        self.trajectory_time_start = None
        self.game_state = GameInfo.GAME_STATE_STOP
        self.distance_to_waypoint = []


        self.start_time = None
        self.timestamps = []  # Elapsed times
        self.predictions = []  # Predictions of when 'space' would be pressed
        self.target = None

        self.seeker_is_looking = False
        self.just_switched = False
        self.robot_state = RobotState.INIALIZING
        self.prev_game_state = GameInfo.GAME_STATE_STOP

        qos_profile = QoSProfile(
            depth=2,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Try changing to RELIABLE if needed
            durability=QoSDurabilityPolicy.VOLATILE
        )
        self.send_waypoints = self.create_publisher(TrajectoryPlan, f'{TOPIC_NAMESPACE}/comm/trajectory', 10)
        self.timer = self.create_timer(0.1, self.check_seeker_state)
        self.pose_tracker = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.mavros_pose_callback,
            qos_profile
        )
        self.game_info_sub = self.create_subscription(
            GameInfo,
            "/flight_club/game_info",
            self.game_info_callback,
            10
        )
        self.get_logger().info("Running...")

    def mavros_pose_callback(self, msg: PoseStamped):
        self.pose = msg
        # self.get_logger().info(f"Pose: {self.pose.pose.position.x}, {self.pose.pose.position.y}, {self.pose.pose.position.z}")

    def game_info_callback(self, msg: GameInfo):
        # Check if the state has changed
        # self.get_logger().info(f"Game state: {msg.game_state}")
        self.game_state = msg.game_state
        if msg.game_state != self.prev_game_state:
            self.get_logger().info(f"Game state changed from {self.prev_game_state} to {msg.game_state}")
            self.prev_game_state = msg.game_state

            now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


            if msg.game_state == GameInfo.GAME_STATE_BLIND_1:
                # Reset everything
                self.start_time = now
                self.pressed_keys = []
                self.timestamps = [0.0]  # Start time
                self.predictions = []
                self.get_logger().info("Key '1' pressed. Timer started.")
                return
            elif msg.game_state == GameInfo.GAME_STATE_BLIND_INDEF:
                self.predictions = []
                self.get_logger().info("Key 'space' pressed. Seeker is NOT looking.")
                self.just_switched = True
                if self.robot_state == RobotState.INITIALIZED or self.robot_state == RobotState.IDLE:
                    self.robot_state = RobotState.EXPECT_MISSION
            elif msg.game_state == GameInfo.GAME_STATE_BLIND_2 or msg.game_state == GameInfo.GAME_STATE_BLIND_3:
                elapsed = (now - self.start_time)
                self.timestamps.append(elapsed)
                self.linear_predict()
            elif msg.game_state == GameInfo.GAME_STATE_SEEKING:
                self.get_logger().info("Seeker is looking.")


    def linear_predict(self):
        n = len(self.timestamps)
        x = np.arange(1, n + 1)
        y = np.array(self.timestamps)

        if len(x) < 2:
            return

        # Fit a line: y = mx + b
        m, b = np.polyfit(x, y, 1)

        # Predict for x = 4 (i.e. the 4th key = 'space')
        predicted_time = m * 4 + b
        self.predictions.append(predicted_time)

        self.get_logger().info(f"Estimated time for 'space' press: +{predicted_time:.3f} seconds")

    def show_prediction_summary(self):
        actual_time = self.timestamps[-1]
        self.get_logger().info("\n=== Prediction Summary ===")
        for i, pred in enumerate(self.predictions, start=1):
            error = abs(pred - actual_time)
            self.get_logger().info(
                f"[After key {i+1}] Predicted: {pred:.3f}s | Actual (space): {actual_time:.3f}s | Error: {error:.3f}s"
            )
        self.get_logger().info("===========================")

    def load_waypoints(self):
        # Load waypoints from the waypoint.npy file
        waypoint_file = self.waypoints
        if not os.path.exists(waypoint_file):
            self.get_logger().error(f"Waypoint file not found: {waypoint_file}")
            return None

        try:
            positions = np.load(waypoint_file)
            self.get_logger().info(f"Loaded {len(positions)} waypoints")
        except Exception as e:
            self.get_logger().error(f"Error loading waypoint file: {e}")
            return None

        waypoints = PoseArray()
        for pos in positions:
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = pos
            waypoints.poses.append(pose)
        return waypoints
    
    def load_occluded(self):
        # Load waypoints from the waypoint.npy file
        waypoint_file = self.occluded_region
        if not os.path.exists(waypoint_file):
            self.get_logger().error(f"Waypoint file not found: {waypoint_file}")
            return None

        try:
            positions = np.load(waypoint_file)
            self.get_logger().info(f"Loaded {len(positions)} out of collision points")
            return positions
        except Exception as e:
            self.get_logger().error(f"Error loading waypoint file: {e}")
            return None

    def load_obstacles(self):
        # Load waypoints from the waypoint.npy file
        waypoint_file = self.obstacles
        if not os.path.exists(waypoint_file):
            self.get_logger().error(f"Waypoint file not found: {waypoint_file}")
            return None

        try:
            positions = np.load(waypoint_file)
            self.get_logger().info(f"Loaded {len(positions)} out of collision points")
            return positions
        except Exception as e:
            self.get_logger().error(f"Error loading waypoint file: {e}")
            return None

    def check_seeker_state(self):
        # This runs at 10 Hz
        # self.get_logger().debug(f"Current robot state: {self.robot_state}")
        # as soon as the switch is detected plan route to the next waypoint. When the space bar is in sight start thinking about what to do next
        self.get_logger().debug(f"Current robot state: {self.robot_state}")
        if self.robot_state == RobotState.INIALIZING and self.game_state == GameInfo.GAME_STATE_BLIND_INDEF:
            self.get_logger().info("Initializing...")
            if self.trajectory_time_start is None:
                self.target = self.waypoints.poses[self.current_index]
                self.end_trajectory_target_time = self.plan_and_publish(self.target)
                self.current_index += 1
                self.just_switched = False
                self.trajectory_time_start = self.get_clock().now()
            else:
                self.check_idle()
        elif self.robot_state == RobotState.EXPECT_MISSION:
            self.target = self.waypoints.poses[self.current_index]
            self.end_trajectory_target_time = self.plan_and_publish(self.target)
            self.current_index += 1
            self.just_switched = False
            self.trajectory_time_start = self.get_clock().now()
            self.robot_state = RobotState.MISSION
        elif self.robot_state == RobotState.MISSION:
            self.check_idle()
            avg_prediction = np.mean(self.predictions)
            if len(self.predictions) > 0:
                current_time = (self.get_clock().now() - self.trajectory_time_start).nanoseconds / 1e9
                time_to_completion = self.end_trajectory_target_time - current_time
                self.get_logger().info(f"Time to completion: {time_to_completion:.3f}s | Avg prediction: {avg_prediction:.3f}s")

                if time_to_completion > avg_prediction:
                    self.get_logger().info("Finding closest not occluded point")
                    if self.occluded is not None and self.obstacles is not None:
                        current_position = np.array([
                            self.pose.pose.position.x,
                            self.pose.pose.position.y,
                            self.pose.pose.position.z
                        ])
                        distances = np.linalg.norm(self.occluded - current_position, axis=1)
                        sorted_indices = np.argsort(distances)

                        for closest_index in sorted_indices:
                            closest_point = self.occluded[closest_index]
                            self.get_logger().debug(f"Checking point: {closest_point}")

                            # Vector from current position to closest point
                            line_vector = closest_point - current_position
                            line_length = np.linalg.norm(line_vector)
                            line_unit_vector = line_vector / line_length

                            # Calculate distances of obstacles to the line segment
                            obstacle_positions = np.array(self.obstacles)
                            point_vectors = obstacle_positions - current_position
                            projections = np.dot(point_vectors, line_unit_vector)
                            closest_points_on_line = current_position + np.clip(projections, 0, line_length)[:, None] * line_unit_vector
                            distances_to_line = np.linalg.norm(obstacle_positions - closest_points_on_line, axis=1)

                            # Check if any obstacle is too close
                            if np.all(distances_to_line >= 0.5):  # Threshold for safety
                                self.get_logger().info(f"Selected safe point: {closest_point}")
                                # Plan and publish trajectory to the closest safe point
                                pose = Pose()
                                pose.position.x, pose.position.y, pose.position.z = closest_point
                                self.target = pose
                                _ = self.plan_and_publish(self.target)
                                self.current_index -= 1
                                self.robot_state = RobotState.ABORT
                                break
                        else:
                            self.get_logger().error("No safe point found!") 
                
        elif self.robot_state == RobotState.ABORT:
            self.check_idle()
        elif self.robot_state == self.robot_state.IDLE and self.game_state == GameInfo.GAME_STATE_BLIND_INDEF:
            self.robot_state = RobotState.EXPECT_MISSION

    def check_idle(self):
        distance_to_waypoint = np.linalg.norm(
            np.array([self.pose.pose.position.x, self.pose.pose.position.y, self.pose.pose.position.z]) -
            np.array([self.target.position.x, self.target.position.y, self.target.position.z])
        )
        self.get_logger().debug(f"Distance to waypoint: {distance_to_waypoint:.3f}")
        self.distance_to_waypoint.append(distance_to_waypoint)
        if len(self.distance_to_waypoint) > 10:
            self.distance_to_waypoint.pop(0)
            if all(d < 0.2 for d in self.distance_to_waypoint):
                self.get_logger().info(f"Reached target waypoint: {self.target.position.x}, {self.target.position.y}, {self.target.position.z}")
                self.get_logger().info(f"Distance to waypoint: {distance_to_waypoint:.3f}")
                if self.robot_state == RobotState.INIALIZING:
                    self.robot_state = RobotState.INITIALIZED
                else:
                    self.robot_state = RobotState.IDLE

    def plan_and_publish(self, next_pos):
        waypoints = np.array([self.pose.pose.position.x, self.pose.pose.position.y, self.pose.pose.position.z])
        waypoints = np.vstack((waypoints, [next_pos.position.x, next_pos.position.y, next_pos.position.z]))
        self.get_logger().info(f'Starting to plan')
        # plan the trajectory
        X0_no_tn, tf, N = initial_guess(waypoints)
        # publish the trajectory
        self.get_logger().info('Publishing trajectory')
        msg = TrajectoryPlan()
        msg.n_points = N
        msg.tf = tf
        msg.data = X0_no_tn
        self.send_waypoints.publish(msg)
        return tf + 1.5  # Add a buffer time for the trajectory to be executed

def main(args=None):
    rclpy.init(args=args)
    node = SequenceTimerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
