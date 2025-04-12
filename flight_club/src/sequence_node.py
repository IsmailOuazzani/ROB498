#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from pynput import keyboard
import numpy as np
import os
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from enum import Enum, auto


from flight_club.msg import TrajectoryPlan
from path_planning_utils.path_generation import initial_guess


TOPIC_NAMESPACE = 'rob498_drone_6'


class RobotState(Enum):
    INIALIZING = auto()
    EXPECT_MISSION = auto()
    MISSION = auto()
    SEEKER_LOOKING = auto()
    ABORT = auto()


class SequenceTimerNode(Node):
    def __init__(self):
        super().__init__('sequence_timer')
        self.get_logger().info("Press 1 to start the sequence timer (then 2, 3, space in order)")

        # Key sequence we're looking for (after '1')
        self.expected_keys = ['2', '3', 'space']
        self.pressed_keys = []

        self.declare_parameter('output_folder', '/tmp')
        self.declare_parameter('occluded_folder', '/tmp')
        self.output_folder = self.get_parameter('output_folder').get_parameter_value().string_value
        self.occluded_folder = self.get_parameter('occluded_folder').get_parameter_value().string_value
        # Load waypoints and initialize index
        self.waypoints = self.load_waypoints()
        self.out_of_collision = self.load_occluded()
        self.current_index = 0


        self.start_time = None
        self.timestamps = []  # Elapsed times
        self.predictions = []  # Predictions of when 'space' would be pressed

        self.seeker_is_looking = False
        self.just_switched = False
        self.robot_state = RobotState.INIALIZING

        qos_profile = QoSProfile(
            depth=2,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Try changing to RELIABLE if needed
            durability=QoSDurabilityPolicy.VOLATILE
        )
        self.send_waypoints = self.create_publisher(TrajectoryPlan, f'{TOPIC_NAMESPACE}/comm/trajectory', 10)
        self.listener = keyboard.Listener(on_press=self.on_key_press)
        self.timer = self.create_timer(0.1, self.check_seeker_state)
        self.pose_tracker = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.mavros_pose_callback,
            qos_profile
        )

        self.listener.start()
        self.get_logger().info("Running...")

    def mavros_pose_callback(self, msg: PoseStamped):
        self.pose = msg
        # self.get_logger().info(f"Pose: {self.pose.pose.position.x}, {self.pose.pose.position.y}, {self.pose.pose.position.z}")

    def on_key_press(self, key):
        try:
            key_str = key.char
        except AttributeError:
            key_str = 'space' if key == keyboard.Key.space else None

        if key_str is None:
            return

        now = self.get_clock().now()


        if key_str == '1' and not self.seeker_is_looking:
            # Reset everything
            self.start_time = now
            self.pressed_keys = []
            self.timestamps = [0.0]  # Start time
            self.predictions = []
            self.get_logger().info("Key '1' pressed. Timer started.")
            return
        elif key_str == 'space' and self.seeker_is_looking:
            self.predictions = []
            self.seeker_is_looking = False
            self.get_logger().info("Key 'space' pressed. Seeker is NOT looking.")
            self.just_switched = True
            self.robot_state = RobotState.EXPECT_MISSION

        if self.start_time is None:
            return  # Wait until '1' is pressed

        if len(self.pressed_keys) < len(self.expected_keys):
            expected_key = self.expected_keys[len(self.pressed_keys)]

            if key_str == expected_key:
                elapsed = (now - self.start_time).nanoseconds / 1e9
                self.timestamps.append(elapsed)
                self.pressed_keys.append(key_str)
                self.get_logger().info(f"Key '{key_str}' pressed at +{elapsed:.3f} seconds")

                if key_str != 'space':
                    # Predict when space will be pressed
                    self.linear_predict()
                else:
                    # On space: print final comparison
                    # self.show_prediction_summary()
                    self.seeker_is_looking = True
                    self.robot_state = RobotState.SEEKER_LOOKING
                    self.get_logger().info("Seeker is looking.")
            elif key_str in self.pressed_keys:
                self.get_logger().info(f"Ignored repeated key '{key_str}'")
            else:
                self.get_logger().info(f"Ignored out-of-sequence key '{key_str}'")

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
        waypoint_file = os.path.join(self.output_folder, 'waypoints.npy')
        if not os.path.exists(waypoint_file):
            self.get_logger().error(f"Waypoint file not found: {waypoint_file}")
            return None

        try:
            positions = np.load(waypoint_file)
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
        waypoint_file = os.path.join(self.occluded_folder, 'occluded_points.npy')
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
        if not self.seeker_is_looking and not self.robot_state == RobotState.INIALIZING:
            # as soon as the switch is detected plan route to the next waypoint. When the space bar is in sight start thinking about what to do next
            if self.robot_state == RobotState.EXPECT_MISSION:
                self.end_trajectory_target_time = self.plan_and_publish(self.waypoints.poses[self.current_index])
                self.current_index += 1
                self.just_switched = False
                self.trajectory_time_start = self.get_clock().now()
                self.robot_state = RobotState.MISSION
            elif len(self.predictions) >0 and self.robot_state == RobotState.MISSION:
                avg_prediction = np.mean(self.predictions)
                current_time = (self.get_clock().now() - self.trajectory_time_start).nanoseconds / 1e9
                time_to_completion = self.end_trajectory_target_time - current_time
                self.get_logger().info(f"Time to completion: {time_to_completion:.3f}s | Avg prediction: {avg_prediction:.3f}s")

                if time_to_completion > avg_prediction:
                    self.get_logger().info("Returning to home")
                    # find the closest not occluded point
                    if self.out_of_collision is not None:
                        current_position = np.array([
                            self.pose.pose.position.x,
                            self.pose.pose.position.y,
                            self.pose.pose.position.z
                        ])
                        distances = np.linalg.norm(self.out_of_collision - current_position, axis=1)
                        closest_index = np.argmin(distances)
                        closest_point = self.out_of_collision[closest_index]
                        self.get_logger().info(f"Closest not occluded point: {closest_point}")
                        # Plan and publish trajectory to the closest not occluded point
                        pose = Pose()
                        pose.position.x, pose.position.y, pose.position.z = closest_point
                        _ = self.plan_and_publish(pose)

                    self.current_index -= 1
                    self.robot_state = RobotState.ABORT
                    
                # check if drone will get to the next waypoint before the space bar is pressed


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
        return tf

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
