#!/usr/bin/env python3

import logging
import sys
import time
import threading
from argparse import ArgumentParser
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.clock import Clock

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Point32, Point
from std_srvs.srv import Empty
from sensor_msgs.msg import PointCloud, ChannelFloat32
from visualization_msgs.msg import Marker, MarkerArray

import numpy as np
from pynput import keyboard
from playsound import playsound

from flight_club.msg import GameInfo

from hiding.hiding import parse_sdf_map



NODE_NAME = "game_loop_node"
NODE_NAMESPACE = "flight_club"

GOAL_TOLERANCE = 0.5  # meters

DATA_DIR = Path(__file__).resolve().parent.parent.parent / "output"
WORLDS_DIR = Path(__file__).resolve().parent.parent.parent / "simulation/worlds"


class GameLoopNode(Node):
  def __init__(self, goal_x: float):
    super().__init__(NODE_NAME, namespace=NODE_NAMESPACE)
    self.declare_parameter("map_name", "dust2")
    self.map_name = self.get_parameter("map_name").get_parameter_value().string_value
    # Set up logging
    self._logger = logging.getLogger("game_loop_logger")
    self._logger.setLevel(logging.INFO)
    fh = logging.FileHandler("game_loop.log")
    fh.setLevel(logging.DEBUG)
    # Create console handler
    ch = logging.StreamHandler()
    ch.setLevel(logging.DEBUG)
    formatter = logging.Formatter("[%(asctime)s] [%(levelname)s]: %(message)s")
    fh.setFormatter(formatter)
    ch.setFormatter(formatter)
    self._logger.addHandler(fh)
    self._logger.addHandler(ch)

    self._logger.info("Initializing GameLoopNode...")

    # Initialize parameters
    self.game_state = GameInfo.GAME_STATE_STOP
    self.goal_x = goal_x

    world_file = WORLDS_DIR / f"{self.map_name}.sdf"
    self.world = parse_sdf_map(world_file)
    occluded_map_file = DATA_DIR / f"{self.map_name}_occluded.npy"
    self.occluded_map = np.load(occluded_map_file).reshape(-1, 3).astype(np.float32)
    waypoints_file = DATA_DIR / f"{self.map_name}_waypoints.npy"
    self.waypoints = np.load(waypoints_file).reshape(-1, 3).astype(np.float32)


    # States we rotate through when pressing space (in the exact order):
    # BLIND_1 -> BLIND_2 -> BLIND_3 -> SEEKING -> (then back to) BLIND_INDEF
    self.cycle_states = [
        GameInfo.GAME_STATE_BLIND_1,
        GameInfo.GAME_STATE_BLIND_2,
        GameInfo.GAME_STATE_BLIND_3,
        GameInfo.GAME_STATE_SEEKING
    ]
    self.cycle_index = 0


    # Set up publishers
    self.game_info_pub = self.create_publisher(
      GameInfo, "game_info", 10
    )
    self.publish_timer = self.create_timer(
      1.0, self.publish_game_info
    )
    self.world_pub = self.create_publisher(
      MarkerArray, "world", 10
    )
    self.publish_world_timer = self.create_timer(
      1.0, self.publish_world
    )
    self.occluded_pub = self.create_publisher(
      PointCloud, "occluded", 10
    )
    self.publish_occluded_timer = self.create_timer(
      1.0, self.publish_occluded,
    )
    self.waypoints_pub = self.create_publisher(
      Marker, "waypoints", 10
    )
    self.publish_waypoints_timer = self.create_timer(
      1.0, self.publish_waypoints,
    )


    # Set up services
    self.start_game_srv = self.create_service(
        Empty,
        "start_game",
        self.start_game_callback
    )
    self.stop_game_srv = self.create_service(
        Empty,
        "stop_game",
        self.stop_game_callback
    )

    # Subscribe to the pose topic
    qos_profile = QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
        depth=2
    )

    self.pose_sub = self.create_subscription(
        PoseStamped,
        "/mavros/local_position/pose",
        self.pose_callback,
        qos_profile
    )

    # Keyboard listener
    self.kb_listener_thread = threading.Thread(target=self._keyboard_listener, daemon=True)
    self.kb_listener_thread.start()


    # Internals
    self._victory_played = False

    self._logger.info("GameLoopNode initialized.")

  def publish_world(self): # TODO: move the formatting part of this function to another file to declutter
    time_now = self.get_clock().now().to_msg()

    marker_array = MarkerArray()
    for model in self.world.models:
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp =time_now
        marker.ns = 'cylinders'
        marker.id = len(marker_array.markers)
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = model.pose[0]
        marker.pose.position.y = model.pose[1]
        marker.pose.position.z = model.pose[2]
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = model.radius * 2
        marker.scale.y = model.radius * 2
        marker.scale.z = model.length
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker_array.markers.append(marker)
    # Create a sphere marker for the seeker
    seeker_marker = Marker()
    seeker_marker.header.frame_id = "map"
    seeker_marker.header.stamp = time_now
    seeker_marker.ns = 'seeker'
    seeker_marker.id = 0
    seeker_marker.type = Marker.SPHERE
    seeker_marker.action = Marker.ADD
    seeker_marker.pose.position.x = self.world.seeker_pose[0]
    seeker_marker.pose.position.y = self.world.seeker_pose[1]
    seeker_marker.pose.position.z = self.world.seeker_pose[2]
    seeker_marker.pose.orientation.x = 0.0
    seeker_marker.pose.orientation.y = 0.0
    seeker_marker.pose.orientation.z = 0.0
    seeker_marker.pose.orientation.w = 1.0
    seeker_marker.scale.x = 1.0
    seeker_marker.scale.y = 1.0
    seeker_marker.scale.z = 1.0
    if self.game_state == GameInfo.GAME_STATE_SEEKING:
        seeker_marker.color.r = 1.0
        seeker_marker.color.g = 0.0
        seeker_marker.color.b = 0.0
    else:
        seeker_marker.color.r = 0.0
        seeker_marker.color.g = 1.0
        seeker_marker.color.b = 0.0
    seeker_marker.color.a = 1.0
    marker_array.markers.append(seeker_marker)
    # Publish the marker array
    self.world_pub.publish(marker_array)
    self._logger.debug("Published world marker array.")

  def publish_occluded(self): # TODO: move the formatting part of this function to another file to declutter
    # Create PointCloud message
    occluded_msg = PointCloud()
    occluded_msg.header = Header()
    occluded_msg.header.stamp = self.get_clock().now().to_msg()
    occluded_msg.header.frame_id = "map"  # arbitrary, if needed
    occluded_msg.points = []
    # single channel with intensity 1.0
    occluded_msg.channels = []
    occluded_msg.channels.append(ChannelFloat32())
    occluded_msg.channels[0].name = "intensity"
    occluded_msg.channels[0].values = [1.0] * len(self.occluded_map)
    # Fill the points
    for point in self.occluded_map:
        p = Point32()
        p.x = float(point[0])
        p.y = float(point[1])
        p.z = float(point[2])
        occluded_msg.points.append(p)
    # Publish the message
    self.occluded_pub.publish(occluded_msg)
    self._logger.debug("Published occluded point cloud.")

  def publish_waypoints(self):    
    # Create a LINE_STRIP marker
    waypoints_marker = Marker()
    waypoints_marker.header.frame_id = "map"
    waypoints_marker.header.stamp = self.get_clock().now().to_msg()
    waypoints_marker.ns = 'waypoints'
    waypoints_marker.id = 0
    waypoints_marker.type = Marker.LINE_STRIP
    waypoints_marker.action = Marker.ADD
    waypoints_marker.scale.x = 0.3
    waypoints_marker.color.r = 0.0
    waypoints_marker.color.g = 1.0
    waypoints_marker.color.b = 1.0
    waypoints_marker.color.a = 1.0
    for point in self.waypoints:
        p = Point()
        p.x = float(point[0])
        p.y = float(point[1])
        p.z = float(point[2])
        waypoints_marker.points.append(p)
    # Publish the marker
    self.waypoints_pub.publish(waypoints_marker)
    self._logger.debug("Published waypoints marker.")


  def start_game_callback(self, request, response):
    self._logger.info("Received start_game service call.")
    if self.game_state == GameInfo.GAME_STATE_STOP:
        self._logger.info("Game transitioning to BLIND_INDEF.")
        self.set_game_state(GameInfo.GAME_STATE_BLIND_INDEF)
    else:
        self._logger.warning("start_game called, but game not in STOP state. No transition.")

    self.game_start_time_ns = time.time_ns()
    return response  # Empty response
  
  def stop_game_callback(self, request, response):
    self._logger.info("Received stop_game service call.")
    self.set_game_state(GameInfo.GAME_STATE_STOP)
    return response  # Empty response
  
  def set_game_state(self, new_state: int):
    self._logger.info(f"Game state changing from {self.game_state} to {new_state}")
    self.game_state = new_state

    if new_state == GameInfo.GAME_STATE_BLIND_INDEF:
        self.cycle_index = 0

    self.publish_game_info(force_publish=True)

  
  def pose_callback(self, msg: PoseStamped):
    """Callback whenever a new pose arrives from /mavros/local_position/pose."""
    # Update last_pose_time for warning checks
    self.last_pose_time = self.get_clock().now()
    x,y,z = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z

    # 1) Check if something (placeholder if True) => if False => LOST
    #    We'll invert the logic: if not True => become LOST
    if not True: #TODO: use query_ball_point on visible or obstacle KDTrees to test loss
        self._logger.error("Game lost condition triggered!")
        self.set_game_state(GameInfo.GAME_STATE_LOST)
        return

    # 2) Another placeholder if False => if True => WON
    if abs(x - self.goal_x) < GOAL_TOLERANCE and self._victory_played == False:
        self._logger.info("Game won condition triggered!")
        self.set_game_state(GameInfo.GAME_STATE_WON)
        # Play victory sound
        def play_victory_sound():
          try:
              playsound("/src/ros_ws/src/drone_packages/victory.mp3")
          except Exception as e:
              self._logger.error(f"Error playing sound: {e}")

        threading.Thread(target=play_victory_sound, daemon=True).start()
        self._victory_played = True
        return


  def _keyboard_listener(self):
    """Background thread to listen for keyboard events (using pynput)."""
    def on_press(key):
        # Only handle space-bar if the game is in a state that allows cycling
        if key == keyboard.Key.space:
            # If in BLIND_INDEF -> go to BLIND_1
            if self.game_state == GameInfo.GAME_STATE_BLIND_INDEF:
                next_state = self.cycle_states[self.cycle_index]
                self.set_game_state(next_state)

            # If in BLIND_1, BLIND_2, BLIND_3, or SEEKING -> cycle next
            elif self.game_state in self.cycle_states:
                self.cycle_index = (self.cycle_index + 1) % len(self.cycle_states)
                next_state = self.cycle_states[self.cycle_index]

                # If we wrapped back to BLIND_1, that means we just ended SEEKING;
                # but the original instructions want us to go back to BLIND_INDEF
                # after SEEKING. So let's detect that:
                if self.cycle_states[self.cycle_index] == GameInfo.GAME_STATE_BLIND_1:
                    self.set_game_state(GameInfo.GAME_STATE_BLIND_INDEF)
                else:
                    self.set_game_state(next_state)
        return True  # continue listening

    with keyboard.Listener(on_press=on_press) as listener:
        listener.join()  # This will block until the listener is stopped

  def publish_game_info(self, force_publish=False):
    """
    Publish the current game info message.
    force_publish=True means publish even if it's just the same state repeated.
    Otherwise, we also do it at a fixed frequency from the timer.
    """
    msg = GameInfo()
    msg.header = Header()
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.header.frame_id = "game_loop"  # arbitrary, if needed
    msg.map_name = self.map_name
    msg.game_state = self.game_state

    # If forced or from the timer
    self.game_info_pub.publish(msg)
    # if force_publish:
    #     self._logger.debug("Published game info (forced).")
    # else:
    #     self._logger.debug("Published game info (timer).")

if __name__ == "__main__":

  rclpy.init(args=sys.argv) #TODO: use argparse instead
  node = GameLoopNode(
    goal_x = -20.0, #TODO: replace with something conditional on the map, with the seeker
  )

  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    node.get_logger().info("Keyboard interrupt, shutting down...")
    node.destroy_node()
    rclpy.shutdown()
  except Exception as e:
    node.get_logger().error(f"Exception occurred: {e}")
    node.destroy_node()
    rclpy.shutdown()
  