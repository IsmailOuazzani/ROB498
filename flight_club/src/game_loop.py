#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.clock import Clock

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty

import logging
import sys
import time
import threading
from pynput import keyboard
from playsound import playsound

from flight_club.msg import GameInfo

from argparse import ArgumentParser

NODE_NAME = "game_loop_node"
NODE_NAMESPACE = "flight_club"


class GameLoopNode(Node):
  def __init__(self, map_name: str):
    super().__init__(NODE_NAME, namespace=NODE_NAMESPACE)

    # Set up logging
    self._logger = logging.getLogger("game_loop_logger")
    self._logger.setLevel(logging.DEBUG)
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
    self.map_name = map_name
    self.game_state = GameInfo.GAME_STATE_STOP

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

    # Keyboard listener
    self.kb_listener_thread = threading.Thread(target=self._keyboard_listener, daemon=True)
    self.kb_listener_thread.start()

    self._logger.info("GameLoopNode initialized.")


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
    if force_publish:
        self._logger.debug("Published game info (forced).")
    else:
        self._logger.debug("Published game info (timer).")

if __name__ == "__main__":
  parser = ArgumentParser(description="Game Loop Node")
  parser.add_argument(
    "-m", "--map", type=str, default="default_map", help="Map name"
  )

  rclpy.init(args=sys.argv)
  node = GameLoopNode(
    map_name=parser.parse_args().map,
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
  