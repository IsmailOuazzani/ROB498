#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from pynput import keyboard
import numpy as np


class SequenceTimerNode(Node):
    def __init__(self):
        super().__init__('sequence_timer')
        self.get_logger().info("Press 1 to start the sequence timer (then 2, 3, space in order)")

        # Key sequence we're looking for (after '1')
        self.expected_keys = ['2', '3', 'space']
        self.pressed_keys = []

        self.start_time = None
        self.timestamps = []  # Elapsed times
        self.predictions = []  # Predictions of when 'space' would be pressed

        self.listener = keyboard.Listener(on_press=self.on_key_press)
        self.listener.start()

    def on_key_press(self, key):
        try:
            key_str = key.char
        except AttributeError:
            key_str = 'space' if key == keyboard.Key.space else None

        if key_str is None:
            return

        now = self.get_clock().now()

        if key_str == '1':
            # Reset everything
            self.start_time = now
            self.pressed_keys = []
            self.timestamps = [0.0]  # Start time
            self.predictions = []
            self.get_logger().info("Key '1' pressed. Timer started.")
            return

        if self.start_time is None:
            return  # Wait until '1' is pressed

        if len(self.pressed_keys) < len(self.expected_keys):
            expected_key = self.expected_keys[len(self.pressed_keys)]

            if key_str == expected_key:
                elapsed = (now - self.start_time).nanoseconds / 1e9
                self.timestamps.append(elapsed)
                self.pressed_keys.append(key_str)
                self.get_logger().info(f"Key '{key_str}' pressed at +{elapsed:.3f} seconds")
                self.get_logger().info(f"timestamps: {self.timestamps}")

                if key_str != 'space':
                    # Predict when space will be pressed
                    self.linear_predict()
                else:
                    # On space: print final comparison
                    self.show_prediction_summary()
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
