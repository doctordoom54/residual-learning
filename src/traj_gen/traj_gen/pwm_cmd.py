#!/usr/bin/env python3
"""
ROS 2 node that drives the Leo Rover by playing back an open-loop 
PWM command matrix exported from a CSV file.
"""
import os
import csv
import signal
import time
import sys
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

LEFT_WHEELS = ("FL", "RL")
RIGHT_WHEELS = ("FR", "RR")

# Fixed publish rate: 50 Hz, regardless of the CSV's native sample rate.
PUBLISH_RATE_HZ = 50.0
PUBLISH_DT = 1.0 / PUBLISH_RATE_HZ

pkg_share_dir = get_package_share_directory('traj_gen')
CSV_FILE_PATH = os.path.join(pkg_share_dir, 'data', 'spiral_pwm_commands.csv')

class LeoCSVPlayer(Node):
    def __init__(self):
        super().__init__("leo_csv_player")

        self.commands = []

        # 1. Load the CSV data
        self._load_csv_data(CSV_FILE_PATH)

        # 2. Setup Publishers
        self._wheel_publishers = {}
        for wheel in LEFT_WHEELS + RIGHT_WHEELS:
            topic = f"firmware/wheel_{wheel}/cmd_pwm_duty"
            self._wheel_publishers[wheel] = self.create_publisher(Float32, topic, 10)

        # 3. Trajectory playback state.
        # We publish at a fixed PUBLISH_DT (50 Hz) and zero-order-hold whichever
        # CSV sample corresponds to the current playback time. This makes
        # playback correct regardless of the CSV's native sample spacing
        # (0.01s, 0.02s, or anything else) since lookup is by elapsed time,
        # not by a fixed row-skip count.
        self._traj_start_time = None
        csv_dt = self.commands[1][0] - self.commands[0][0]
        self.trajectory_duration = self.commands[-1][0]

        # 4. Start the timer at the fixed 50 Hz rate
        self._timer = self.create_timer(PUBLISH_DT, self._on_timer)

        self.get_logger().info(
            f"\n--- Loaded CSV Trajectory ---\n"
            f"Total Commands:   {len(self.commands)} rows\n"
            f"CSV Sample Step:  {csv_dt:.4f} seconds\n"
            f"Publish Rate:     {PUBLISH_RATE_HZ:.1f} Hz (fixed, zero-order hold)\n"
            f"Total Duration:   {self.trajectory_duration:.2f} seconds\n"
            f"-----------------------------"
        )

    def _load_csv_data(self, filepath):
        """Reads the CSV file and stores the data as a list of tuples."""
        if not os.path.exists(filepath):
            self.get_logger().error(f"Cannot find CSV file at: {filepath}")
            sys.exit(1)

        with open(filepath, 'r') as f:
            reader = csv.reader(f)
            header = next(reader)  # Skip header (time,pwm_r,pwm_l)

            for row in reader:
                if len(row) == 3:
                    t = float(row[0])
                    pwm_r = float(row[1])
                    pwm_l = float(row[2])
                    self.commands.append((t, pwm_r, pwm_l))

        if len(self.commands) < 2:
            self.get_logger().error("CSV file contains insufficient data.")
            sys.exit(1)

    def _lookup_command(self, elapsed):
        """
        Zero-order hold lookup: returns the most recent CSV command whose
        timestamp is <= elapsed. Advances a cached index forward only,
        since elapsed time is monotonically increasing.
        """
        idx = self._lookup_index
        n = len(self.commands)

        while idx + 1 < n and self.commands[idx + 1][0] <= elapsed:
            idx += 1

        self._lookup_index = idx
        return self.commands[idx]

    def _on_timer(self):
        """Publishes the zero-order-held command at a fixed 50 Hz rate."""
        if self._traj_start_time is None:
            self._traj_start_time = time.monotonic()
            self._lookup_index = 0

        elapsed = time.monotonic() - self._traj_start_time

        # 1. Check end of trajectory
        if elapsed >= self.trajectory_duration:
            self.get_logger().info("Trajectory complete. Shutting down...")
            self._timer.cancel()
            self.stop()
            rclpy.shutdown()
            return

        # 2. Zero-order hold: get the command active at this elapsed time
        _, pwm_r, pwm_l = self._lookup_command(elapsed)

        # 3. Publish
        right_msg = Float32(data=pwm_r)
        left_msg = Float32(data=pwm_l)

        for wheel in RIGHT_WHEELS:
            self._wheel_publishers[wheel].publish(right_msg)
        for wheel in LEFT_WHEELS:
            self._wheel_publishers[wheel].publish(left_msg)

    def stop(self):
        """
        Publish 0% duty cycle to all wheels on shutdown to prevent 
        the rover from coasting at the last commanded PWM value.
        """
        zero = Float32(data=0.0)
        for _ in range(10):
            for wheel, pub in self._wheel_publishers.items():
                pub.publish(zero)
            time.sleep(0.02)
        self.get_logger().info("Stop command sent to all wheels.")


def main():
    rclpy.init()
    node = LeoCSVPlayer()

    stop_requested = False

    def _on_sigint(signum, frame):
        nonlocal stop_requested
        if stop_requested:
            return
        stop_requested = True
        node._timer.cancel()
        node.stop()

    # Intercept SIGINT to guarantee the stop commands are broadcasted
    signal.signal(signal.SIGINT, _on_sigint)

    try:
        while rclpy.ok() and not stop_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        if not stop_requested:
            node._timer.cancel()
            node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()