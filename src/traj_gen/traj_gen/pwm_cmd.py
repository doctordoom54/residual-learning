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

pkg_share_dir = get_package_share_directory('traj_gen')
CSV_FILE_PATH = os.path.join(pkg_share_dir, 'data', 'spiral_pwm_commands.csv')

class LeoCSVPlayer(Node):
    def __init__(self):
        super().__init__("leo_csv_player")

        self.commands = []
        self.current_index = 0

        # 1. Load the CSV data
        self._load_csv_data(CSV_FILE_PATH)

        # 2. Setup Publishers
        self._wheel_publishers = {}
        for wheel in LEFT_WHEELS + RIGHT_WHEELS:
            topic = f"firmware/wheel_{wheel}/cmd_pwm_duty"
            self._wheel_publishers[wheel] = self.create_publisher(Float32, topic, 10)

        # 3. Calculate dynamic publish rate
        dt = self.commands[1][0] - self.commands[0][0]
        publish_rate_hz = 1.0 / dt
        
        # 4. Start the timer
        self._timer = self.create_timer(dt, self._on_timer)

        self.get_logger().info(
            f"\n--- Loaded CSV Trajectory ---\n"
            f"Total Commands: {len(self.commands)} steps\n"
            f"Time Step (dt): {dt:.4f} seconds\n"
            f"Publish Rate:   {publish_rate_hz:.1f} Hz\n"
            f"Total Duration: {(len(self.commands) * dt):.1f} seconds\n"
            f"-----------------------------"
        )

    def _load_csv_data(self, filepath):
        """Reads the CSV file and stores the data as a list of tuples."""
        if not os.path.exists(filepath):
            self.get_logger().error(f"Cannot find CSV file at: {filepath}")
            sys.exit(1)

        with open(filepath, 'r') as f:
            reader = csv.reader(f)
            header = next(reader) # Skip header (time,pwm_r,pwm_l)
            
            for row in reader:
                if len(row) == 3:
                    t = float(row[0])
                    pwm_r = float(row[1])
                    pwm_l = float(row[2])
                    self.commands.append((t, pwm_r, pwm_l))

        if len(self.commands) < 2:
            self.get_logger().error("CSV file contains insufficient data.")
            sys.exit(1)

    def _on_timer(self):
            """
            Publishes command every 0.02s. 
            Since input data is 0.01s, incrementing by 2 selects every 0.02s sample.
            """
            
            # 1. Check end of trajectory
            if self.current_index >= len(self.commands):
                self.get_logger().info("Trajectory complete. Shutting down...")
                self._timer.cancel()
                self.stop()
                # We call shutdown asynchronously to allow the stop commands to publish
                rclpy.shutdown()
                return

            # 2. Extract command from CSV (Zero-Order Hold)
            _, pwm_r, pwm_l = self.commands[self.current_index]

            # 3. Publish
            right_msg = Float32(data=pwm_r)
            left_msg = Float32(data=pwm_l)

            for wheel in RIGHT_WHEELS:
                self._wheel_publishers[wheel].publish(right_msg)
            for wheel in LEFT_WHEELS:
                self._wheel_publishers[wheel].publish(left_msg)

            # 4. Advance pointer by 2 to skip the intermediate 0.01s step
            self.current_index += 2
            
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