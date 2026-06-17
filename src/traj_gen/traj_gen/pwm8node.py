#!/usr/bin/env python3
"""
ROS 2 node that drives the Leo Rover in a continuous figure-8 (lemniscate)
by publishing time-varying sinusoidal PWM commands to the wheel topics at 50 Hz.

Topics (std_msgs/msg/Float32, units: % duty cycle):
    firmware/wheel_FL/cmd_pwm_duty   (left)
    firmware/wheel_RL/cmd_pwm_duty   (left)
    firmware/wheel_FR/cmd_pwm_duty   (right)
    firmware/wheel_RR/cmd_pwm_duty   (right)
"""
import math
import signal
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

PUBLISH_RATE_HZ = 50.0

# --- Rover Parameters ---
R_WHEEL = 0.06      # r: wheel radius (m)
TRACK_WIDTH = 0.353  # B: distance between left and right wheels (m)
ALPHA = 0.77169     # Motor torque constant approx
BETA = 15.58681     # Motor friction constant approx

# --- Figure-8 Mathematics ---
BESSEL_ROOT = 2.404825
MAX_DELTA_PWM = 50.0  # 100 - 50 = 50
PWM_BASE = 70.0
PWM_AMP = 25.0

# Precompute the cycle frequency based on maximum yaw rate
MAX_YAW_RATE = (R_WHEEL * ALPHA * MAX_DELTA_PWM) / (TRACK_WIDTH * BETA)
W_CYCLE = MAX_YAW_RATE / BESSEL_ROOT
T_CYCLE = 2.0 * math.pi / W_CYCLE

LEFT_WHEELS = ("FL", "RL")
RIGHT_WHEELS = ("FR", "RR")


class LeoFigure8(Node):
    def __init__(self):
        super().__init__("leo_figure_8")

        self._wheel_publishers = {}
        for wheel in LEFT_WHEELS + RIGHT_WHEELS:
            topic = f"firmware/wheel_{wheel}/cmd_pwm_duty"
            self._wheel_publishers[wheel] = self.create_publisher(Float32, topic, 10)

        self._start_time = self.get_clock().now()
        period = 1.0 / PUBLISH_RATE_HZ
        self._timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f"Starting continuous Figure-8 trajectory.\n"
            f"Cycle Frequency: {W_CYCLE:.4f} rad/s\n"
            f"Cycle Period: {T_CYCLE:.2f} seconds\n"
            f"Base PWM: {PWM_BASE}%, Amplitude: {PWM_AMP}%"
        )

    def _on_timer(self):
        # Calculate elapsed time in seconds
        elapsed_s = (self.get_clock().now() - self._start_time).nanoseconds * 1e-9

        # Evaluate the sinusoidal mathematical control policy
        wave = math.cos(W_CYCLE * elapsed_s)
        pwm_r = PWM_BASE + PWM_AMP * wave
        pwm_l = PWM_BASE - PWM_AMP * wave

        # Create Float32 messages
        right_msg = Float32(data=pwm_r)
        left_msg = Float32(data=pwm_l)

        # Publish to respective wheels
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
    node = LeoFigure8()

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