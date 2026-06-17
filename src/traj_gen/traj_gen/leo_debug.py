#!/usr/bin/env python3
import time
import threading
from collections import deque
from functools import partial

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

import matplotlib.pyplot as plt
import matplotlib.animation as animation


class LeoPWMDebugger(Node):
    def __init__(self):
        super().__init__("leo_pwm_debugger")

        self.topics = [
            "firmware/wheel_FL/cmd_pwm_duty",
            "firmware/wheel_RL/cmd_pwm_duty",
            "firmware/wheel_FR/cmd_pwm_duty",
            "firmware/wheel_RR/cmd_pwm_duty"
        ]

        self.msg_counts = {topic: 0 for topic in self.topics}
        self.latest_data = {topic: 0.0 for topic in self.topics}
        
        # Deques act as a rolling window. 500 items at 50Hz = ~10 seconds of history
        self.history = {topic: deque(maxlen=500) for topic in self.topics}
        self.start_time = time.time()

        # Create subscriptions for all wheel topics
        for topic in self.topics:
            self.create_subscription(
                Float32, 
                topic, 
                partial(self.listener_callback, topic_name=topic), 
                10
            )

        # Timer to calculate and log the frequency and data every 2 seconds
        self.stats_period = 2.0
        self.create_timer(self.stats_period, self.print_stats)

        self.get_logger().info("PWM Debugger started. Listening to wheel commands...")

    def listener_callback(self, msg, topic_name):
        """Stores the latest message data, increments counter, and saves history."""
        self.latest_data[topic_name] = msg.data
        self.msg_counts[topic_name] += 1
        
        # Record the elapsed time and value for plotting
        current_time = time.time() - self.start_time
        self.history[topic_name].append((current_time, msg.data))

    def print_stats(self):
        """Calculates the frequency and prints the latest state to the terminal."""
        self.get_logger().info("\n--- Node Debug Stats ---")
        
        for topic in self.topics:
            freq = self.msg_counts[topic] / self.stats_period
            current_pwm = self.latest_data[topic]
            
            self.get_logger().info(
                f"Topic: {topic.split('/')[-2]} | "
                f"Rate: {freq:.1f} Hz | "
                f"Data: {current_pwm:.2f}%"
            )
            
            self.msg_counts[topic] = 0


def main():
    rclpy.init()
    node = LeoPWMDebugger()

    # 1. Run ROS 2 spin in a background daemon thread
    # Matplotlib MUST run in the main thread to display the GUI safely.
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    # 2. Set up the Matplotlib Figure
    fig, ax = plt.subplots(figsize=(12, 6))
    ax.set_title("Real-Time Rover PWM Commands")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("PWM Duty Cycle (%)")
    ax.set_ylim(-5, 105)  # Bounded to show standard 0-100% duty cycle
    ax.grid(True, linestyle=':', alpha=0.7)

    # Create empty line objects for each topic
    lines = {}
    colors = ['#2980b9', '#c0392b', '#27ae60', '#f39c12'] # Blue, Red, Green, Orange
    
    for i, topic in enumerate(node.topics):
        wheel_name = topic.split('/')[-2]
        lines[topic], = ax.plot([], [], label=wheel_name, color=colors[i], linewidth=2)
    
    ax.legend(loc="upper right")

    # 3. Define the animation update function
    def update_plot(frame):
        current_time = time.time() - node.start_time
        
        # Fast update data for each line
        for topic in node.topics:
            data = list(node.history[topic])
            if data:
                times = [d[0] for d in data]
                vals = [d[1] for d in data]
                lines[topic].set_data(times, vals)
        
        # Scroll the X-axis to dynamically show the last 10 seconds
        window_size = 10.0
        ax.set_xlim(max(0, current_time - window_size), max(window_size, current_time))
        
        return lines.values()

    # 4. Start the animation loop (updates every 50ms / 20fps)
    ani = animation.FuncAnimation(fig, update_plot, interval=50, cache_frame_data=False)
    
    try:
        # plt.show() blocks the main thread until you close the window
        plt.show()  
    except KeyboardInterrupt:
        pass
    finally:
        # Clean shutdown when the window is closed
        node.get_logger().info("Shutting down debugger...")
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        ros_thread.join(timeout=1.0)

if __name__ == "__main__":
    main()