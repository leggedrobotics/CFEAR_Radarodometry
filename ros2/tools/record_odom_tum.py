#!/usr/bin/env python3
"""Record /odometry to a TUM trajectory file (t x y z qx qy qz qw).

    python3 record_odom_tum.py out.tum [odom_topic]

Stop with Ctrl+C; the file is flushed after every pose.
"""
import sys

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class TumRecorder(Node):
    def __init__(self, path, topic):
        super().__init__("tum_recorder")
        self.f = open(path, "w")
        self.n = 0
        self.create_subscription(Odometry, topic, self.cb, 100)
        self.get_logger().info(f"recording {topic} -> {path}")

    def cb(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.f.write(f"{t:.6f} {p.x} {p.y} {p.z} {q.x} {q.y} {q.z} {q.w}\n")
        self.f.flush()
        self.n += 1
        if self.n % 200 == 0:
            self.get_logger().info(f"recorded {self.n} poses")


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "odom_tum.txt"
    topic = sys.argv[2] if len(sys.argv) > 2 else "/odometry"
    rclpy.init()
    node = TumRecorder(path, topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
