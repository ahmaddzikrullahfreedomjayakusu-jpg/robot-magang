#!/usr/bin/env python3
"""Diagnostic: print the LiDAR's live front-sector distance repeatedly.

Run this with RPLidar already running (e.g. after scripts/start.sh mapping),
in a separate terminal, while the robot is NOT facing a real wall. If the
number stays small and constant no matter what's actually in front of the
robot, front_stop_m is triggering on the robot's own body, not a real
obstacle.

Usage: python3 diag_front_distance.py
"""
import math
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


def sector_min_range(scan, center_deg, half_width_deg):
    center = math.radians(center_deg)
    half = math.radians(half_width_deg)
    best = float("inf")
    angle = scan.angle_min
    for r in scan.ranges:
        diff = abs(((angle - center + math.pi) % (2 * math.pi)) - math.pi)
        if diff <= half and scan.range_min <= r <= scan.range_max:
            best = min(best, r)
        angle += scan.angle_increment
    return best


class Diag(Node):
    def __init__(self):
        super().__init__("diag_front_distance")
        self.create_subscription(LaserScan, "/scan", self.on_scan, 10)

    def on_scan(self, msg):
        front = sector_min_range(msg, 180.0, 20.0)
        fr = sector_min_range(msg, 135.0, 25.0)
        fl = sector_min_range(msg, -135.0, 25.0)
        print(f"front={front:.2f} m   front-right={fr:.2f} m   front-left={fl:.2f} m", flush=True)


def main():
    rclpy.init()
    node = Diag()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
