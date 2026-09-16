"""Blanks out /scan returns too close to the robot's own body before SLAM/Nav2 see it.

The laptop that powers the LiDAR rides along right behind the robot, so the
LiDAR sees it as a "ghost obstacle" on every scan, smearing fake obstacle
streaks across the whole map as the robot moves.

Filtering by RANGE (a circle of radius exclude_radius_m around the LiDAR,
diameter 0.50m by default) rather than by a fixed angle: anything close
enough to be the robot's own laptop/cables is dropped no matter which
direction it's in, while a real wall or obstacle further behind the robot
at the same angle stays visible. A pure angle-window filter would also have
blinded the robot to any real obstacle at that bearing, at any distance.

An optional angle window (off by default) is still available for a fixed
directional blind spot if you ever need one alongside the radius filter.
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanBlindSpotFilter(Node):

    def __init__(self):
        super().__init__("scan_blind_spot_filter")

        self.declare_parameter("input_topic", "/scan")
        self.declare_parameter("output_topic", "/scan_filtered")
        self.declare_parameter("exclude_radius_m", 0.25)
        self.declare_parameter("exclude_angle_min_deg", 0.0)
        self.declare_parameter("exclude_angle_max_deg", 0.0)

        self.exclude_radius = float(self.get_parameter("exclude_radius_m").value)
        self.exclude_min = math.radians(float(self.get_parameter("exclude_angle_min_deg").value))
        self.exclude_max = math.radians(float(self.get_parameter("exclude_angle_max_deg").value))
        self.angle_filter_enabled = self.exclude_min != self.exclude_max

        self.pub = self.create_publisher(LaserScan, self.get_parameter("output_topic").value, 10)
        self.create_subscription(
            LaserScan, self.get_parameter("input_topic").value, self.on_scan, 10
        )

        msg = f"scan_blind_spot_filter ready: blanking returns closer than {self.exclude_radius:.2f} m"
        if self.angle_filter_enabled:
            msg += (
                f", plus [{math.degrees(self.exclude_min):.0f}, "
                f"{math.degrees(self.exclude_max):.0f}] deg in the LiDAR's own frame"
            )
        self.get_logger().info(msg)

    def on_scan(self, msg: LaserScan) -> None:
        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max

        ranges = list(msg.ranges)
        intensities = list(msg.intensities) if msg.intensities else []

        angle = msg.angle_min
        for i in range(len(ranges)):
            too_close = ranges[i] < self.exclude_radius
            in_angle_window = self.angle_filter_enabled and self._in_excluded_sector(self._normalize(angle))
            if too_close or in_angle_window:
                ranges[i] = float("inf")
                if intensities:
                    intensities[i] = 0.0
            angle += msg.angle_increment

        out.ranges = ranges
        out.intensities = intensities
        self.pub.publish(out)

    @staticmethod
    def _normalize(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def _in_excluded_sector(self, angle: float) -> bool:
        if self.exclude_min <= self.exclude_max:
            return self.exclude_min <= angle <= self.exclude_max
        # window wraps across +-pi (e.g. min=170deg, max=-170deg)
        return angle >= self.exclude_min or angle <= self.exclude_max


def main() -> None:
    rclpy.init()
    node = ScanBlindSpotFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
