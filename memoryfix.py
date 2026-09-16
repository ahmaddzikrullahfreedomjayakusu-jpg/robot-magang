#!/usr/bin/env python3

import math

import rclpy

from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Path

from geometry_msgs.msg import (
    PoseStamped,
    Point
)

from visualization_msgs.msg import Marker

from std_msgs.msg import Bool, String

from tf2_ros import (
    Buffer,
    TransformListener
)


# ============================================================
# MEMORY CONFIG
# ============================================================

ROBOT_WIDTH = 0.50

PATH_STEP = 0.05

# kalau >5% titik lidar masuk area unknown
# dianggap menemukan area baru
NEW_AREA_THRESHOLD = 0.05

# Area lama dikunci: jangan update map lama.
KNOWN_SCAN_DIVIDER = 0

SCAN_SAMPLE_STEP = 2


class MemoryNode(Node):

    def __init__(self):

        super().__init__("smart_memory")

        # ====================================================
        # TF
        # ====================================================

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        # ====================================================
        # SUBSCRIBERS
        # ====================================================

        self.create_subscription(
            OccupancyGrid,
            "/map",
            self.map_callback,
            5
        )

        self.create_subscription(
            LaserScan,
            "/scan_front",
            self.scan_callback,
            20
        )

        # ====================================================
        # OUTPUT SCAN TO SLAM
        # ====================================================

        self.scan_pub = self.create_publisher(
            LaserScan,
            "/scan_slam",
            20
        )

        # ====================================================
        # MEMORY OUTPUT
        # ====================================================

        self.mode_pub = self.create_publisher(
            String,
            "/map_mode",
            10
        )

        self.ready_pub = self.create_publisher(
            Bool,
            "/smart_memory_ready",
            1
        )

        self.path_pub = self.create_publisher(
            Path,
            "/robot_path",
            10
        )

        self.return_pub = self.create_publisher(
            Path,
            "/return_path",
            10
        )

        self.home_pub = self.create_publisher(
            PoseStamped,
            "/home_pose",
            10
        )

        self.trail_pub = self.create_publisher(
            Marker,
            "/visited_area",
            10
        )

        # ====================================================
        # MEMORY
        # ====================================================

        self.map = None

        self.mode = "EXPLORE"

        self.known_counter = 0

        self.home_pose = None

        self.path_points = []

        self.last_path_x = None
        self.last_path_y = None

        self.visited_triangles = []

        self.create_timer(
            0.10,
            self.update_robot_memory
        )

        self.create_timer(
            1.0,
            self.publish_ready
        )

        self.get_logger().info(
            "==================================="
        )

        self.get_logger().info(
            "SMART MEMORY READY"
        )

        self.get_logger().info(
            "KNOWN AREA  = MAP SOFT LOCK"
        )

        self.get_logger().info(
            "NEW AREA    = FULL MAPPING"
        )

        self.get_logger().info(
            "ROBOT WIDTH = 50 cm"
        )

        self.get_logger().info(
            "==================================="
        )

    # ========================================================
    # MAP
    # ========================================================

    def map_callback(self, msg):

        self.map = msg

    # ========================================================
    # TF POSE
    # ========================================================

    def get_pose(self, child="base_footprint"):

        try:

            tf = self.tf_buffer.lookup_transform(
                "map",
                child,
                Time()
            )

            x = tf.transform.translation.x
            y = tf.transform.translation.y

            q = tf.transform.rotation

            yaw = math.atan2(
                2.0 * (
                    q.w * q.z +
                    q.x * q.y
                ),
                1.0 - 2.0 * (
                    q.y * q.y +
                    q.z * q.z
                )
            )

            return x, y, yaw

        except Exception:

            return None

    # ========================================================
    # WORLD -> MAP CELL
    # ========================================================

    def world_to_map(self, x, y):

        if self.map is None:
            return None

        info = self.map.info

        ox = info.origin.position.x
        oy = info.origin.position.y

        q = info.origin.orientation

        oyaw = math.atan2(
            2.0 * (
                q.w * q.z +
                q.x * q.y
            ),
            1.0 - 2.0 * (
                q.y * q.y +
                q.z * q.z
            )
        )

        dx = x - ox
        dy = y - oy

        c = math.cos(-oyaw)
        s = math.sin(-oyaw)

        lx = c * dx - s * dy
        ly = s * dx + c * dy

        mx = int(
            lx /
            info.resolution
        )

        my = int(
            ly /
            info.resolution
        )

        if (
            mx < 0 or
            my < 0 or
            mx >= info.width or
            my >= info.height
        ):

            return None

        return mx, my

    # ========================================================
    # CHECK CELL
    # ========================================================

    def map_value(self, x, y):

        cell = self.world_to_map(
            x,
            y
        )

        if cell is None:
            return -1

        mx, my = cell

        index = (
            my *
            self.map.info.width +
            mx
        )

        if (
            index < 0 or
            index >= len(self.map.data)
        ):

            return -1

        return self.map.data[index]

    # ========================================================
    # SCAN MEMORY / MAP GATE
    # ========================================================

    def scan_callback(self, scan):

        # belum punya map
        if self.map is None:

            self.mode = "EXPLORE"

            self.publish_mode()

            self.scan_pub.publish(scan)

            return

        try:

            tf = self.tf_buffer.lookup_transform(
                "map",
                scan.header.frame_id,
                Time()
            )

        except Exception:

            # TF belum ready
            self.scan_pub.publish(scan)

            return

        tx = tf.transform.translation.x
        ty = tf.transform.translation.y

        q = tf.transform.rotation

        yaw = math.atan2(
            2.0 * (
                q.w * q.z +
                q.x * q.y
            ),
            1.0 - 2.0 * (
                q.y * q.y +
                q.z * q.z
            )
        )

        total = 0
        sampled_total = 0
        sampled_unknown = 0
        unknown = 0

        unknown_scan = self.make_unknown_scan(scan)

        angle = scan.angle_min

        for i, r in enumerate(scan.ranges):

            if (
                not math.isfinite(r) or
                r < scan.range_min or
                r > scan.range_max
            ):

                angle += scan.angle_increment
                continue

            world_angle = (
                yaw +
                angle
            )

            ex = (
                tx +
                r *
                math.cos(world_angle)
            )

            ey = (
                ty +
                r *
                math.sin(world_angle)
            )

            value = self.map_value(
                ex,
                ey
            )

            total += 1

            # belum pernah dipetakan
            if value == -1:

                unknown += 1
                unknown_scan.ranges[i] = r

                if (
                    unknown_scan.intensities and
                    i < len(scan.intensities)
                ):

                    unknown_scan.intensities[i] = scan.intensities[i]

            if i % SCAN_SAMPLE_STEP == 0:

                sampled_total += 1

                if value == -1:

                    sampled_unknown += 1

            angle += scan.angle_increment

        if sampled_total == 0:

            self.scan_pub.publish(scan)

            return

        unknown_ratio = (
            sampled_unknown /
            sampled_total
        )

        # ====================================================
        # NEW AREA
        # ====================================================

        if unknown_ratio >= NEW_AREA_THRESHOLD:

            if self.mode != "EXPLORE":

                self.get_logger().info(
                    f"NEW AREA -> MAPPING "
                    f"unknown={unknown_ratio:.0%}"
                )

            self.mode = "EXPLORE"

            # semua scan masuk SLAM
            self.scan_pub.publish(scan)

        # ====================================================
        # KNOWN AREA
        # ====================================================

        else:

            if self.mode != "KNOWN":

                self.get_logger().info(
                    f"KNOWN AREA -> MAP SOFT LOCK "
                    f"unknown={unknown_ratio:.0%}"
                )

            self.mode = "KNOWN"

            # Map lama jangan di-update lagi.
            # Hanya beam yang jatuh ke cell unknown yang boleh masuk SLAM.
            self.known_counter += 1

            if unknown > 0:

                self.scan_pub.publish(
                    unknown_scan
                )

        self.publish_mode()

    def make_unknown_scan(self, scan):

        unknown_scan = LaserScan()

        unknown_scan.header = scan.header
        unknown_scan.angle_min = scan.angle_min
        unknown_scan.angle_max = scan.angle_max
        unknown_scan.angle_increment = scan.angle_increment
        unknown_scan.time_increment = scan.time_increment
        unknown_scan.scan_time = scan.scan_time
        unknown_scan.range_min = scan.range_min
        unknown_scan.range_max = scan.range_max
        unknown_scan.ranges = [
            float("inf")
        ] * len(scan.ranges)

        if scan.intensities:

            unknown_scan.intensities = [
                0.0
            ] * len(scan.intensities)

        return unknown_scan

    # ========================================================
    # MODE
    # ========================================================

    def publish_mode(self):

        msg = String()

        msg.data = self.mode

        self.mode_pub.publish(
            msg
        )

    def publish_ready(self):

        msg = Bool()

        msg.data = True

        self.ready_pub.publish(
            msg
        )

    # ========================================================
    # ROBOT MEMORY
    # ========================================================

    def update_robot_memory(self):

        pose = self.get_pose()

        if pose is None:
            return

        x, y, yaw = pose

        now = (
            self.get_clock()
            .now()
            .to_msg()
        )

        # ====================================================
        # HOME
        # ====================================================

        if self.home_pose is None:

            home = PoseStamped()

            home.header.frame_id = "map"
            home.header.stamp = now

            home.pose.position.x = x
            home.pose.position.y = y

            home.pose.orientation.z = math.sin(
                yaw / 2
            )

            home.pose.orientation.w = math.cos(
                yaw / 2
            )

            self.home_pose = home

            self.home_pub.publish(
                home
            )

            self.get_logger().info(
                "HOME SAVED"
            )

        # ====================================================
        # PATH INITIAL
        # ====================================================

        if self.last_path_x is None:

            self.last_path_x = x
            self.last_path_y = y

            self.add_path(
                x,
                y,
                yaw,
                now
            )

            return

        dx = x - self.last_path_x
        dy = y - self.last_path_y

        distance = math.hypot(
            dx,
            dy
        )

        if distance < PATH_STEP:
            return

        old_x = self.last_path_x
        old_y = self.last_path_y

        self.add_visited_segment(
            old_x,
            old_y,
            x,
            y,
            now
        )

        self.add_path(
            x,
            y,
            yaw,
            now
        )

        self.last_path_x = x
        self.last_path_y = y

    # ========================================================
    # SAVE PATH
    # ========================================================

    def add_path(
        self,
        x,
        y,
        yaw,
        stamp
    ):

        p = PoseStamped()

        p.header.frame_id = "map"
        p.header.stamp = stamp

        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.position.z = 0.03

        p.pose.orientation.z = math.sin(
            yaw / 2
        )

        p.pose.orientation.w = math.cos(
            yaw / 2
        )

        self.path_points.append(
            p
        )

        # ====================================================
        # PATH PERGI
        # ====================================================

        path = Path()

        path.header.frame_id = "map"
        path.header.stamp = stamp

        path.poses = list(
            self.path_points
        )

        self.path_pub.publish(
            path
        )

        # ====================================================
        # PATH PULANG
        #
        # Sama persis tetapi urutan dibalik.
        # ====================================================

        return_path = Path()

        return_path.header.frame_id = "map"
        return_path.header.stamp = stamp

        return_path.poses = list(
            reversed(
                self.path_points
            )
        )

        self.return_pub.publish(
            return_path
        )

    # ========================================================
    # VISITED CORRIDOR 50CM
    # ========================================================

    def add_visited_segment(
        self,
        x1,
        y1,
        x2,
        y2,
        stamp
    ):

        heading = math.atan2(
            y2 - y1,
            x2 - x1
        )

        half = (
            ROBOT_WIDTH /
            2.0
        )

        nx = (
            -math.sin(heading) *
            half
        )

        ny = (
            math.cos(heading) *
            half
        )

        p1 = Point(
            x=x1 + nx,
            y=y1 + ny,
            z=0.015
        )

        p2 = Point(
            x=x1 - nx,
            y=y1 - ny,
            z=0.015
        )

        p3 = Point(
            x=x2 + nx,
            y=y2 + ny,
            z=0.015
        )

        p4 = Point(
            x=x2 - nx,
            y=y2 - ny,
            z=0.015
        )

        self.visited_triangles.extend(
            [
                p1, p2, p3,
                p3, p2, p4
            ]
        )

        marker = Marker()

        marker.header.frame_id = "map"
        marker.header.stamp = stamp

        marker.ns = "visited"

        marker.id = 0

        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD

        marker.pose.orientation.w = 1.0

        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0

        marker.color.r = 0.10
        marker.color.g = 1.00
        marker.color.b = 0.20
        marker.color.a = 0.28

        marker.points = (
            self.visited_triangles
        )

        self.trail_pub.publish(
            marker
        )


def main():

    rclpy.init()

    node = MemoryNode()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
