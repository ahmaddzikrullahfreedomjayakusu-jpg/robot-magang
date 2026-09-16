#!/usr/bin/env python3

import math
import socket
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster


# ============================================================
# CONFIG
# ============================================================

HP_IP = "192.168.0.148"
PORT = 8888

WHEEL_BASE = 0.50

ENCODER_SIGN = -1.0

# Panah Odometry RViz searah depan robot fisik.
ODOM_DISPLAY_OFFSET = 0.0

# native depan LiDAR merupakan belakang fisik robot
FRONT_LIMIT = math.pi / 2.0

MAX_ENCODER_DELTA = 0.50

TRAIL_STEP = 0.03

ROBOT_ARROW_LENGTH = 0.25

ENCODER_LOG_INTERVAL = 1.0


class RobotNode(Node):

    def __init__(self):

        super().__init__("robot_driver")

        self.odom_pub = self.create_publisher(
            Odometry,
            "/odom",
            20
        )

        self.scan_pub = self.create_publisher(
            LaserScan,
            "/scan_front",
            20
        )

        self.trail_pub = self.create_publisher(
            Marker,
            "/odom_trail",
            10
        )

        self.path_pub = self.create_publisher(
            Path,
            "/odom_path",
            10
        )

        self.body_pub = self.create_publisher(
            Marker,
            "/robot_body",
            10
        )

        self.encoder_ready_pub = self.create_publisher(
            Bool,
            "/encoder_ready",
            1
        )

        self.encoder_debug_pub = self.create_publisher(
            String,
            "/encoder_debug",
            10
        )

        self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            20
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        self.odom_timer = self.create_timer(
            0.10,
            self.publish_stationary_odom
        )

        self.encoder_ready_timer = self.create_timer(
            0.50,
            self.publish_encoder_ready
        )

        # ====================================================
        # POSE
        # ====================================================

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.prev_left = None
        self.prev_right = None
        self.prev_ts = None
        self.encoder_ready = False
        self.last_encoder_log = 0.0

        self.trail_points = []
        self.path_poses = []
        self.last_trail_x = None
        self.last_trail_y = None

        # ====================================================
        # TCP
        # ====================================================

        self.sock = None
        self.running = True

        threading.Thread(
            target=self.receiver_loop,
            daemon=True
        ).start()

        self.get_logger().info("ROBOT NODE READY")

    # ========================================================
    # CONNECT HP
    # ========================================================

    def connect_hp(self):

        while rclpy.ok() and self.running:

            try:

                self.get_logger().info(
                    f"Connecting HP {HP_IP}:{PORT}"
                )

                s = socket.socket(
                    socket.AF_INET,
                    socket.SOCK_STREAM
                )

                s.settimeout(2.0)

                s.connect(
                    (HP_IP, PORT)
                )

                s.settimeout(0.5)

                self.sock = s

                self.get_logger().info(
                    "HP CONNECTED"
                )

                return True

            except Exception as e:

                self.get_logger().warning(
                    f"HP connect gagal: {e}"
                )

                time.sleep(1)

        return False

    # ========================================================
    # PACKET
    # ========================================================

    def parse_packet(self, pkt):

        if len(pkt) != 11:
            return None

        if pkt[0] != 0xBB:
            return None

        if pkt[10] != 0x55:
            return None

        cs = 0

        for b in pkt[1:9]:
            cs ^= b

        if cs != pkt[9]:
            return None

        ts = struct.unpack_from(
            "<I",
            pkt,
            1
        )[0]

        left = struct.unpack_from(
            "<h",
            pkt,
            5
        )[0]

        right = struct.unpack_from(
            "<h",
            pkt,
            7
        )[0]

        return ts, left, right

    # ========================================================
    # RECEIVE
    # ========================================================

    def receiver_loop(self):

        buffer = bytearray()

        while rclpy.ok() and self.running:

            if self.sock is None:

                if not self.connect_hp():
                    continue

                buffer.clear()

            try:

                data = self.sock.recv(256)

                if not data:
                    raise RuntimeError("connection closed")

                buffer.extend(data)

                while len(buffer) >= 11:

                    if buffer[0] != 0xBB:

                        del buffer[0]

                        continue

                    pkt = bytes(
                        buffer[:11]
                    )

                    result = self.parse_packet(pkt)

                    if result is None:

                        del buffer[0]

                        continue

                    del buffer[:11]

                    ts, left, right = result

                    self.encoder_update(
                        ts,
                        left,
                        right
                    )

            except socket.timeout:

                continue

            except Exception as e:

                self.get_logger().warning(
                    f"TCP: {e}"
                )

                try:
                    self.sock.close()
                except Exception:
                    pass

                self.sock = None

                time.sleep(0.5)

    # ========================================================
    # ENCODER ODOM
    # ========================================================

    def encoder_update(
        self,
        ts,
        left_cm,
        right_cm
    ):

        left = (
            ENCODER_SIGN *
            left_cm /
            100.0
        )

        right = (
            ENCODER_SIGN *
            right_cm /
            100.0
        )

        if self.prev_left is None:

            self.prev_left = left
            self.prev_right = right
            self.prev_ts = ts

            return

        dl = left - self.prev_left
        dr = right - self.prev_right

        dt = (
            ts -
            self.prev_ts
        ) / 1000.0

        self.prev_left = left
        self.prev_right = right
        self.prev_ts = ts

        if dt <= 0:
            return

        if (
            abs(dl) > MAX_ENCODER_DELTA or
            abs(dr) > MAX_ENCODER_DELTA
        ):

            self.get_logger().warning(
                "Encoder jump ignored"
            )

            return

        if not self.encoder_ready:

            self.encoder_ready = True

            self.get_logger().info(
                "ENCODER DATA READY"
            )

        ds = (
            dl +
            dr
        ) / 2.0

        dtheta = (
            dr -
            dl
        ) / WHEEL_BASE

        middle = (
            self.theta +
            dtheta / 2
        )

        self.x += (
            ds *
            math.cos(middle)
        )

        self.y += (
            ds *
            math.sin(middle)
        )

        self.theta += dtheta

        self.theta = math.atan2(
            math.sin(self.theta),
            math.cos(self.theta)
        )

        self.publish_odom(
            ds / dt,
            dtheta / dt
        )

        self.publish_encoder_debug(
            ts,
            left_cm,
            right_cm,
            dl,
            dr,
            dt
        )

    def publish_encoder_debug(
        self,
        ts,
        left_cm,
        right_cm,
        dl,
        dr,
        dt
    ):

        now = time.time()

        if now - self.last_encoder_log < ENCODER_LOG_INTERVAL:
            return

        self.last_encoder_log = now

        text = (
            f"ts={ts} "
            f"left_cm={left_cm} "
            f"right_cm={right_cm} "
            f"dl={dl:.3f} "
            f"dr={dr:.3f} "
            f"dt={dt:.3f} "
            f"x={self.x:.3f} "
            f"y={self.y:.3f} "
            f"theta={self.theta:.3f}"
        )

        msg = String()
        msg.data = text
        self.encoder_debug_pub.publish(msg)

        self.get_logger().info(
            f"ENCODER {text}"
        )

    def publish_encoder_ready(self):

        msg = Bool()
        msg.data = self.encoder_ready
        self.encoder_ready_pub.publish(msg)

    # ========================================================
    # ODOM
    # ========================================================

    def publish_stationary_odom(self):

        self.publish_odom(
            0.0,
            0.0
        )

    def publish_odom(self, linear, angular):

        now = (
            self.get_clock()
            .now()
            .to_msg()
        )

        # TF asli untuk SLAM
        qz_tf = math.sin(
            self.theta / 2
        )

        qw_tf = math.cos(
            self.theta / 2
        )

        # visual odometry RViz
        visual_heading = (
            self.theta +
            ODOM_DISPLAY_OFFSET
        )

        qz_vis = math.sin(
            visual_heading / 2
        )

        qw_vis = math.cos(
            visual_heading / 2
        )

        msg = Odometry()

        msg.header.stamp = now
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_footprint"

        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y

        msg.pose.pose.orientation.z = qz_vis
        msg.pose.pose.orientation.w = qw_vis

        msg.twist.twist.linear.x = linear
        msg.twist.twist.angular.z = angular

        self.odom_pub.publish(msg)

        # TF
        t = TransformStamped()

        t.header.stamp = now

        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y

        t.transform.rotation.z = qz_tf
        t.transform.rotation.w = qw_tf

        self.tf_broadcaster.sendTransform(t)

        self.publish_odom_trail(now)

    def publish_odom_trail(self, stamp):

        if self.last_trail_x is None:

            self.last_trail_x = self.x
            self.last_trail_y = self.y

            self.trail_points.append(
                Point(
                    x=self.x,
                    y=self.y,
                    z=0.04
                )
            )

        else:

            distance = math.hypot(
                self.x - self.last_trail_x,
                self.y - self.last_trail_y
            )

            if distance >= TRAIL_STEP:

                self.last_trail_x = self.x
                self.last_trail_y = self.y

                self.trail_points.append(
                    Point(
                        x=self.x,
                        y=self.y,
                        z=0.04
                    )
                )

        marker = Marker()

        marker.header.frame_id = "odom"
        marker.header.stamp = stamp

        marker.ns = "odom_trail"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.04

        marker.color.r = 1.0
        marker.color.g = 0.15
        marker.color.b = 0.05
        marker.color.a = 1.0

        marker.points = list(self.trail_points)

        self.trail_pub.publish(marker)

        self.publish_odom_path(stamp)
        self.publish_robot_body(stamp)

    def publish_odom_path(self, stamp):

        if not self.path_poses:

            self.path_poses.append(
                self.make_pose(stamp)
            )

        elif (
            self.last_trail_x == self.x and
            self.last_trail_y == self.y and
            len(self.path_poses) < len(self.trail_points)
        ):

            self.path_poses.append(
                self.make_pose(stamp)
            )

        path = Path()

        path.header.frame_id = "odom"
        path.header.stamp = stamp
        path.poses = list(self.path_poses)

        self.path_pub.publish(path)

    def make_pose(self, stamp):

        visual_heading = (
            self.theta +
            ODOM_DISPLAY_OFFSET
        )

        pose = PoseStamped()

        pose.header.frame_id = "odom"
        pose.header.stamp = stamp

        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = 0.04

        pose.pose.orientation.z = math.sin(
            visual_heading / 2
        )

        pose.pose.orientation.w = math.cos(
            visual_heading / 2
        )

        return pose

    def publish_robot_body(self, stamp):

        heading = (
            self.theta +
            ODOM_DISPLAY_OFFSET
        )

        start = Point(
            x=self.x,
            y=self.y,
            z=0.08
        )

        end = Point(
            x=self.x + ROBOT_ARROW_LENGTH * math.cos(heading),
            y=self.y + ROBOT_ARROW_LENGTH * math.sin(heading),
            z=0.08
        )

        marker = Marker()

        marker.header.frame_id = "odom"
        marker.header.stamp = stamp

        marker.ns = "robot_body"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.05
        marker.scale.y = 0.10
        marker.scale.z = 0.10

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.points = [
            start,
            end
        ]

        self.body_pub.publish(marker)

    # ========================================================
    # FRONT LIDAR
    # ========================================================

    def scan_callback(self, msg):

        out = LaserScan()

        out.header = msg.header

        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max

        out.angle_increment = msg.angle_increment

        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time

        out.range_min = msg.range_min
        out.range_max = msg.range_max

        out.ranges = list(msg.ranges)

        if msg.intensities:
            out.intensities = list(
                msg.intensities
            )

        angle = msg.angle_min

        for i in range(len(out.ranges)):

            a = math.atan2(
                math.sin(angle),
                math.cos(angle)
            )

            # Hilangkan belakang fisik robot
            if abs(a) < FRONT_LIMIT:

                out.ranges[i] = float("inf")

                if (
                    out.intensities and
                    i < len(out.intensities)
                ):

                    out.intensities[i] = 0.0

            angle += msg.angle_increment

        self.scan_pub.publish(out)

    # ========================================================

    def destroy_node(self):

        self.running = False

        try:

            if self.sock:
                self.sock.close()

        except Exception:
            pass

        super().destroy_node()


def main():

    rclpy.init()

    node = RobotNode()

    try:

        rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):

        pass

    if rclpy.ok():
        node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
