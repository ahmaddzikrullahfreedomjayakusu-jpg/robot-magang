#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener


# ============================================================
# SMART HARD MAP MEMORY V2
#
# Prinsip:
# - Navigasi/LiDAR tetap hidup seperti sebelumnya.
# - /scan_slam hanya aktif saat EXPLORE.
# - Saat robot kembali ke koridor lama -> KNOWN/HARD LOCK.
# - Di KNOWN tidak ada scan yang dikirim ke SLAM, sehingga map lama
#   tidak ditulis ulang saat putar balik / kembali lewat jalur lama.
# - Jika robot benar-benar keluar dari koridor lama -> EXPLORE lagi.
#
# Penentu koridor lama menggunakan ODOMETRY (odom->base_footprint),
# bukan map->base, supaya keputusan lock tidak ikut bergeser ketika
# SLAM mengoreksi transform map->odom.
# ============================================================

ROBOT_WIDTH = 0.50
PATH_STEP = 0.05

# Jarak pusat robot ke centerline jalur lama agar dianggap kembali
# ke area yang sudah pernah dilalui.
KNOWN_CORRIDOR_RADIUS_M = 0.38

# Harus benar-benar keluar sedikit lebih jauh agar mapping dibuka lagi.
# Hysteresis ini mencegah KNOWN<->EXPLORE berkedip di batas koridor.
EXIT_CORRIDOR_RADIUS_M = 0.58

# Jangan menganggap titik yang baru saja dilewati sebagai "jalur lama".
# Harus ada separasi panjang lintasan minimal ini.
REVISIT_MIN_PATH_SEPARATION_M = 1.00

# Kondisi harus stabil beberapa saat sebelum mode berpindah.
ENTER_KNOWN_CONFIRM_SEC = 0.45
EXIT_KNOWN_CONFIRM_SEC = 0.55

# Sesudah keluar KNOWN dan kembali EXPLORE, beri jarak minimal sebelum
# diizinkan mengunci lagi. Ini mencegah robot langsung terkunci kembali
# karena masih berada dekat percabangan jalur lama.
EXPLORE_REENTRY_GUARD_M = 0.70

# Sampling pencarian nearest historical path agar tetap ringan.
HISTORY_SEARCH_STEP = 2

# When the wheel encoders confirm an in-place rotation, SLAM input is frozen
# completely. After rotation ends, wait briefly before allowing scans again so
# the LiDAR has settled at the new heading.
ROTATION_MAP_SETTLE_SEC = 0.80

# After any encoder-confirmed U-turn, mapping stays HARD CLOSED until the
# LiDAR sees a genuinely new/unknown corridor. Returning to the same corridor
# must never rewrite the old map, even if left/right swap after the turn.
POST_ROTATION_MIN_TRAVEL_M = 0.25
NEW_AREA_UNKNOWN_RATIO = 0.72
NEW_AREA_CONFIRM_SEC = 0.90
NEW_AREA_SAMPLE_STEP = 6
NEW_AREA_PROBE_MAX_M = 2.50
NEW_AREA_MIN_VALID_SAMPLES = 30

# Once genuinely-new space is confirmed, mapping remains open until the robot
# revisits an old corridor or another rotation explicitly closes it again.


class MemoryNode(Node):

    def __init__(self):
        super().__init__("smart_memory")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ====================================================
        # INPUT
        # ====================================================
        self.create_subscription(
            OccupancyGrid,
            "/map",
            self.map_callback,
            5,
        )

        self.create_subscription(
            LaserScan,
            "/scan_front",
            self.scan_callback,
            20,
        )

        self.create_subscription(
            Bool,
            "/encoder_rotation",
            self.rotation_callback,
            20,
        )

        # ====================================================
        # OUTPUT TO SLAM
        # ====================================================
        self.scan_pub = self.create_publisher(
            LaserScan,
            "/scan_slam",
            20,
        )

        # ====================================================
        # MEMORY / DIAGNOSTIC OUTPUT
        # ====================================================
        self.mode_pub = self.create_publisher(
            String,
            "/map_mode",
            10,
        )

        self.reason_pub = self.create_publisher(
            String,
            "/map_gate_reason",
            10,
        )

        self.ready_pub = self.create_publisher(
            Bool,
            "/smart_memory_ready",
            1,
        )

        self.path_pub = self.create_publisher(
            Path,
            "/robot_path",
            10,
        )

        self.return_pub = self.create_publisher(
            Path,
            "/return_path",
            10,
        )

        self.home_pub = self.create_publisher(
            PoseStamped,
            "/home_pose",
            10,
        )

        # Dipisah dari /odom_trail. Tidak lagi memakai /visited_area agar
        # RViz tidak mendapat dua marker hijau dari frame berbeda.
        self.corridor_pub = self.create_publisher(
            Marker,
            "/known_corridor",
            10,
        )

        # ====================================================
        # STATE
        # ====================================================
        self.map = None
        self.mode = "EXPLORE"
        self.mode_reason = "STARTUP MAPPING"

        self.home_pose = None

        # Path visual pada frame map (dipakai untuk robot_path/return_path).
        self.path_points = []
        self.last_map_path_x = None
        self.last_map_path_y = None

        # Path fisik stabil untuk gate pada frame odom.
        # item = (x, y, cumulative_distance)
        self.odom_history = []
        self.odom_cumulative = 0.0
        self.last_odom_x = None
        self.last_odom_y = None

        # Snapshot jalur lama ketika memasuki KNOWN.
        # Titik yang ditambahkan sesudah lock tidak masuk snapshot ini.
        self.locked_corridor = []

        self.known_candidate_since = None
        self.exit_candidate_since = None
        self.explore_reentry_after_distance = 0.0

        self.encoder_rotating = False
        self.rotation_release_after = 0.0

        self.post_rotation_guard = False
        self.post_rotation_anchor_distance = 0.0
        self.new_area_candidate_since = None
        self.new_area_open = True  # initial mapping is allowed
        self.latest_unknown_ratio = 1.0

        self.create_timer(0.05, self.update_robot_memory)
        self.create_timer(1.0, self.publish_ready)
        self.create_timer(0.25, self.publish_status)

        self.get_logger().info("===================================")
        self.get_logger().info("SMART HARD MAP MEMORY V2 READY")
        self.get_logger().info("EXPLORE = /scan_slam OPEN")
        self.get_logger().info("KNOWN   = /scan_slam HARD LOCK")
        self.get_logger().info("GATE REF = ODOM HISTORICAL ROUTE")
        self.get_logger().info("===================================")

    # ========================================================
    # ENCODER ROTATION GATE
    # ========================================================
    def rotation_callback(self, msg):
        now = time.monotonic()

        if msg.data:
            if not self.encoder_rotating:
                # Rotation starts: close mapping immediately and invalidate any
                # previous permission to keep expanding the map.
                self.post_rotation_guard = True
                self.post_rotation_anchor_distance = self.odom_cumulative
                self.new_area_open = False
                self.new_area_candidate_since = None

            self.encoder_rotating = True
            self.rotation_release_after = now + ROTATION_MAP_SETTLE_SEC
            self.mode_reason = "ENCODER ROTATION -> SLAM HARD FREEZE"
        else:
            if self.encoder_rotating:
                self.rotation_release_after = max(
                    self.rotation_release_after,
                    now + ROTATION_MAP_SETTLE_SEC,
                )
            self.encoder_rotating = False

    def rotation_gate_active(self):
        return (
            self.encoder_rotating or
            time.monotonic() < self.rotation_release_after
        )

    # ========================================================
    # MAP
    # ========================================================
    def map_callback(self, msg):
        self.map = msg

    # ========================================================
    # TF HELPERS
    # ========================================================
    def get_pose(self, target_frame, child="base_footprint"):
        try:
            tf = self.tf_buffer.lookup_transform(
                target_frame,
                child,
                Time(),
            )

            x = tf.transform.translation.x
            y = tf.transform.translation.y
            q = tf.transform.rotation

            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )

            return x, y, yaw
        except Exception:
            return None

    # ========================================================
    # MAP CELL / NEW-AREA CONFIRMATION
    # ========================================================
    def world_to_map(self, x, y):
        if self.map is None:
            return None

        info = self.map.info
        ox = info.origin.position.x
        oy = info.origin.position.y
        q = info.origin.orientation
        oyaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

        dx = x - ox
        dy = y - oy
        c = math.cos(-oyaw)
        ss = math.sin(-oyaw)
        lx = c * dx - ss * dy
        ly = ss * dx + c * dy

        mx = int(lx / info.resolution)
        my = int(ly / info.resolution)
        if mx < 0 or my < 0 or mx >= info.width or my >= info.height:
            return None
        return mx, my

    def map_value(self, x, y):
        cell = self.world_to_map(x, y)
        if cell is None:
            return -1
        mx, my = cell
        idx = my * self.map.info.width + mx
        if idx < 0 or idx >= len(self.map.data):
            return -1
        return self.map.data[idx]

    def scan_unknown_ratio(self, scan):
        """
        Conservative new-area test. We sample points ALONG each LiDAR ray, not
        only the endpoint. If the robot turned around in a known corridor, most
        sampled cells are already known and this ratio stays low. Mapping is
        reopened only when the forward space is genuinely mostly unknown.
        """
        if self.map is None:
            return 1.0, 0

        try:
            tf = self.tf_buffer.lookup_transform(
                "map",
                scan.header.frame_id,
                Time(),
            )
        except Exception:
            return 0.0, 0

        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        q = tf.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

        unknown = 0
        total = 0
        angle = scan.angle_min

        for i, r in enumerate(scan.ranges):
            if i % NEW_AREA_SAMPLE_STEP != 0:
                angle += scan.angle_increment
                continue

            if (
                not math.isfinite(r) or
                r < scan.range_min or
                r > scan.range_max
            ):
                angle += scan.angle_increment
                continue

            probe = min(float(r), NEW_AREA_PROBE_MAX_M)
            world_angle = yaw + angle

            # Ignore the immediate robot footprint. Probe the corridor farther
            # ahead at several depths.
            for frac in (0.35, 0.55, 0.75, 0.95):
                d = probe * frac
                if d < 0.30:
                    continue
                px = tx + d * math.cos(world_angle)
                py = ty + d * math.sin(world_angle)
                total += 1
                if self.map_value(px, py) == -1:
                    unknown += 1

            angle += scan.angle_increment

        if total == 0:
            return 0.0, 0
        return unknown / total, total

    # ========================================================
    # MODE SWITCH
    # ========================================================
    def set_mode(self, new_mode, reason):
        if new_mode == self.mode:
            self.mode_reason = reason
            return

        old = self.mode
        self.mode = new_mode
        self.mode_reason = reason

        self.known_candidate_since = None
        self.exit_candidate_since = None

        if new_mode == "KNOWN":
            self.new_area_open = False
            self.new_area_candidate_since = None
            # Snapshot hanya history lama. Path baru selama mode KNOWN tidak
            # boleh memperpanjang area lock karena itu bisa menutupi area baru.
            cutoff_distance = self.odom_cumulative - REVISIT_MIN_PATH_SEPARATION_M
            self.locked_corridor = [
                (x, y, d)
                for (x, y, d) in self.odom_history
                if d <= cutoff_distance
            ]

            self.get_logger().info(
                f"{old} -> KNOWN | HARD MAP LOCK | "
                f"reference_points={len(self.locked_corridor)} | {reason}"
            )

        else:
            # EXPLORE means the robot may be leaving the old route, but SLAM
            # is still CLOSED until LiDAR proves the new corridor is mostly
            # unknown. This is intentionally conservative.
            self.new_area_open = False
            self.new_area_candidate_since = None
            self.locked_corridor = []
            self.explore_reentry_after_distance = (
                self.odom_cumulative + EXPLORE_REENTRY_GUARD_M
            )

            self.get_logger().info(
                f"{old} -> EXPLORE | MAPPING OPEN | {reason}"
            )

        self.publish_status()

    # ========================================================
    # PATH / CORRIDOR DISTANCE
    # ========================================================
    @staticmethod
    def nearest_distance(x, y, points):
        if not points:
            return float("inf")

        best = float("inf")

        for i in range(0, len(points), HISTORY_SEARCH_STEP):
            px, py, _ = points[i]
            d = math.hypot(x - px, y - py)
            if d < best:
                best = d

        # Pastikan titik terakhir ikut dicek walaupun sampling melompatinya.
        px, py, _ = points[-1]
        best = min(best, math.hypot(x - px, y - py))

        return best

    def historical_points_for_revisit(self):
        cutoff_distance = (
            self.odom_cumulative -
            REVISIT_MIN_PATH_SEPARATION_M
        )

        if cutoff_distance <= 0.0:
            return []

        return [
            p for p in self.odom_history
            if p[2] <= cutoff_distance
        ]

    # ========================================================
    # ODOM HISTORY + HARD LOCK DECISION
    # ========================================================
    def update_odom_memory_and_mode(self):
        pose = self.get_pose("odom")
        if pose is None:
            return

        x, y, _ = pose

        if self.last_odom_x is None:
            self.last_odom_x = x
            self.last_odom_y = y
            self.odom_history.append((x, y, 0.0))
            return

        step = math.hypot(
            x - self.last_odom_x,
            y - self.last_odom_y,
        )

        if step >= PATH_STEP:
            self.odom_cumulative += step
            self.last_odom_x = x
            self.last_odom_y = y
            self.odom_history.append(
                (x, y, self.odom_cumulative)
            )

        now = time.monotonic()

        if self.mode == "EXPLORE":
            # Sesudah baru keluar dari jalur lama, jangan langsung relock.
            if self.odom_cumulative < self.explore_reentry_after_distance:
                self.known_candidate_since = None
                self.mode_reason = "EXPLORE REENTRY GUARD"
                return

            historical = self.historical_points_for_revisit()
            d_old = self.nearest_distance(x, y, historical)

            if d_old <= KNOWN_CORRIDOR_RADIUS_M:
                if self.known_candidate_since is None:
                    self.known_candidate_since = now

                held = now - self.known_candidate_since
                self.mode_reason = (
                    f"OLD ROUTE CANDIDATE d={d_old:.2f}m "
                    f"hold={held:.2f}s"
                )

                if held >= ENTER_KNOWN_CONFIRM_SEC:
                    self.set_mode(
                        "KNOWN",
                        f"REVISIT CONFIRMED d={d_old:.2f}m",
                    )
            else:
                self.known_candidate_since = None
                self.mode_reason = (
                    f"EXPLORING d_old={d_old:.2f}m"
                    if math.isfinite(d_old)
                    else "EXPLORING NO OLD ROUTE"
                )

        else:  # KNOWN
            d_lock = self.nearest_distance(
                x,
                y,
                self.locked_corridor,
            )

            if d_lock <= EXIT_CORRIDOR_RADIUS_M:
                self.exit_candidate_since = None
                self.mode_reason = (
                    f"HARD LOCK d={d_lock:.2f}m"
                )
            else:
                if self.exit_candidate_since is None:
                    self.exit_candidate_since = now

                held = now - self.exit_candidate_since
                self.mode_reason = (
                    f"LEAVING OLD ROUTE d={d_lock:.2f}m "
                    f"hold={held:.2f}s"
                )

                if held >= EXIT_KNOWN_CONFIRM_SEC:
                    self.set_mode(
                        "EXPLORE",
                        f"NEW ROUTE CONFIRMED d={d_lock:.2f}m",
                    )

    # ========================================================
    # SCAN GATE
    # ========================================================
    def scan_callback(self, scan):
        # 1) U-turn / in-place rotation: absolute hard freeze.
        if self.rotation_gate_active():
            self.mode_reason = "ENCODER ROTATION/SETTLE -> /scan_slam CLOSED"
            return

        # 2) Known corridor: NEVER write the map. LiDAR still remains alive on
        # /scan_front for navigation; only SLAM input is closed.
        if self.mode == "KNOWN":
            self.new_area_open = False
            self.new_area_candidate_since = None
            self.mode_reason = "KNOWN CORRIDOR -> HARD MAP LOCK"
            return

        # 3) EXPLORE is not automatically permission to map. After a U-turn or
        # leaving a known corridor, require genuinely empty/unknown space.
        ratio, samples = self.scan_unknown_ratio(scan)
        self.latest_unknown_ratio = ratio
        now = time.monotonic()

        # Initial boot mapping is allowed until we have enough odom history.
        # Once the robot has moved, every future reopen is strict.
        initial_mapping = (
            self.odom_cumulative < REVISIT_MIN_PATH_SEPARATION_M and
            not self.post_rotation_guard
        )

        if initial_mapping:
            self.new_area_open = True
            self.mode_reason = f"INITIAL MAPPING unknown={ratio:.0%}"
            self.scan_pub.publish(scan)
            return

        if self.post_rotation_guard:
            travelled = max(
                0.0,
                self.odom_cumulative - self.post_rotation_anchor_distance,
            )
            if travelled < POST_ROTATION_MIN_TRAVEL_M:
                self.new_area_candidate_since = None
                self.mode_reason = (
                    f"POST UTURN HARD LOCK travel={travelled:.2f}m "
                    f"unknown={ratio:.0%}"
                )
                return

        if self.new_area_open:
            self.mode_reason = f"NEW AREA OPEN unknown={ratio:.0%}"
            self.scan_pub.publish(scan)
            return

        qualifies = (
            samples >= NEW_AREA_MIN_VALID_SAMPLES and
            ratio >= NEW_AREA_UNKNOWN_RATIO
        )

        if qualifies:
            if self.new_area_candidate_since is None:
                self.new_area_candidate_since = now

            held = now - self.new_area_candidate_since
            self.mode_reason = (
                f"NEW AREA CANDIDATE unknown={ratio:.0%} "
                f"hold={held:.2f}s"
            )

            if held >= NEW_AREA_CONFIRM_SEC:
                self.new_area_open = True
                self.post_rotation_guard = False
                self.new_area_candidate_since = None
                self.mode_reason = (
                    f"NEW AREA CONFIRMED unknown={ratio:.0%} -> SLAM OPEN"
                )
                self.scan_pub.publish(scan)
        else:
            self.new_area_candidate_since = None
            self.mode_reason = (
                f"HARD LOCK: NOT EMPTY/NEW ENOUGH unknown={ratio:.0%}"
            )
            return

    # ========================================================
    # MAP PATH OUTPUT
    # ========================================================
    def update_map_path(self):
        pose = self.get_pose("map")
        if pose is None:
            return

        x, y, yaw = pose
        now = self.get_clock().now().to_msg()

        if self.home_pose is None:
            home = PoseStamped()
            home.header.frame_id = "map"
            home.header.stamp = now
            home.pose.position.x = x
            home.pose.position.y = y
            home.pose.orientation.z = math.sin(yaw / 2.0)
            home.pose.orientation.w = math.cos(yaw / 2.0)
            self.home_pose = home
            self.home_pub.publish(home)
            self.get_logger().info("HOME SAVED")

        if self.last_map_path_x is None:
            self.last_map_path_x = x
            self.last_map_path_y = y
            self.add_map_path_point(x, y, yaw, now)
            return

        distance = math.hypot(
            x - self.last_map_path_x,
            y - self.last_map_path_y,
        )

        if distance < PATH_STEP:
            return

        self.last_map_path_x = x
        self.last_map_path_y = y
        self.add_map_path_point(x, y, yaw, now)

    def add_map_path_point(self, x, y, yaw, stamp):
        p = PoseStamped()
        p.header.frame_id = "map"
        p.header.stamp = stamp
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.position.z = 0.03
        p.pose.orientation.z = math.sin(yaw / 2.0)
        p.pose.orientation.w = math.cos(yaw / 2.0)

        self.path_points.append(p)

        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = stamp
        path.poses = list(self.path_points)
        self.path_pub.publish(path)

        return_path = Path()
        return_path.header.frame_id = "map"
        return_path.header.stamp = stamp
        return_path.poses = list(reversed(self.path_points))
        self.return_pub.publish(return_path)

    # ========================================================
    # KNOWN CORRIDOR VISUALIZATION
    #
    # Topic baru /known_corridor. Tidak menimpa /odom_trail dan tidak
    # membuat dua marker berbeda pada /visited_area.
    # ========================================================
    def publish_known_corridor(self):
        if not self.locked_corridor:
            return

        # Transform point odom -> map melalui TF saat ini.
        try:
            tf = self.tf_buffer.lookup_transform(
                "map",
                "odom",
                Time(),
            )
        except Exception:
            return

        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        q = tf.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        c = math.cos(yaw)
        s = math.sin(yaw)

        half = ROBOT_WIDTH / 2.0
        points = []

        transformed = []
        for ox, oy, _ in self.locked_corridor:
            mx = tx + c * ox - s * oy
            my = ty + s * ox + c * oy
            transformed.append((mx, my))

        for i in range(1, len(transformed)):
            x1, y1 = transformed[i - 1]
            x2, y2 = transformed[i]

            dx = x2 - x1
            dy = y2 - y1
            seg = math.hypot(dx, dy)
            if seg < 1e-5:
                continue

            nx = -dy / seg * half
            ny = dx / seg * half

            points.extend([
                Point(x=x1 + nx, y=y1 + ny, z=0.08),
                Point(x=x1 - nx, y=y1 - ny, z=0.08),
                Point(x=x2 + nx, y=y2 + ny, z=0.08),
                Point(x=x2 + nx, y=y2 + ny, z=0.08),
                Point(x=x1 - nx, y=y1 - ny, z=0.08),
                Point(x=x2 - nx, y=y2 - ny, z=0.08),
            ])

        if not points:
            return

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "known_corridor"
        marker.id = 0
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0

        # Cyan transparan supaya tidak tertukar dengan hijau odometry asli.
        marker.color.r = 0.0
        marker.color.g = 0.75
        marker.color.b = 1.0
        marker.color.a = 0.18
        marker.points = points
        self.corridor_pub.publish(marker)

    # ========================================================
    # TIMERS / STATUS
    # ========================================================
    def update_robot_memory(self):
        # During an encoder-confirmed spin, keep path/visual odometry alive
        # elsewhere but do not change KNOWN/EXPLORE classification. This avoids
        # a U-turn being misread as entering a new mapping region.
        if self.rotation_gate_active():
            self.mode_reason = "ENCODER ROTATION -> MODE HOLD + MAP FREEZE"
            self.update_map_path()
            return

        self.update_odom_memory_and_mode()
        self.update_map_path()

        if self.mode == "KNOWN":
            self.publish_known_corridor()

    def publish_status(self):
        mode_msg = String()
        mode_msg.data = self.mode
        self.mode_pub.publish(mode_msg)

        reason_msg = String()
        reason_msg.data = self.mode_reason
        self.reason_pub.publish(reason_msg)

    def publish_ready(self):
        msg = Bool()
        msg.data = True
        self.ready_pub.publish(msg)


def main():
    rclpy.init()
    node = MemoryNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()