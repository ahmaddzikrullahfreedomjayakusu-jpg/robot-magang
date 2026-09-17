#!/usr/bin/env python3

# Variant of robot1.py that talks to the STM32 over a USB cable plugged
# straight into this laptop, instead of relaying through the phone
# (stm_bridge.py) over TCP/WiFi. Everything else -- odom, TF, encoder math,
# the 5/11-byte packet formats -- is identical to robot1.py, since the
# STM32 doesn't care whether the bytes arrived over the phone's serial link
# or this one; only the transport in between changed.
#
# robot1.py itself is untouched and still the one to use for manual/SLAM
# mapping (phone stays handy while walking the robot around, no cable to
# manage). Use this one only when the laptop is USB-tethered directly to
# the robot, e.g. for the reactive coverage run.

import math
import os
import struct
import threading
import time

import rclpy
import serial
import serial.tools.list_ports
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String, Int16MultiArray
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster


# ============================================================
# CONFIG
# ============================================================

# Auto-detected at connect time: whatever's plugged in, minus the RPLidar
# (a known Silicon Labs CP210x, VID:PID 10C4:EA60) -- no manual "ls" + edit
# needed, just plug the STM32's USB cable into the laptop. Set STM32_USB_PORT
# to skip auto-detect and force a specific port instead.
SERIAL_PORT_OVERRIDE = os.environ.get("STM32_USB_PORT", "")
LIDAR_VID_PID = "10C4:EA60"  # Silicon Labs CP210x -- exclude this from auto-detect
BAUD = 115200


def autodetect_stm32_port():
    """Pick the first USB-serial device that isn't the RPLidar."""
    if SERIAL_PORT_OVERRIDE:
        return SERIAL_PORT_OVERRIDE

    candidates = []
    for port in serial.tools.list_ports.comports():
        if port.vid is None:
            continue  # not a real USB device (e.g. onboard ttyS* ports)
        hwid_vid_pid = f"{port.vid:04X}:{port.pid:04X}" if port.pid is not None else ""
        if hwid_vid_pid == LIDAR_VID_PID:
            continue
        candidates.append(port.device)

    if candidates:
        return sorted(candidates)[0]
    return None

WHEEL_BASE = 0.50

ENCODER_SIGN = -1.0

# Panah Odometry RViz searah depan robot fisik.
ODOM_DISPLAY_OFFSET = 0.0

# native depan LiDAR merupakan belakang fisik robot
FRONT_LIMIT = math.pi / 2.0

MAX_ENCODER_DELTA = 0.50

TRAIL_STEP = 0.05

ROBOT_BODY_WIDTH = 0.50
ROBOT_BODY_LENGTH = 0.50
ROBOT_CAR_WIDTH = 0.28
ROBOT_CAR_LENGTH = 0.42
ROBOT_CAR_HEIGHT = 0.12
TRAIL_Z = 0.035

ENCODER_LOG_INTERVAL = 1.0
MOTOR_LOG_INTERVAL = 0.5
ENCODER_TIMEOUT_SEC = 1.5

# Encoder motion gate for SLAM collaboration.
# In-place rotation is detected from REAL wheel encoder movement, not from
# camera/LiDAR estimation. During this state memory.py will freeze /scan_slam.
ROTATION_MIN_WHEEL_MOVE_M = 0.003
ROTATION_DS_RATIO_MAX = 0.35
ROTATION_HOLD_SEC = 0.35

# Collaboration with autonomous motor command. The command only confirms the
# intended maneuver; encoder distance remains the source of pose/yaw.
MOTOR_NEUTRAL_VALUE = 127
SPIN_CMD_DEADBAND = 12
SPIN_CMD_RECENT_SEC = 0.60

# During an encoder-confirmed in-place spin, never integrate tiny wheel
# mismatch as x/y translation. This prevents the odom trail from "walking"
# sideways during a U-turn.
FORCE_ZERO_TRANSLATION_DURING_SPIN = True


class RobotNode(Node):

    def __init__(self):

        super().__init__("robot_driver")

        marker_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )

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
            marker_qos
        )

        self.visual_marker_pub = self.create_publisher(
            Marker,
            "/visualization_marker",
            marker_qos
        )

        self.footprint_trail_pub = self.create_publisher(
            Marker,
            "/robot_footprint_trail",
            marker_qos
        )

        self.footprint_blocks_pub = self.create_publisher(
            Marker,
            "/robot_footprint_blocks",
            marker_qos
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

        self.rotation_pub = self.create_publisher(
            Bool,
            "/encoder_rotation",
            10
        )

        self.motion_pub = self.create_publisher(
            String,
            "/encoder_motion",
            10
        )

        self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            20
        )

        self.create_subscription(
            Int16MultiArray,
            "/motor_rpm",
            self.motor_callback,
            10
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

        self.rotation_timer = self.create_timer(
            0.10,
            self.publish_rotation_state
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
        self.last_encoder_rx = 0.0
        self.last_encoder_log = 0.0
        self.last_motor_log = 0.0
        self.sock_lock = threading.Lock()

        self.last_cmd_left = MOTOR_NEUTRAL_VALUE
        self.last_cmd_right = MOTOR_NEUTRAL_VALUE
        self.last_cmd_time = 0.0

        self.encoder_rotating = False
        self.last_rotation_motion_time = 0.0
        self.rotation_accum_rad = 0.0

        self.trail_points = []
        self.trail_triangles = []
        self.path_poses = []
        self.last_trail_x = None
        self.last_trail_y = None

        # ====================================================
        # USB SERIAL (direct to STM32, no phone relay)
        # ====================================================

        self.ser = None
        self.running = True

        threading.Thread(
            target=self.receiver_loop,
            daemon=True
        ).start()

        self.get_logger().info("ROBOT NODE READY (direct USB)")

    # ========================================================
    # CONNECT SERIAL
    # ========================================================

    def connect_serial(self):

        while rclpy.ok() and self.running:

            port = autodetect_stm32_port()

            if port is None:
                self.get_logger().warning(
                    "Tidak ada USB serial selain LiDAR terdeteksi -- "
                    "colokkan kabel STM32 ke laptop, cek: ls /dev/ttyUSB* /dev/ttyACM*"
                )
                time.sleep(1.0)
                continue

            try:

                self.get_logger().info(
                    f"Connecting serial {port} @ {BAUD}"
                )

                s = serial.Serial(
                    port=port,
                    baudrate=BAUD,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=0.05,
                    write_timeout=1.0,
                )

                self.ser = s

                self.get_logger().info(
                    f"SERIAL CONNECTED {port}"
                )

                return True

            except Exception as e:

                self.get_logger().warning(
                    f"Serial connect gagal {port}: {e}"
                )

                time.sleep(0.5)

        return False


    # ========================================================
    # MOTOR COMMAND
    # ========================================================

    def make_motor_packet(self, kiri, kanan):

        kiri = max(0, min(255, int(kiri)))
        kanan = max(0, min(255, int(kanan)))
        checksum = (0xAA + kanan + kiri) & 0xFF

        return bytes([
            0xAA,
            kanan,
            kiri,
            checksum,
            0x55
        ])

    def motor_callback(self, msg):

        if len(msg.data) < 2:
            return

        kiri = int(msg.data[0])
        kanan = int(msg.data[1])

        # Keep the latest autonomous command only as maneuver intent.
        # Encoder movement remains the actual odometry source.
        self.last_cmd_left = kiri
        self.last_cmd_right = kanan
        self.last_cmd_time = time.monotonic()

        packet = self.make_motor_packet(kiri, kanan)
        now = time.time()

        if self.ser is None:
            self.get_logger().warning(
                "MOTOR: serial belum connected, command diabaikan"
            )
            return

        with self.sock_lock:
            try:
                self.ser.write(packet)
                if now - self.last_motor_log >= MOTOR_LOG_INTERVAL:
                    self.last_motor_log = now
                    self.get_logger().info(
                        f"MOTOR CMD kiri={kiri} kanan={kanan}"
                    )
            except Exception as e:
                self.get_logger().warning(
                    f"MOTOR send gagal: {e}"
                )

                try:
                    self.ser.close()
                except Exception:
                    pass

                self.ser = None

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

            if self.ser is None:

                if not self.connect_serial():
                    continue

                buffer.clear()

            try:

                with self.sock_lock:
                    data = self.ser.read(256)

                if not data:
                    # timeout=0.05 expiring with nothing to read is the
                    # normal case for serial (unlike a TCP recv() returning
                    # empty, which means the connection closed) -- just
                    # loop and try again, don't treat it as a disconnect.
                    continue

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

            except Exception as e:

                self.get_logger().warning(
                    f"Serial: {e}"
                )

                try:
                    self.ser.close()
                except Exception:
                    pass

                self.ser = None
                self.encoder_ready = False

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

        self.last_encoder_rx = time.time()

        ds = (
            dl +
            dr
        ) / 2.0

        # Robot fisiknya jalan terbalik dari desain (sama alasan seperti
        # LiDAR menghadap belakang), jadi channel encoder kiri/kanan yang
        # terbaca ketuker dari sisi fisik kiri/kanan yang sebenarnya.
        # Dibalik di sini supaya arah putar sesuai kenyataan.
        dtheta = (
            dl -
            dr
        ) / WHEEL_BASE

        # ====================================================
        # REAL ENCODER ROTATION DETECTION
        # ====================================================
        # Autonomous spin-right command is left wheel forward + right wheel
        # reverse (97,157); spin-left is the opposite. We do NOT trust the
        # command itself here: the encoder confirms whether the wheels really
        # rotated in opposite directions.
        wheel_motion = abs(dl) + abs(dr)
        opposite_wheels = (dl * dr) < 0.0
        near_zero_translation = abs(ds) <= max(
            0.002,
            ROTATION_DS_RATIO_MAX * max(abs(dl), abs(dr))
        )

        cmd_recent = (
            time.monotonic() - self.last_cmd_time
        ) <= SPIN_CMD_RECENT_SEC
        cmd_left_delta = self.last_cmd_left - MOTOR_NEUTRAL_VALUE
        cmd_right_delta = self.last_cmd_right - MOTOR_NEUTRAL_VALUE
        command_requests_spin = (
            cmd_recent and
            abs(cmd_left_delta) >= SPIN_CMD_DEADBAND and
            abs(cmd_right_delta) >= SPIN_CMD_DEADBAND and
            (cmd_left_delta * cmd_right_delta) < 0
        )

        encoder_confirms_spin = (
            opposite_wheels and
            wheel_motion >= ROTATION_MIN_WHEEL_MOVE_M and
            near_zero_translation
        )

        # Collaboration: autonomous command says "I intend to spin", while
        # encoders must still show real opposite-wheel movement.
        rotating_now = (
            encoder_confirms_spin and
            (command_requests_spin or near_zero_translation)
        )

        if rotating_now:
            self.encoder_rotating = True
            self.last_rotation_motion_time = time.monotonic()
            self.rotation_accum_rad += dtheta

        # Critical U-turn correction: unequal wheel travel during an in-place
        # spin must NOT become fake x/y motion. Let encoders update theta only.
        ds_pose = 0.0 if (
            FORCE_ZERO_TRANSLATION_DURING_SPIN and rotating_now
        ) else ds

        middle = (
            self.theta +
            dtheta / 2
        )

        self.x += (
            ds_pose *
            math.cos(middle)
        )

        self.y += (
            ds_pose *
            math.sin(middle)
        )

        self.theta += dtheta

        self.theta = math.atan2(
            math.sin(self.theta),
            math.cos(self.theta)
        )

        self.publish_odom(
            ds_pose / dt,
            dtheta / dt,
            update_trail=True
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

    def publish_rotation_state(self):
        # Keep rotation TRUE briefly between sparse encoder packets so the
        # SLAM gate cannot flicker ON/OFF during an in-place turn.
        if (
            self.encoder_rotating and
            time.monotonic() - self.last_rotation_motion_time > ROTATION_HOLD_SEC
        ):
            self.encoder_rotating = False

        b = Bool()
        b.data = bool(self.encoder_rotating)
        self.rotation_pub.publish(b)

        status = String()
        if self.encoder_rotating:
            direction = "LEFT" if self.rotation_accum_rad > 0.0 else "RIGHT"
            degrees = math.degrees(abs(self.rotation_accum_rad))
            status.data = f"ROTATING_{direction} encoder_yaw={degrees:.1f}deg MAP_FREEZE"
        else:
            status.data = "TRANSLATION_OR_STOP MAP_GATE_NORMAL"
            # New rotation starts counting from zero.
            self.rotation_accum_rad = 0.0
        self.motion_pub.publish(status)

    def publish_encoder_ready(self):

        msg = Bool()
        if (
            self.encoder_ready and
            self.last_encoder_rx > 0.0 and
            time.time() - self.last_encoder_rx > ENCODER_TIMEOUT_SEC
        ):
            self.encoder_ready = False

        msg.data = self.encoder_ready
        self.encoder_ready_pub.publish(msg)

    # ========================================================
    # ODOM
    # ========================================================

    def publish_stationary_odom(self):

        self.publish_odom(
            0.0,
            0.0,
            update_trail=False
        )

    def publish_odom(self, linear, angular, update_trail=True):

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

        # Do not republish the complete growing trail from the 10 Hz
        # stationary timer. Encoder packets update trail/path; the body marker
        # still follows theta in real time.
        if update_trail:
            self.publish_odom_trail(now)
        else:
            self.publish_robot_body(now)

    def publish_odom_trail(self, stamp):

        if self.last_trail_x is None:

            self.last_trail_x = self.x
            self.last_trail_y = self.y

            self.trail_points.append(
                Point(
                    x=self.x,
                    y=self.y,
                    z=TRAIL_Z
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
                        z=TRAIL_Z
                    )
                )

        self.rebuild_trail_triangles()

        marker = Marker()

        marker.header.frame_id = "odom"
        marker.header.stamp = stamp

        marker.ns = "odom_trail_footprint"
        marker.id = 0
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD

        marker.pose.orientation.w = 1.0

        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.78

        marker.points = list(self.trail_triangles)

        self.trail_pub.publish(marker)
        self.visual_marker_pub.publish(marker)
        self.footprint_trail_pub.publish(marker)

        block_marker = Marker()

        block_marker.header.frame_id = "odom"
        block_marker.header.stamp = stamp

        block_marker.ns = "robot_footprint_blocks"
        block_marker.id = 0
        block_marker.type = Marker.CUBE_LIST
        block_marker.action = Marker.ADD

        block_marker.pose.orientation.w = 1.0

        block_marker.scale.x = ROBOT_BODY_LENGTH
        block_marker.scale.y = ROBOT_BODY_WIDTH
        block_marker.scale.z = 0.03

        block_marker.color.r = 0.0
        block_marker.color.g = 1.0
        block_marker.color.b = 0.0
        block_marker.color.a = 0.16

        block_marker.points = list(self.trail_points)

        self.footprint_blocks_pub.publish(block_marker)

        self.publish_odom_path(stamp)
        self.publish_robot_body(stamp)

    def rebuild_trail_triangles(self):

        self.trail_triangles = []

        if len(self.trail_points) == 1:
            center = self.trail_points[0]
            half_x = ROBOT_BODY_LENGTH / 2.0
            half_y = ROBOT_BODY_WIDTH / 2.0

            p1 = Point(
                x=center.x - half_x,
                y=center.y - half_y,
                z=TRAIL_Z
            )

            p2 = Point(
                x=center.x + half_x,
                y=center.y - half_y,
                z=TRAIL_Z
            )

            p3 = Point(
                x=center.x - half_x,
                y=center.y + half_y,
                z=TRAIL_Z
            )

            p4 = Point(
                x=center.x + half_x,
                y=center.y + half_y,
                z=TRAIL_Z
            )

            self.trail_triangles.extend(
                [
                    p1, p2, p3,
                    p3, p2, p4,
                    p1, p3, p2,
                    p3, p4, p2
                ]
            )

            return

        if len(self.trail_points) < 2:
            return

        half = ROBOT_BODY_WIDTH / 2.0

        for i in range(len(self.trail_points) - 1):

            a = self.trail_points[i]
            b = self.trail_points[i + 1]

            dx = b.x - a.x
            dy = b.y - a.y
            length = math.hypot(dx, dy)

            if length < 0.001:
                continue

            nx = -dy / length * half
            ny = dx / length * half

            p1 = Point(
                x=a.x + nx,
                y=a.y + ny,
                z=TRAIL_Z
            )

            p2 = Point(
                x=a.x - nx,
                y=a.y - ny,
                z=TRAIL_Z
            )

            p3 = Point(
                x=b.x + nx,
                y=b.y + ny,
                z=TRAIL_Z
            )

            p4 = Point(
                x=b.x - nx,
                y=b.y - ny,
                z=TRAIL_Z
            )

            self.trail_triangles.extend(
                [
                    p1, p2, p3,
                    p3, p2, p4,
                    p1, p3, p2,
                    p3, p4, p2
                ]
            )

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

        marker = Marker()

        marker.header.frame_id = "odom"
        marker.header.stamp = stamp

        marker.ns = "robot_body"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose.position.x = self.x
        marker.pose.position.y = self.y
        marker.pose.position.z = TRAIL_Z + ROBOT_CAR_HEIGHT / 2.0

        marker.pose.orientation.z = math.sin(
            heading / 2
        )

        marker.pose.orientation.w = math.cos(
            heading / 2
        )

        marker.scale.x = ROBOT_CAR_LENGTH
        marker.scale.y = ROBOT_CAR_WIDTH
        marker.scale.z = ROBOT_CAR_HEIGHT

        marker.color.r = 0.05
        marker.color.g = 0.55
        marker.color.b = 1.0
        marker.color.a = 1.0

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

            if self.ser:
                self.ser.close()

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