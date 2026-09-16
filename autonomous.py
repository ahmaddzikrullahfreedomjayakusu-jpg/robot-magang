
BUILD_ID = "EDGE_LIDAR_WEB_AUTO_MANUAL_2026_09_07"
import os
import math
import time
import json
import signal
import threading
import socket
import subprocess
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse
import serial
import serial.tools.list_ports

import cv2
import numpy as np
from ultralytics import YOLO

try:
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry, Path
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Int16MultiArray
except Exception:
    rclpy = None
    Node = None
    Odometry = None
    Path = None
    LaserScan = None
    Int16MultiArray = None

# ============================================================
# FREEDOM - LAPTOP + HP MJPEG + BODY ROBOT + LOGIKA GARIS
# PROCESSING LAPTOP + YOLO SEGMENTATION + LOCAL SPACE CONTROL
# ============================================================

HP_IP = "192.168.0.147"
MJPEG_URL = "http://{}:8080/mjpeg".format(HP_IP)

FRAME_W = 640
FRAME_H = 480
FRAME_BYTES = FRAME_W * FRAME_H * 3

# ============================================================
# YOLO SEGMENTATION - OBSTACLE MENYENTUH MONCONG ROBOT
#
# Tidak memakai kotak ROI ungu.
# Zona bahaya = ZONA MERAH body robot (1/4 bagian paling depan).
# Trigger hanya jika segmentation mask object overlap zona merah.
# ============================================================
YOLO_MODEL = "yolo11n-seg.pt"
YOLO_IMGSZ = 416
YOLO_CONF = 0.20
YOLO_EVERY_N_FRAMES = 1

# ============================================================
# CAMERA BODY GUARD
# Body trapezoid pada layar = footprint robot.
# Kamera mulai koreksi SEBELUM mask object masuk body.
# ============================================================
CAM_BODY_WARN_SCALE = 1.26
CAM_BODY_HARD_SCALE = 1.12

CAM_BODY_WARN_MIN_PX = 18
CAM_BODY_HARD_MIN_PX = 32
CAM_BODY_CENTER_STOP_PX = 70
CAM_BODY_SELF_RATIO_IGNORE = 0.55

CAM_BODY_SOFT_STEER = 22.0
CAM_BODY_HARD_STEER = 60.0
CAM_BODY_EMERGENCY_STEER = 78.0

_cam_guard_prev = 0.0
CAM_BODY_GUARD_ALPHA = 0.72

yolo_model = None
yolo_ready = False

yolo_input_lock = threading.Lock()
yolo_result_lock = threading.Lock()
yolo_input_frame = None
yolo_last_result = None
yolo_last_infer_ms = 0.0
yolo_frame_counter = 0


def yolo_worker_loop():
    global yolo_input_frame
    global yolo_last_result
    global yolo_last_infer_ms
    global yolo_model
    global yolo_ready

    print("[YOLO] Loading {}...".format(YOLO_MODEL), flush=True)

    try:
        yolo_model = YOLO(YOLO_MODEL)
        yolo_ready = True
        print("[YOLO] SEGMENTATION READY", flush=True)
    except Exception as e:
        print("[YOLO] GAGAL LOAD:", e, flush=True)
        return

    while True:
        frame_for_yolo = None

        with yolo_input_lock:
            if yolo_input_frame is not None:
                frame_for_yolo = yolo_input_frame
                yolo_input_frame = None

        if frame_for_yolo is None:
            time.sleep(0.002)
            continue

        try:
            t0 = time.perf_counter()

            result = yolo_model(
                frame_for_yolo,
                imgsz=YOLO_IMGSZ,
                conf=YOLO_CONF,
                device="cpu",
                verbose=False
            )[0]

            infer_ms = (time.perf_counter() - t0) * 1000.0

            with yolo_result_lock:
                yolo_last_result = result
                yolo_last_infer_ms = infer_ms

        except Exception as e:
            print("[YOLO] ERROR:", e)



def scale_polygon_about_center(points, scale):
    pts = np.asarray(
        points,
        dtype=np.float32
    )

    center = np.mean(
        pts,
        axis=0
    )

    out = (
        center +
        (pts - center) * float(scale)
    )

    out[:, 0] = np.clip(
        out[:, 0],
        0,
        FRAME_W - 1
    )

    out[:, 1] = np.clip(
        out[:, 1],
        0,
        FRAME_H - 1
    )

    return out.astype(np.int32)


def build_camera_body_guard_masks(body_points):
    """
    Buat 3 level:
      body  = footprint asli
      hard  = sedikit lebih besar
      warn  = lebih besar lagi

    Juga split kiri/tengah/kanan untuk arah koreksi.
    """
    body_poly = np.array(
        body_points,
        dtype=np.float32
    )

    hard_poly = scale_polygon_about_center(
        body_poly,
        CAM_BODY_HARD_SCALE
    )

    warn_poly = scale_polygon_about_center(
        body_poly,
        CAM_BODY_WARN_SCALE
    )

    def make_mask(poly):
        m = np.zeros(
            (FRAME_H, FRAME_W),
            dtype=np.uint8
        )

        cv2.fillPoly(
            m,
            [poly.astype(np.int32)],
            255
        )

        return m > 0

    body_mask = make_mask(
        body_poly.astype(np.int32)
    )

    hard_mask = make_mask(
        hard_poly
    )

    warn_mask = make_mask(
        warn_poly
    )

    # warning ring only, hard ring only
    warn_ring = (
        warn_mask &
        (~hard_mask)
    )

    hard_ring = (
        hard_mask &
        (~body_mask)
    )

    # split by image/body center
    xs = body_poly[:, 0]
    body_cx = float(
        np.mean(xs)
    )

    x_left = int(
        body_cx -
        (np.max(xs) - np.min(xs)) * 0.12
    )

    x_right = int(
        body_cx +
        (np.max(xs) - np.min(xs)) * 0.12
    )

    xx = np.arange(
        FRAME_W
    )[None, :]

    left_half = (
        xx < x_left
    )

    center_half = (
        (xx >= x_left) &
        (xx <= x_right)
    )

    right_half = (
        xx > x_right
    )

    return {
        "body": body_mask,
        "hard": hard_mask,
        "warn": warn_mask,
        "warn_ring": warn_ring,
        "hard_ring": hard_ring,
        "left": np.broadcast_to(
            left_half,
            (FRAME_H, FRAME_W)
        ),
        "center": np.broadcast_to(
            center_half,
            (FRAME_H, FRAME_W)
        ),
        "right": np.broadcast_to(
            right_half,
            (FRAME_H, FRAME_W)
        ),
        "warn_poly": warn_poly,
        "hard_poly": hard_poly,
    }


def camera_body_guard(
    result,
    frame_draw,
    guard_masks,
    preferred_steer=0.0
):
    """
    CAMERA collision guard berbasis segmentation mask.

    Return:
      action: NONE / GUARD_LEFT / GUARD_RIGHT / STOP
      steer : koreksi kamera
      level : NONE / WARN / HARD / BODY
      label
    """
    global _cam_guard_prev

    raw_steer = 0.0
    level = "NONE"
    label_out = "-"
    action = "NONE"

    if (
        result is None or
        result.masks is None or
        result.boxes is None
    ):
        _cam_guard_prev *= CAM_BODY_GUARD_ALPHA
        return (
            action,
            float(_cam_guard_prev),
            level,
            label_out,
            0,
            0,
            0
        )

    polygons = result.masks.xy
    boxes = result.boxes
    names = result.names

    total_left = 0
    total_center = 0
    total_right = 0

    hard_total = 0
    warn_total = 0
    body_total = 0

    n = min(
        len(polygons),
        len(boxes)
    )

    for i in range(n):
        poly = np.asarray(
            polygons[i],
            dtype=np.float32
        )

        if (
            poly.ndim != 2 or
            poly.shape[0] < 3
        ):
            continue

        poly_i = np.round(
            poly
        ).astype(np.int32)

        poly_i[:, 0] = np.clip(
            poly_i[:, 0],
            0,
            FRAME_W - 1
        )

        poly_i[:, 1] = np.clip(
            poly_i[:, 1],
            0,
            FRAME_H - 1
        )

        mask_u8 = np.zeros(
            (FRAME_H, FRAME_W),
            dtype=np.uint8
        )

        cv2.fillPoly(
            mask_u8,
            [poly_i],
            255
        )

        mask = (
            mask_u8 > 0
        )
        mask_area = int(
            np.count_nonzero(mask)
        )

        if mask_area <= 0:
            continue

        warn_px = int(
            np.count_nonzero(
                mask &
                guard_masks["warn_ring"]
            )
        )

        hard_px = int(
            np.count_nonzero(
                mask &
                guard_masks["hard_ring"]
            )
        )

        body_px = int(
            np.count_nonzero(
                mask &
                guard_masks["body"]
            )
        )

        self_ratio = body_px / max(1, mask_area)

        if (
            self_ratio >= CAM_BODY_SELF_RATIO_IGNORE and
            hard_px < CAM_BODY_HARD_MIN_PX
        ):
            continue

        if (
            warn_px <
            CAM_BODY_WARN_MIN_PX and
            hard_px <
            CAM_BODY_HARD_MIN_PX and
            body_px <
            CAM_BODY_HARD_MIN_PX
        ):
            continue

        cls_id = int(
            boxes.cls[i].item()
        )

        if isinstance(names, dict):
            label = names.get(
                cls_id,
                str(cls_id)
            )
        else:
            label = str(cls_id)

        label_out = label

        active_mask = (
            mask &
            (
                guard_masks["warn_ring"] |
                guard_masks["hard_ring"]
            )
        )

        total_left += int(
            np.count_nonzero(
                active_mask &
                guard_masks["left"]
            )
        )

        total_center += int(
            np.count_nonzero(
                active_mask &
                guard_masks["center"]
            )
        )

        total_right += int(
            np.count_nonzero(
                active_mask &
                guard_masks["right"]
            )
        )

        warn_total += warn_px
        hard_total += hard_px
        body_total += body_px

    # Nothing close to body
    if (
        warn_total == 0 and
        hard_total == 0 and
        body_total == 0
    ):
        _cam_guard_prev *= CAM_BODY_GUARD_ALPHA

        return (
            "NONE",
            float(_cam_guard_prev),
            "NONE",
            "-",
            0,
            0,
            0
        )

    # Very close center / hard ring overlap -> STOP.
    # Body interior itself is ignored so robot body is not detected.
    if (
        hard_total >= CAM_BODY_HARD_MIN_PX and
        total_center >= CAM_BODY_CENTER_STOP_PX
    ):
        _cam_guard_prev *= 0.5

        return (
            "STOP",
            0.0,
            "BODY",
            label_out,
            total_left,
            total_center,
            total_right
        )

    # Choose correction direction
    if total_left > total_right:
        direction = "RIGHT"
        sign = +1.0
    elif total_right > total_left:
        direction = "LEFT"
        sign = -1.0
    else:
        # If center dominant, follow the current edge preference.
        direction = (
            "LEFT"
            if preferred_steer < 0.0
            else "RIGHT"
        )
        sign = (
            -1.0
            if direction == "LEFT"
            else +1.0
        )

    if (
        hard_total >= CAM_BODY_CENTER_STOP_PX
    ):
        level = "HARD"
        raw_steer = (
            sign *
            CAM_BODY_EMERGENCY_STEER
        )
    elif (
        hard_total >= CAM_BODY_HARD_MIN_PX
    ):
        level = "HARD"
        raw_steer = (
            sign *
            CAM_BODY_HARD_STEER
        )
    else:
        level = "WARN"
        raw_steer = (
            sign *
            CAM_BODY_SOFT_STEER
        )

    _cam_guard_prev = (
        CAM_BODY_GUARD_ALPHA *
        _cam_guard_prev +
        (1.0 - CAM_BODY_GUARD_ALPHA) *
        raw_steer
    )

    action = (
        "GUARD_RIGHT"
        if _cam_guard_prev > 0
        else "GUARD_LEFT"
    )

    return (
        action,
        float(_cam_guard_prev),
        level,
        label_out,
        total_left,
        total_center,
        total_right
    )


def build_red_zone_masks(width_px, height_px, body_points):
    """
    YOLO hanya boleh mempengaruhi kontrol jika object menyentuh
    ZONA MERAH body robot (1/4 bagian paling depan).

    Zona merah dibagi 3:
      left   = kiri merah
      center = tengah merah
      right  = kanan merah
    """
    kiri_atas, kanan_atas, kanan_bawah, kiri_bawah = [
        np.asarray(p, dtype=np.float32) for p in body_points
    ]

    # ZONA MERAH = 25% bagian paling depan body.
    RED_DEPTH = 0.25

    left_red_bottom = lerp_point(
        kiri_atas,
        kiri_bawah,
        RED_DEPTH
    )

    right_red_bottom = lerp_point(
        kanan_atas,
        kanan_bawah,
        RED_DEPTH
    )

    def mix(a, b, f):
        return (1.0 - f) * a + f * b

    # 32.5% kiri, 35% tengah, 32.5% kanan.
    top_l1 = mix(kiri_atas, kanan_atas, 0.325)
    top_l2 = mix(kiri_atas, kanan_atas, 0.675)

    bot_l1 = mix(left_red_bottom, right_red_bottom, 0.325)
    bot_l2 = mix(left_red_bottom, right_red_bottom, 0.675)

    polys = {
        "left": np.array(
            [kiri_atas, top_l1, bot_l1, left_red_bottom],
            dtype=np.int32
        ),
        "center": np.array(
            [top_l1, top_l2, bot_l2, bot_l1],
            dtype=np.int32
        ),
        "right": np.array(
            [top_l2, kanan_atas, right_red_bottom, bot_l2],
            dtype=np.int32
        ),
    }

    masks = {}

    for name, poly in polys.items():
        m = np.zeros(
            (height_px, width_px),
            dtype=np.uint8
        )
        cv2.fillPoly(
            m,
            [poly],
            255
        )
        masks[name] = m > 0

    return masks


def yolo_segmentation_obstacle(result, frame_draw, zone_masks):
    """
    Kontur YOLO tetap digambar untuk semua object.

    Tetapi action hanya aktif bila segmentation mask object
    overlap dengan ZONA MERAH body.

    Return:
      NONE        = object belum menyentuh merah
      AVOID_LEFT  = obstacle di kanan merah -> hindar kiri
      AVOID_RIGHT = obstacle di kiri merah -> hindar kanan
      TURN_BACK   = obstacle dominan di tengah merah
    """

    SIDE_MIN_OVERLAP = 35
    CENTER_MIN_OVERLAP = 45
    CENTER_DOMINANCE = 1.30

    total_left = 0
    total_center = 0
    total_right = 0
    trigger_label = "-"

    if result is None or result.masks is None or result.boxes is None:
        return "NONE", trigger_label, 0, 0, 0

    polygons = result.masks.xy
    boxes = result.boxes
    names = result.names

    n = min(
        len(polygons),
        len(boxes)
    )

    for i in range(n):
        poly = np.asarray(
            polygons[i],
            dtype=np.float32
        )

        if poly.ndim != 2 or poly.shape[0] < 3:
            continue

        poly_i = np.round(poly).astype(np.int32)

        poly_i[:, 0] = np.clip(
            poly_i[:, 0],
            0,
            FRAME_W - 1
        )

        poly_i[:, 1] = np.clip(
            poly_i[:, 1],
            0,
            FRAME_H - 1
        )

        mask_u8 = np.zeros(
            (FRAME_H, FRAME_W),
            dtype=np.uint8
        )

        cv2.fillPoly(
            mask_u8,
            [poly_i],
            255
        )

        mask = mask_u8 > 0

        ov_l = int(
            np.count_nonzero(
                mask & zone_masks["left"]
            )
        )

        ov_c = int(
            np.count_nonzero(
                mask & zone_masks["center"]
            )
        )

        ov_r = int(
            np.count_nonzero(
                mask & zone_masks["right"]
            )
        )

        cls_id = int(
            boxes.cls[i].item()
        )

        conf = float(
            boxes.conf[i].item()
        )

        if isinstance(names, dict):
            label = names.get(
                cls_id,
                str(cls_id)
            )
        else:
            label = str(cls_id)

        touching_red = (
            ov_l >= SIDE_MIN_OVERLAP or
            ov_c >= CENTER_MIN_OVERLAP or
            ov_r >= SIDE_MIN_OVERLAP
        )

        # Kontur kuning = terdeteksi tapi belum kena zona merah.
        # Kontur merah = benar-benar menyentuh zona merah.
        color = (
            (0, 0, 255)
            if touching_red
            else (0, 255, 255)
        )

        thickness = (
            3
            if touching_red
            else 2
        )

        cv2.polylines(
            frame_draw,
            [poly_i],
            True,
            color,
            thickness
        )

        x, y, w, h = cv2.boundingRect(
            poly_i
        )

        cv2.putText(
            frame_draw,
            "{} {:.2f}".format(
                label,
                conf
            ),
            (
                x,
                max(18, y - 5)
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            color,
            1
        )

        # HANYA object yang menyentuh zona merah
        # boleh masuk perhitungan action.
        if touching_red:
            total_left += ov_l
            total_center += ov_c
            total_right += ov_r
            trigger_label = label

    # Tidak ada object yang benar-benar menyentuh merah.
    if (
        total_left == 0 and
        total_center == 0 and
        total_right == 0
    ):
        return (
            "NONE",
            "-",
            0,
            0,
            0
        )

    side_max = max(
        total_left,
        total_right
    )

    # Tengah benar-benar dominan -> putar balik.
    if (
        total_center >= CENTER_MIN_OVERLAP and
        total_center >= side_max * CENTER_DOMINANCE
    ):
        return (
            "TURN_BACK",
            trigger_label,
            total_left,
            total_center,
            total_right
        )

    # Dominan kanan -> hindar kiri.
    if (
        total_right >= SIDE_MIN_OVERLAP and
        total_right > total_left
    ):
        return (
            "AVOID_LEFT",
            trigger_label,
            total_left,
            total_center,
            total_right
        )

    # Dominan kiri -> hindar kanan.
    if (
        total_left >= SIDE_MIN_OVERLAP and
        total_left > total_right
    ):
        return (
            "AVOID_RIGHT",
            trigger_label,
            total_left,
            total_center,
            total_right
        )

    # Kiri-kanan seimbang dan center cukup kuat.
    if total_center >= CENTER_MIN_OVERLAP:
        return (
            "TURN_BACK",
            trigger_label,
            total_left,
            total_center,
            total_right
        )

    return (
        "NONE",
        trigger_label,
        total_left,
        total_center,
        total_right
    )


ROI_START_RATIO = 0.55
EDGE_THRESHOLD = 80

# OpenCV HoughLines theta = 0..180.
# Program referensi: kiri +63, kanan -63.
LEFT_THETA_DEG = 63.0
RIGHT_THETA_DEG = 117.0   # ekuivalen -63 derajat
THETA_TOLERANCE = 8.0
HOUGH_THRESHOLD = 40

# ============================================================
# MOTOR COMMAND BERBASIS TITIK NETRAL 127
#
# SESUAI HASIL UJI ROBOT:
#   97  = MAJU
#   127 = STOP
#   157 = MUNDUR
# ============================================================
MOTOR_NEUTRAL = 127
MOTOR_STEP = 30

MOTOR_FORWARD = MOTOR_NEUTRAL - MOTOR_STEP   # 97
MOTOR_REVERSE = MOTOR_NEUTRAL + MOTOR_STEP   # 157

# Forward lebih pelan = mendekati 127.
MOTOR_FORWARD_SLOW = MOTOR_NEUTRAL - 20      # 107
MOTOR_FORWARD_VERY_SLOW = MOTOR_NEUTRAL - 10 # 117

# Putar balik di tempat:
# kiri mundur 157, kanan maju 97.
SPIN_LEFT_CMD = MOTOR_REVERSE
SPIN_RIGHT_CMD = MOTOR_FORWARD

# Berhenti dulu sebelum melakukan putar balik.
OBSTACLE_STOP_SECONDS = 0.40

# Durasi awal putar balik sekitar 180 derajat.
# Harus dikalibrasi pada robot nyata.
TURN_BACK_SECONDS = 1.40

TURN_COOLDOWN_SECONDS = 2.0

# ============================================================
# LAPTOP -> HP TCP BRIDGE -> STM32
#
# DISAMAKAN DENGAN MANUAL CONTROLLER YANG SUDAH TERBUKTI JALAN.
#
# Jalur:
#   Laptop autonomous processing
#      -> TCP 192.168.0.149:8888
#      -> HP bridge
#      -> USB OTG
#      -> STM32
#
# Packet:
#   [0] 0xAA
#   [1] command kanan
#   [2] command kiri
#   [3] checksum = (0xAA + kanan + kiri) & 0xFF
#   [4] 0x55
#
# Motor:
#   127 = STOP
#   97 = MAJU (+30)
#    97 = MUNDUR (-30)
# ============================================================

CONTROL_HP_IP = "192.168.0.147"
CONTROL_TCP_PORT = 8888

SEND_INTERVAL = 0.05
USE_ROS_MOTOR_TOPIC = True

sock = None
sock_lock = threading.Lock()
ros_motor_pub = None

_last_send = 0.0
_last_cmd = None
_drive_cmd_lock = threading.Lock()
_drive_cmd = None


# ============================================================
# ROS PATH FOLLOWER
# ROS mapping tetap jalan sendiri. Program ini hanya membaca
# /odom dan /odom_path untuk steering AUTO.
# ============================================================

ROS_PATH_LOOKAHEAD_M = 0.45
ROS_PATH_MAX_STEER = 55.0
ROS_PATH_GAIN = 1.25
ROS_PATH_TIMEOUT_SEC = 1.0
ROS_PATH_MIN_POINTS = 2
ROS_LIDAR_TOPIC = "/scan_front"
ROS_SCAN_FLIP_X = True
ROS_SCAN_FLIP_Y = False
ROS_PATH_DRAW_SCALE_PX_PER_M = 120.0
ROS_PATH_DRAW_LOOKAHEAD_POINTS = 120

ros_nav_lock = threading.Lock()
ros_nav_ready = False
ros_nav_odom = None
ros_nav_path = []
ros_nav_last_odom_time = 0.0
ros_nav_last_path_time = 0.0
ros_nav_status = "ROS NAV OFF"


def make_packet(kiri, kanan):
    kiri = max(0, min(255, int(kiri)))
    kanan = max(0, min(255, int(kanan)))

    checksum = (
        0xAA +
        kanan +
        kiri
    ) & 0xFF

    return bytes([
        0xAA,
        kanan,
        kiri,
        checksum,
        0x55
    ])


def quat_to_yaw(q):
    return math.atan2(
        2.0 * (
            q.w * q.z +
            q.x * q.y
        ),
        1.0 - 2.0 * (
            q.y * q.y +
            q.z * q.z
        )
    )


def normalize_angle(angle):
    return math.atan2(
        math.sin(angle),
        math.cos(angle)
    )


class RosPathBridge(Node if Node is not None else object):

    def __init__(self):
        super().__init__("magang_lidar_ros_bridge")
        global ros_motor_pub

        ros_motor_pub = self.create_publisher(
            Int16MultiArray,
            "/motor_rpm",
            10
        )

        self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            20
        )

        self.create_subscription(
            Path,
            "/odom_path",
            self.path_callback,
            10
        )

        self.create_subscription(
            LaserScan,
            ROS_LIDAR_TOPIC,
            self.scan_callback,
            20
        )

        self.get_logger().info(
            "ROS BRIDGE READY: /odom + /odom_path + /scan_front + /motor_rpm"
        )

    def odom_callback(self, msg):
        global ros_nav_odom
        global ros_nav_last_odom_time

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        with ros_nav_lock:
            ros_nav_odom = (
                float(p.x),
                float(p.y),
                quat_to_yaw(q)
            )
            ros_nav_last_odom_time = time.monotonic()

    def path_callback(self, msg):
        global ros_nav_path
        global ros_nav_last_path_time

        points = []

        for pose in msg.poses:
            p = pose.pose.position
            points.append(
                (
                    float(p.x),
                    float(p.y)
                )
            )

        with ros_nav_lock:
            ros_nav_path = points
            ros_nav_last_path_time = time.monotonic()

    def scan_callback(self, msg):
        global lidar_latest_scan
        global lidar_last_scan_time
        global lidar_scan_hz
        global lidar_scan_counter
        global lidar_connected

        points = []
        angle = msg.angle_min

        for r in msg.ranges:
            if (
                math.isfinite(r) and
                msg.range_min <= r <= msg.range_max
            ):
                dist = r * 1000.0

                x = dist * math.cos(angle)
                y = -(
                    dist *
                    math.sin(angle)
                )

                if ROS_SCAN_FLIP_X:
                    x = -x

                if ROS_SCAN_FLIP_Y:
                    y = -y

                points.append(
                    (
                        x,
                        y
                    )
                )

            angle += msg.angle_increment

        now = time.monotonic()

        with lidar_scan_lock:
            lidar_latest_scan = points

        lidar_connected = True
        lidar_last_scan_time = now
        lidar_scan_counter += 1

        if msg.scan_time > 0:
            lidar_scan_hz = 1.0 / msg.scan_time


def ros_bridge_loop():
    global ros_nav_ready
    global ros_nav_status

    if rclpy is None:
        ros_nav_status = "ROS NAV: rclpy tidak tersedia"
        print("[ROS NAV] rclpy tidak tersedia", flush=True)
        return

    try:
        rclpy.init(args=None)
        node = RosPathBridge()

        with ros_nav_lock:
            ros_nav_ready = True
            ros_nav_status = "ROS NAV READY"

        rclpy.spin(node)

    except Exception as e:
        with ros_nav_lock:
            ros_nav_ready = False
            ros_nav_status = "ROS NAV ERROR {}".format(e)

        print(
            "[ROS NAV] ERROR: {}".format(e),
            flush=True
        )

    finally:
        try:
            if rclpy is not None and rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


def ros_path_follow_info():
    now = time.monotonic()

    with ros_nav_lock:
        odom = ros_nav_odom
        path = list(ros_nav_path)
        odom_age = now - ros_nav_last_odom_time
        path_age = now - ros_nav_last_path_time
        status = ros_nav_status

    if odom is None:
        return {
            "valid": False,
            "steering": 0.0,
            "reason": "ROS ODOM BELUM ADA",
            "target_x": FRAME_W / 2,
            "target_y": FRAME_H / 2,
            "confidence": 0.0,
        }

    if (
        len(path) < ROS_PATH_MIN_POINTS or
        odom_age > ROS_PATH_TIMEOUT_SEC or
        path_age > ROS_PATH_TIMEOUT_SEC
    ):
        return {
            "valid": False,
            "steering": 0.0,
            "reason": status,
            "target_x": FRAME_W / 2,
            "target_y": FRAME_H / 2,
            "confidence": 0.0,
        }

    x, y, yaw = odom

    nearest_i = min(
        range(len(path)),
        key=lambda i: math.hypot(
            path[i][0] - x,
            path[i][1] - y
        )
    )

    target_i = nearest_i
    traveled = 0.0

    for i in range(nearest_i, len(path) - 1):
        x1, y1 = path[i]
        x2, y2 = path[i + 1]
        traveled += math.hypot(
            x2 - x1,
            y2 - y1
        )
        target_i = i + 1

        if traveled >= ROS_PATH_LOOKAHEAD_M:
            break

    tx, ty = path[target_i]

    dx = tx - x
    dy = ty - y

    c = math.cos(-yaw)
    s = math.sin(-yaw)

    local_x = c * dx - s * dy
    local_y = s * dx + c * dy

    if local_x < 0.05 and target_i < len(path) - 1:
        tx, ty = path[min(target_i + 1, len(path) - 1)]
        dx = tx - x
        dy = ty - y
        local_x = c * dx - s * dy
        local_y = s * dx + c * dy

    target_angle = math.atan2(
        local_y,
        max(0.05, local_x)
    )

    # local_y positif artinya target di kiri. steering_to_rpm memakai
    # steering negatif untuk belok kiri.
    steering = -math.degrees(target_angle) * ROS_PATH_GAIN
    steering = float(
        np.clip(
            steering,
            -ROS_PATH_MAX_STEER,
            ROS_PATH_MAX_STEER
        )
    )

    dist_to_path = math.hypot(
        path[nearest_i][0] - x,
        path[nearest_i][1] - y
    )

    confidence = float(
        np.clip(
            1.0 - dist_to_path,
            0.0,
            1.0
        )
    )

    target_x = FRAME_W / 2 + steering * 3.0
    target_x = float(
        np.clip(
            target_x,
            0,
            FRAME_W - 1
        )
    )

    return {
        "valid": True,
        "steering": steering,
        "reason": "ROS PATH i={}/{} d={:.2f}m".format(
            target_i,
            len(path),
            dist_to_path
        ),
        "target_x": target_x,
        "target_y": FRAME_H * 0.55,
        "confidence": confidence,
    }


def control_connect():
    """
    Sama dengan manual controller:
      socket TCP
      TCP_NODELAY
      connect HP:8888
    """
    global sock

    with sock_lock:
        if sock is not None:
            return True

        try:
            s = socket.socket(
                socket.AF_INET,
                socket.SOCK_STREAM
            )

            s.setsockopt(
                socket.IPPROTO_TCP,
                socket.TCP_NODELAY,
                1
            )

            s.settimeout(1.0)

            s.connect((
                CONTROL_HP_IP,
                CONTROL_TCP_PORT
            ))

            # Setelah connect gunakan timeout pendek,
            # sama seperti manual controller.
            s.settimeout(0.3)

            sock = s

            print(
                "[CTRL] Connected {}:{}"
                .format(
                    CONTROL_HP_IP,
                    CONTROL_TCP_PORT
                ),
                flush=True
            )

            return True

        except Exception as e:
            sock = None

            print(
                "[CTRL] Connect failed: {}"
                .format(e),
                flush=True
            )

            return False


def control_disconnect():
    global sock

    with sock_lock:
        if sock is not None:
            try:
                sock.close()
            except Exception:
                pass

        sock = None


def control_send_rpm(rpm_kiri, rpm_kanan, force=False):
    """
    Kirim command autonomous memakai format yang sama
    dengan manual controller user.
    """
    global _last_send
    global _last_cmd
    global sock
    global _drive_cmd

    kiri = max(
        0,
        min(255, int(round(rpm_kiri)))
    )

    kanan = max(
        0,
        min(255, int(round(rpm_kanan)))
    )

    now = time.monotonic()
    cmd = (kiri, kanan)

    with _drive_cmd_lock:
        _drive_cmd = cmd

    if ros_motor_pub is not None and Int16MultiArray is not None:
        msg = Int16MultiArray()
        msg.data = [kiri, kanan]
        ros_motor_pub.publish(msg)
        _last_send = now
        _last_cmd = cmd
        return True

    if USE_ROS_MOTOR_TOPIC:
        return False

    if not force:
        if (
            cmd == _last_cmd and
            (now - _last_send) < SEND_INTERVAL
        ):
            return False

        if (
            now - _last_send
        ) < SEND_INTERVAL:
            return False

    packet = make_packet(
        kiri,
        kanan
    )

    if not control_connect():
        return False

    with sock_lock:
        s = sock

        if s is None:
            return False

        try:
            s.sendall(packet)

            _last_send = now
            _last_cmd = cmd

            return True

        except Exception as e:
            print(
                "[CTRL] Send failed: {}"
                .format(e),
                flush=True
            )

            try:
                s.close()
            except Exception:
                pass

            sock = None

            return False


def control_reconnect_loop():
    """
    Reconnect otomatis ringan.
    """
    while running:
        if sock is None:
            control_connect()

        time.sleep(1.0)




# ============================================================
# RPLIDAR A1 DIRECT USB -> LAPTOP
# ============================================================

LIDAR_BAUD = 115200
LIDAR_PORT = None

LIDAR_MIN_DISTANCE_MM = 150.0
LIDAR_MAX_DISTANCE_MM = 4000.0
LIDAR_TIMEOUT_SEC = 0.70

ROBOT_RADIUS_MM = 300.0

# ============================================================
# LIDAR BODY SELF-FILTER
# Titik di dalam body robot adalah komponen robot sendiri.
# ============================================================
LIDAR_SELF_IGNORE_MARGIN_MM = 45.0
LIDAR_SELF_IGNORE_RADIUS_MM = (
    ROBOT_RADIUS_MM +
    LIDAR_SELF_IGNORE_MARGIN_MM
)

# Self-filter hanya untuk area samping robot.
# Area depan dan depan-serong tetap dianggap obstacle
# walaupun jaraknya dekat ke body.
LIDAR_SELF_IGNORE_SIDE_START_DEG = 55.0
LIDAR_SELF_IGNORE_SIDE_END_DEG = 125.0

# Clearance dihitung dari permukaan body, bukan pusat LiDAR.
LIDAR_EMERGENCY_CLEARANCE_MM = 120.0
LIDAR_HARD_AVOID_CLEARANCE_MM = 450.0
LIDAR_SOFT_AVOID_CLEARANCE_MM = 650.0

# ============================================================
# FREE-SPACE / GAP NAVIGATOR
# LiDAR menentukan robot harus lewat MANA, bukan sekadar wall-follow.
# ============================================================
LIDAR_GAP_BIN_DEG = 5
LIDAR_GAP_MIN_ANGLE_DEG = 20
LIDAR_PLAN_MAX_RANGE_MM = 3000.0

# Sebuah arah dianggap layak dilewati jika obstacle terdekat
# masih lebih jauh dari nilai ini.
LIDAR_GAP_FREE_MM = 1050.0

# Jika depan cukup kosong, robot tetap memilih lurus.
LIDAR_FRONT_CLEAR_MM = 1350.0

# Steering smoothing + "commit turn" agar tidak langsung balik ke tengah.
LIDAR_STEER_ALPHA = 0.72
LIDAR_TARGET_ALPHA = 0.68
LIDAR_TURN_COMMIT_SEC = 0.85
LIDAR_TURN_COMMIT_MIN_STEER = 42.0

# CAMERA PRIMARY + LIDAR SAFETY ONLY
LIDAR_SAFETY_FRONT_STOP_MM = 520.0
LIDAR_SAFETY_FRONT_WARN_MM = 800.0
LIDAR_SAFETY_SIDE_WARN_MM = 520.0
LIDAR_SAFETY_SOFT_STEER = 18.0
LIDAR_SAFETY_HARD_STEER = 32.0
LIDAR_SAFETY_HOLD_SEC = 0.45

# ============================================================
# SMART BODY CLEARANCE
# Yellow circle = physical robot body.
# Start correcting BEFORE obstacle reaches body.
# ============================================================
LIDAR_BODY_EARLY_WARN_MM = 900.0   # mulai koreksi halus dari luar body
LIDAR_BODY_MEDIUM_MM = 650.0       # koreksi sedang
LIDAR_BODY_HARD_MM = 450.0         # koreksi kuat
LIDAR_BODY_STOP_MM = 160.0         # stop jika benar-benar terlalu dekat

# Corner intelligence:
# Jika depan dekat tetapi kiri/kanan masih terbuka, itu dianggap belokan,
# bukan jalan buntu.
LIDAR_CORNER_OPEN_MM = 950.0
LIDAR_CORNER_FRONT_MM = 850.0
LIDAR_CORNER_STEER = 42.0

# Smoothing agar koreksi tidak patah-patah.
LIDAR_REPULSION_ALPHA = 0.78

# Side sensing intentionally short for narrow corridors.
LIDAR_SIDE_USE_MAX_MM = 620.0
LIDAR_SIDE_OPEN_MM = 560.0
LIDAR_SIDE_BLOCK_MM = 410.0

# Escape memory for true trap.
LIDAR_ESCAPE_MEMORY_SEC = 4.0
LIDAR_TRAPPED_CONFIRM_SEC = 0.28
LIDAR_REVERSE_ESCAPE_SEC = 0.90
LIDAR_REVERSE_TURN_STEER = 55.0

# ============================================================
# YELLOW BODY CLEARANCE POLICY
# Semua jarak dihitung dari BATAS body kuning.
# ============================================================
LIDAR_YELLOW_WARN_CLEAR_MM = 500.0
LIDAR_YELLOW_MED_CLEAR_MM = 320.0
LIDAR_YELLOW_HARD_CLEAR_MM = 180.0
LIDAR_YELLOW_STOP_CLEAR_MM = 70.0

# Minimal koreksi, bahkan untuk satu titik kecil.
LIDAR_MIN_AVOID_STEER = 16.0
LIDAR_MED_AVOID_STEER = 30.0
LIDAR_HARD_AVOID_STEER = 48.0

# ============================================================
# OCCUPANCY / DENSITY DIRECTION CHOICE
# Nilai bahaya berdasarkan:
#   - jumlah titik
#   - kedekatan ke body kuning
#   - posisi kiri/tengah/kanan
# ============================================================
LIDAR_OCCUPANCY_MAX_RANGE_MM = 1700.0
LIDAR_OCCUPANCY_FRONT_X_MM = 1600.0

# Batas sektor lateral.
LIDAR_OCC_LEFT_MAX_Y_MM = 900.0
LIDAR_OCC_RIGHT_MAX_Y_MM = 900.0

# Bobot.
LIDAR_OCC_NEAR_POWER = 2.2
LIDAR_OCC_CENTER_WEIGHT = 1.35
LIDAR_OCC_SIDE_WEIGHT = 1.0

# Minimal perbedaan score agar pindah arah.
LIDAR_OCC_SCORE_MARGIN = 0.12

# ============================================================
# CAMERA -> RECOMMENDATION, LIDAR -> SAFETY GATE
# ============================================================
FUSION_CAMERA_CENTER_DEADBAND = 12.0

# LiDAR evaluates only around the body safety envelope.
LIDAR_BODY_SAFETY_RADIUS_MM = ROBOT_RADIUS_MM + 350.0  # 650 mm
LIDAR_BODY_HARD_RADIUS_MM = ROBOT_RADIUS_MM + 70.0

# ============================================================
# COOPERATIVE SAFETY LIMITS
# ============================================================
# LiDAR safe boundary. Jangan sampai obstacle masuk lingkaran merah.
LIDAR_RED_SAFETY_RADIUS_MM = 650.0
LIDAR_EMERGENCY_RADIUS_MM = LIDAR_RED_SAFETY_RADIUS_MM

# Equal weighting between camera and LiDAR safety
FUSION_CAMERA_SAFETY_WEIGHT = 1.0
FUSION_LIDAR_SAFETY_WEIGHT = 1.0

FUSION_MAX_SAFETY_STEER = 68.0
FUSION_CONFLICT_MARGIN = 0.15

# ============================================================
# THIRD SAFETY: CANNY SIDE-LINE GUARD
#
# Detect ANY strong edge/line near the projected LEFT/RIGHT
# sides of the camera body footprint.
# Bottom floor is ignored to reduce reflections/tiles/shadows.
# ============================================================

CANNY_SIDE_Y_TOP = int(FRAME_H * 0.30)
CANNY_SIDE_Y_BOTTOM = int(FRAME_H * 0.72)

CONTOUR_CANNY_T1 = 55
CONTOUR_CANNY_T2 = 135

# Width of left/right safety corridors around projected body side.
CONTOUR_SIDE_BAND_PX = 58

# How close a detected edge may approach the projected body line.
CONTOUR_SIDE_DANGER_PX = 30

# Contour safety is valid only OUTSIDE the body side.
# Tiny negative values are tolerated for segmentation/edge thickness,
# but large negative values mean the contour is actually in the CENTER
# and must NOT be treated as side collision.
CONTOUR_BODY_PENETRATION_TOL_PX = 0.0
CONTOUR_OUTSIDE_MARGIN_PX = 0

# Hough parameters to reject tiny noisy edges.
CANNY_HOUGH_THRESHOLD = 20
CANNY_MIN_LINE_LENGTH = 34
CANNY_MAX_LINE_GAP = 14

CONTOUR_SIDE_AVOID_STEER = 48.0
CONTOUR_SIDE_HOLD_FRAMES = 3

# ============================================================
# CONTOUR = LANE ASSIST + SIDE CORRECTION
#
# Contour is NOT a stop trigger.
# It helps estimate the lane center from left/right boundaries.
# ============================================================
CONTOUR_LANE_BLEND = 0.55
HSV_LANE_BLEND = 0.45

CONTOUR_LANE_MIN_POINTS = 12
CONTOUR_LANE_FAR_Y_RATIO = 0.38
CONTOUR_LANE_NEAR_Y_RATIO = 0.68

CONTOUR_LANE_MAX_STEER = 24.0
CONTOUR_LANE_DEADBAND_PX = 22.0

# ============================================================
# FAR CORRIDOR CENTER + SOFT BODY TOUCH CORRECTION
# ============================================================

# Use the farthest usable left/right contour boundaries
# and aim at their midpoint.
FAR_CONTOUR_Y_MIN_RATIO = 0.30
FAR_CONTOUR_Y_MAX_RATIO = 0.58
FAR_CONTOUR_MIN_POINTS = 8

# ============================================================
# FAR LANE VISION
#
# Navigation uses the farthest visible left/right boundaries
# ABOVE the robot body.
# Near-body contour remains only for collision safety.
# ============================================================
FAR_LANE_SCAN_Y_TOP = int(FRAME_H * 0.18)
FAR_LANE_SCAN_Y_BOTTOM = int(FRAME_H * 0.62)

FAR_LANE_CENTER_GAP_PX = 38
FAR_LANE_MIN_CONTOUR_POINTS = 10
FAR_LANE_MIN_HEIGHT_PX = 22

FAR_LANE_CANNY_T1 = 45
FAR_LANE_CANNY_T2 = 120

FAR_LANE_CLOSE_KERNEL = 5
FAR_LANE_TARGET_SMOOTH = 0.78
FAR_LANE_STEER_MAX = 32.0
FAR_LANE_STEER_GAIN = 1.15

# ============================================================
# EDGE ROAD BOUNDARIES
#
# Detected left/right edges are treated as ROAD BOUNDARIES.
# The stable midpoint between them is used internally for steering.
# ============================================================
EDGE_ROAD_MIN_WIDTH_PX = 90
EDGE_ROAD_SAMPLE_COUNT = 9
EDGE_ROAD_TARGET_LOOKAHEAD_RATIO = 0.24
EDGE_ROAD_CENTER_SMOOTH = 0.80

_edge_road_center_state = FRAME_W / 2.0
_edge_left_state = None
_edge_right_state = None
_edge_width_state = FRAME_W * 0.58

EDGE_ONLY_ROI_TOP_RATIO = 0.34
EDGE_ONLY_ROI_BOTTOM_RATIO = 0.92
EDGE_ONLY_CANNY_LOW = 60
EDGE_ONLY_CANNY_HIGH = 150
EDGE_ONLY_HOUGH_THRESHOLD = 35
EDGE_ONLY_MIN_LINE_LENGTH = 42
EDGE_ONLY_MAX_LINE_GAP = 22
EDGE_ONLY_SMOOTH = 0.78
EDGE_ONLY_WIDTH_SMOOTH = 0.88
EDGE_ONLY_MIN_WIDTH_PX = 150
EDGE_ONLY_MAX_STEER = 32.0
EDGE_ONLY_STEER_GAIN = 42.0
EDGE_ONLY_CENTER_DEADBAND_PX = 18.0
EDGE_ONLY_MIN_DX_DOWN_PX = 28.0
EDGE_ONLY_MIN_BOTTOM_Y_RATIO = 0.62
EDGE_ONLY_SIDE_MARGIN_PX = 24.0
EDGE_ONLY_MIN_ABS_SLOPE = 0.22
EDGE_ONLY_MAX_ABS_SLOPE = 2.20
EDGE_ONLY_ONE_SIDE_MIN_STEER = 16.0
CAMERA_ONLY_STEER_ALPHA = 0.82
CAMERA_ONLY_EDGE_LOST_HOLD_SEC = 0.45
_camera_only_steer_state = 0.0
_camera_only_last_valid_ts = 0.0

# ============================================================
# UNIFIED SENSOR FUSION
# ============================================================
UNIFIED_FRONT_WARN_MM = 1000.0
UNIFIED_FRONT_CAUTION_MM = 750.0
UNIFIED_FRONT_HARD_MM = 520.0

UNIFIED_SIDE_LIDAR_LIMIT_MM = LIDAR_RED_SAFETY_RADIUS_MM

# ============================================================
# PROACTIVE LIDAR AVOIDANCE
# Red circle = hard boundary.
# Start steering BEFORE any point reaches it.
# ============================================================
LIDAR_PREWARN_BUFFER_MM = 180.0
LIDAR_SIDE_PREWARN_MM = LIDAR_RED_SAFETY_RADIUS_MM + LIDAR_PREWARN_BUFFER_MM
LIDAR_ARROW_GUIDE_MM = LIDAR_RED_SAFETY_RADIUS_MM + 450.0
LIDAR_FRONT_PREWARN_MM = LIDAR_ARROW_GUIDE_MM
LIDAR_SIDE_HARD_MM = LIDAR_RED_SAFETY_RADIUS_MM

LIDAR_PREWARN_STEER_MIN = 24.0
LIDAR_PREWARN_STEER_MAX = 74.0

UNIFIED_SIDE_CAMERA_WARN_PX = 46.0
UNIFIED_SIDE_CAMERA_HARD_PX = 12.0

UNIFIED_NAV_MAX_STEER = 28.0
UNIFIED_SAFETY_MAX_STEER = 55.0

UNIFIED_SINGLE_SENSOR_GAIN = 0.55
UNIFIED_DOUBLE_SENSOR_GAIN = 0.82
UNIFIED_TRIPLE_SENSOR_GAIN = 1.00

UNIFIED_STEER_ALPHA = 0.76
_unified_steer_state = 0.0

UNIFIED_FRONT_LOOKAHEAD_BIAS = 0.25

_far_lane_target_state = FRAME_W / 2.0

# Soft correction only when contour gets into/touches body safety.
# Goal: stay close, but never overlap the body.
SOFT_BODY_TOUCH_MARGIN_PX = 4.0
SOFT_BODY_WARN_MARGIN_PX = 18.0

SOFT_BODY_CORRECTION_MIN = 4.0
SOFT_BODY_CORRECTION_MAX = 18.0
SOFT_BODY_CORRECTION_GAIN = 0.65

# ============================================================
# COOPERATIVE SENSOR CONFIRMATION + PREDICTIVE BODY CORRECTION
# ============================================================

# Start correcting earlier than actual body touch.
PREDICTIVE_BODY_WARN_MARGIN_PX = 42.0
PREDICTIVE_BODY_CAUTION_MARGIN_PX = 68.0

# Camera contour alone is NOT enough for a hard avoidance decision.
# Confirmation can come from LiDAR or YOLO on the same side.
SENSOR_CONFIRMATION_REQUIRED = True

# If contour is extremely close to body, allow emergency correction
# even before cross-confirmation.
CONTOUR_EMERGENCY_MARGIN_PX = 8.0

# Predictive steering range.
PREDICTIVE_STEER_MIN = 4.0
PREDICTIVE_STEER_MAX = 24.0
PREDICTIVE_STEER_GAIN = 0.50

# LiDAR side confirmation threshold uses the safety ring.
LIDAR_CONFIRM_RADIUS_MM = LIDAR_RED_SAFETY_RADIUS_MM

# YOLO overlap threshold for side confirmation.
YOLO_CONFIRM_OVERLAP_PX = 35

_predictive_steer_state = 0.0
PREDICTIVE_STEER_ALPHA = 0.80

# Temporal smoothing so correction is small and gradual.
SOFT_BODY_STEER_ALPHA = 0.78
_soft_body_steer_state = 0.0

# Contour filtering: reject tiny random edges, keep object-like shapes.
CONTOUR_MIN_AREA = 180
CONTOUR_MIN_PERIMETER = 70.0
CONTOUR_MIN_HEIGHT = 28
CONTOUR_MIN_WIDTH = 12

# Morphology connects fragmented chair/person/object edges.
CONTOUR_CLOSE_KERNEL = 7
CONTOUR_DILATE_KERNEL = 3

_contour_side_hold = 0
_contour_side_last_action = "NONE"


# ============================================================
# THIRD SAFETY: RED LINE GUARD
# Check MIDDLE camera only; lower floor deliberately ignored.
# ============================================================



# STOP must remain persistent before 360 spin begins.
UTURN_BLOCKED_CONFIRM_SEC = 0.65

# ============================================================
# STABLE FORWARD NAVIGATION
#
# Once a valid road is found, robot prefers going straight.
# Small camera/path movements are ignored to prevent zig-zag.
# Safety sensors are the ones that command strong avoidance.
# ============================================================
PATH_STRAIGHT_DEADBAND = 18.0
PATH_GENTLE_STEER_MAX = 18.0
PATH_STEER_GAIN = 0.45

# Strong safety correction only.
SAFETY_STEER_MIN = 34.0

# Auto-360 requires a TRUE dead-end, not ordinary STOP.
UTURN_TRUE_TRAP_CONFIRM_SEC = 1.00
_true_trap_since = 0.0
_uturn_blocked_since = 0.0

# ============================================================
# BODY-MATCHED LIDAR SAFETY GEOMETRY
# Approx. 15 cm tighter than previous envelope.
# ============================================================
LIDAR_FRONT_REACH_MM = LIDAR_FRONT_PREWARN_MM
LIDAR_SIDE_REACH_MM = LIDAR_SIDE_PREWARN_MM
LIDAR_SIDE_HALF_WIDTH_MM = ROBOT_RADIUS_MM + 210.0

# Camera remains backup visual safety.
CAMERA_BACKUP_ENABLED = True

# Risk limits for accepting camera recommendation.
LIDAR_ACCEPT_RISK = 2.8
LIDAR_BLOCK_RISK = 5.5

# ============================================================
# AUTO 360 U-TURN WHEN NO SAFE PATH
# ============================================================
UTURN_STOP_BEFORE_SEC = 0.25
UTURN_SPIN_DURATION_SEC = 3.35
UTURN_SETTLE_SEC = 0.20

# Direction alternates so the robot does not always spin same way.
_uturn_state = "IDLE"      # IDLE / STOPPING / SPINNING / SETTLING
_uturn_started = 0.0
_uturn_direction = "LEFT"
_uturn_last_direction = "RIGHT"

# Final steering magnitudes when LiDAR must override camera.
FUSION_OVERRIDE_SOFT = 26.0
FUSION_OVERRIDE_HARD = 44.0

# ============================================================
# LIDAR -> CAMERA VIRTUAL PATH CALIBRATION
# ============================================================
LIDAR_CAM_BASE_X = FRAME_W // 2
LIDAR_CAM_BASE_Y = int(FRAME_H * 0.90)
LIDAR_CAM_HORIZON_Y = int(FRAME_H * 0.43)

LIDAR_CAM_MIN_X_MM = 300.0
LIDAR_CAM_MAX_X_MM = 2600.0
LIDAR_CAM_PIXELS_PER_M_NEAR = 220.0
LIDAR_CAM_PIXELS_PER_M_FAR = 75.0

LIDAR_PATH_STEP_MM = 180.0
LIDAR_PATH_LOOKAHEAD_MM = 1150.0
LIDAR_PATH_WIDTH_MM = ROBOT_RADIUS_MM * 2.0 + 180.0
LIDAR_PATH_MAX_LATERAL_MM = 900.0

LIDAR_PATH_ALPHA = 0.72
LIDAR_PATH_STEER_ALPHA = 0.70

_lidar_virtual_path = []
_lidar_virtual_steer = 0.0
_lidar_virtual_target_y = 0.0

# Smoothing keputusan supaya tidak flip kiri-kanan.
LIDAR_DIR_HOLD_SEC = 0.55
_lidar_dir_hold_until = 0.0
_lidar_dir_hold_sign = 0

_lidar_last_open_dir = "LEFT"
_lidar_last_open_time = 0.0
_lidar_trapped_since = 0.0
_lidar_prev_repulsion = 0.0
_lidar_safety_hold_until = 0.0
_lidar_safety_hold_steer = 0.0
LIDAR_EMERGENCY_MM = (
    ROBOT_RADIUS_MM +
    LIDAR_EMERGENCY_CLEARANCE_MM
)
LIDAR_FRONT_WARN_MM = 900.0
LIDAR_DIAGONAL_WARN_MM = 650.0
LIDAR_SIDE_WARN_MM = 480.0
LIDAR_DEADEND_SIDE_MM = 700.0
LIDAR_DEADEND_FRONT_MM = 850.0

LIDAR_FRONT_HALF_DEG = 22.0
LIDAR_DIAG_MAX_DEG = 60.0
LIDAR_SIDE_MAX_DEG = 100.0
LIDAR_FRONT_OFFSET_DEG = 0.0

LIDAR_AVOID_STEER = 78.0
LIDAR_SOFT_BIAS = 22.0

lidar_ser = None
lidar_lock = threading.Lock()
lidar_scan_lock = threading.Lock()

lidar_latest_scan = []
lidar_last_scan_time = 0.0
lidar_scan_hz = 0.0
lidar_scan_counter = 0
lidar_connected = False

lidar_front = float("inf")
lidar_front_left = float("inf")
lidar_front_right = float("inf")
lidar_left = float("inf")
lidar_right = float("inf")

lidar_action = "TIMEOUT"
lidar_reason = "LiDAR belum siap"
lidar_steer_hint = 0.0
lidar_turn_dir = "LEFT"
lidar_left_score = 0.0
lidar_center_score = 0.0
lidar_right_score = 0.0
lidar_left_count = 0
lidar_center_count = 0
lidar_right_count = 0
lidar_hard_left = False
lidar_hard_center = False
lidar_hard_right = False
lidar_target_angle = 0.0
_lidar_prev_target_angle = 0.0
_lidar_prev_steer = 0.0
_lidar_commit_until = 0.0
_lidar_commit_sign = 0


def choose_lidar_port():
    """
    Pilih port RPLIDAR langsung dari USB laptop.

    PENTING:
      - Jangan pernah memilih /dev/ttyS*
      - Prioritas /dev/ttyUSB*
      - Metadata RPLIDAR / CP210x / CH340 / Silicon Labs
    """
    ports = list(
        serial.tools.list_ports.comports()
    )

    if LIDAR_PORT:
        return LIDAR_PORT

    if not ports:
        return None

    usb_candidates = []

    for p in ports:
        dev = p.device or ""

        # Serial internal laptop tidak boleh dipakai.
        if "/dev/ttyS" in dev:
            continue

        text = "{} {} {}".format(
            p.description or "",
            p.manufacturer or "",
            p.hwid or ""
        ).lower()

        score = 0

        if "rplidar" in text:
            score += 100

        if (
            "silicon labs" in text or
            "cp210" in text
        ):
            score += 80

        if "ch340" in text:
            score += 70

        if (
            "usb serial" in text or
            "usb-serial" in text
        ):
            score += 60

        if "/dev/ttyUSB" in dev:
            score += 50

        if score > 0:
            usb_candidates.append(
                (score, dev, text)
            )

    if not usb_candidates:
        print(
            "[LIDAR] Tidak menemukan USB LiDAR. "
            "Cek: python3 -m serial.tools.list_ports -v",
            flush=True
        )
        return None

    usb_candidates.sort(
        key=lambda x: x[0],
        reverse=True
    )

    selected = usb_candidates[0][1]

    print(
        "[LIDAR] Auto port: {}".format(
            selected
        ),
        flush=True
    )

    return selected



def lidar_parse_node(node):
    if len(node) != 5:
        return None

    b0 = node[0]
    start_flag = b0 & 0x01
    inverse_flag = (b0 >> 1) & 0x01

    if start_flag == inverse_flag:
        return None

    quality = b0 >> 2
    angle_raw = node[1] | (node[2] << 8)

    if (angle_raw & 0x01) != 1:
        return None

    angle_deg = (angle_raw >> 1) / 64.0
    distance_mm = (node[3] | (node[4] << 8)) / 4.0

    return start_flag, quality, angle_deg, distance_mm


def lidar_normalize_angle(angle):
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


def lidar_stable_distance(values):
    vals = [float(v) for v in values if np.isfinite(v)]
    if not vals:
        return float("inf")
    vals.sort()
    n = min(3, len(vals))
    return sum(vals[:n]) / n


def lidar_open_serial():
    """
    Open RPLIDAR langsung USB.

    Fix untuk kondisi:
      CONNECTED tetapi SCAN 0.0Hz / #0

    Urutan:
      1. Open serial 115200
      2. DTR LOW -> motor adapter A1 aktif
      3. STOP
      4. RESET
      5. bersihkan input
      6. START SCAN
      7. tunggu descriptor A5 5A
    """
    global lidar_ser
    global lidar_connected

    port = choose_lidar_port()

    if port is None:
        lidar_connected = False
        return False

    try:
        s = serial.Serial(
            port=port,
            baudrate=LIDAR_BAUD,
            timeout=0.25,
            write_timeout=0.5
        )

        # RPLIDAR A1 USB adapter biasanya memakai DTR untuk motor.
        try:
            s.dtr = False
        except Exception as e:
            print(
                "[LIDAR] DTR warning:",
                e,
                flush=True
            )

        time.sleep(0.20)

        # STOP scan lama.
        try:
            s.write(bytes([0xA5, 0x25]))
            s.flush()
        except Exception:
            pass

        time.sleep(0.08)

        # RESET device supaya state scan bersih.
        try:
            s.write(bytes([0xA5, 0x40]))
            s.flush()
        except Exception:
            pass

        time.sleep(0.60)

        try:
            s.reset_input_buffer()
            s.reset_output_buffer()
        except Exception:
            pass

        # Pastikan motor aktif sekali lagi.
        try:
            s.dtr = False
        except Exception:
            pass

        time.sleep(0.15)

        # START standard scan.
        s.write(bytes([0xA5, 0x20]))
        s.flush()

        # Cari response descriptor:
        # A5 5A xx xx xx xx xx
        descriptor = bytearray()
        deadline = time.monotonic() + 2.0

        while time.monotonic() < deadline:
            b = s.read(1)

            if not b:
                continue

            descriptor.extend(b)

            # Sinkronkan ke A5 5A.
            while (
                len(descriptor) >= 2 and
                not (
                    descriptor[0] == 0xA5 and
                    descriptor[1] == 0x5A
                )
            ):
                del descriptor[0]

            if len(descriptor) >= 7:
                descriptor = descriptor[:7]
                break

        if (
            len(descriptor) < 7 or
            descriptor[0] != 0xA5 or
            descriptor[1] != 0x5A
        ):
            print(
                "[LIDAR] WARNING: descriptor scan tidak diterima. "
                "Tetap mencoba parser measurement...",
                flush=True
            )
        else:
            print(
                "[LIDAR] Scan descriptor OK: {}".format(
                    " ".join(
                        "{:02X}".format(x)
                        for x in descriptor
                    )
                ),
                flush=True
            )

        lidar_ser = s
        lidar_connected = True

        print(
            "[LIDAR] DIRECT USB CONNECTED {} @ {}"
            .format(
                port,
                LIDAR_BAUD
            ),
            flush=True
        )

        return True

    except Exception as e:
        lidar_ser = None
        lidar_connected = False

        print(
            "[LIDAR] Connect/start gagal {}: {}"
            .format(
                port,
                e
            ),
            flush=True
        )

        try:
            s.close()
        except Exception:
            pass

        return False



def lidar_close():
    global lidar_ser
    global lidar_connected

    with lidar_lock:
        s = lidar_ser
        lidar_ser = None
        lidar_connected = False

    if s is not None:
        try:
            s.write(bytes([0xA5, 0x25]))  # STOP
            s.flush()
            time.sleep(0.03)
        except Exception:
            pass

        # Matikan motor via DTR pada adapter A1.
        try:
            s.dtr = True
        except Exception:
            pass

        try:
            s.close()
        except Exception:
            pass



def lidar_reader_loop():
    global lidar_ser
    global lidar_connected
    global lidar_latest_scan
    global lidar_last_scan_time
    global lidar_scan_hz
    global lidar_scan_counter

    buf = bytearray()
    curr_scan = []

    last_scan_perf = time.perf_counter()
    last_byte_time = time.monotonic()
    last_debug_time = 0.0
    valid_nodes_total = 0

    while running:
        if lidar_ser is None:
            if not lidar_open_serial():
                time.sleep(1.0)
                continue

            buf.clear()
            curr_scan = []
            last_byte_time = time.monotonic()
            valid_nodes_total = 0

        try:
            waiting = lidar_ser.in_waiting

            data = lidar_ser.read(
                max(
                    1,
                    min(
                        4096,
                        waiting if waiting > 0 else 1
                    )
                )
            )

            if data:
                buf.extend(data)
                last_byte_time = time.monotonic()

            else:
                # Connected tetapi benar-benar tidak ada byte.
                if (
                    time.monotonic() -
                    last_byte_time
                ) > 2.0:
                    print(
                        "[LIDAR] CONNECTED tapi tidak ada data scan. "
                        "Restart motor + scan...",
                        flush=True
                    )

                    lidar_close()
                    time.sleep(0.5)
                    continue

                time.sleep(0.002)

        except Exception as e:
            print(
                "[LIDAR] Read error:",
                e,
                flush=True
            )

            lidar_close()
            time.sleep(0.5)
            continue

        # Measurement node RPLIDAR standard = 5 bytes.
        while len(buf) >= 5:
            node = lidar_parse_node(
                bytes(buf[:5])
            )

            if node is None:
                # Shift satu byte sampai alignment valid.
                del buf[0]
                continue

            del buf[:5]

            start_flag, quality, angle_deg, dist = node
            valid_nodes_total += 1

            # Start flag menandakan scan revolution baru.
            if start_flag == 1:
                if len(curr_scan) >= 20:
                    now_perf = time.perf_counter()

                    with lidar_scan_lock:
                        lidar_latest_scan = list(curr_scan)

                    lidar_scan_counter += 1

                    dt = (
                        now_perf -
                        last_scan_perf
                    )

                    if dt > 0:
                        lidar_scan_hz = (
                            1.0 / dt
                        )

                    last_scan_perf = now_perf
                    lidar_last_scan_time = time.monotonic()

                curr_scan = []

            if (
                quality > 0 and
                LIDAR_MIN_DISTANCE_MM <= dist <= LIDAR_MAX_DISTANCE_MM
            ):
                a = math.radians(
                    lidar_normalize_angle(
                        angle_deg -
                        LIDAR_FRONT_OFFSET_DEG
                    )
                )

                x = (
                    dist *
                    math.cos(a)
                )

                # =================================================
                # ORIENTASI LIDAR FISIK TERPASANG MIRROR KIRI/KANAN
                #
                # Sebelumnya:
                #   +Y = kiri LiDAR
                #
                # Pada pemasangan robot ini hasilnya terbalik,
                # maka sumbu lateral dibalik:
                #   +Y = kiri ROBOT
                #   -Y = kanan ROBOT
                #
                # Dengan membalik Y di sini, SEMUA ikut benar:
                # - map radar
                # - LEFT / RIGHT
                # - wall following
                # - obstacle avoidance
                # - pilihan arah U-turn
                # =================================================
                y = -(
                    dist *
                    math.sin(a)
                )

                curr_scan.append(
                    (x, y)
                )

        # Debug tiap 2 detik kalau node ada tetapi revolution belum terbentuk.
        now_dbg = time.monotonic()

        if now_dbg - last_debug_time >= 2.0:
            last_debug_time = now_dbg

            if (
                lidar_connected and
                lidar_scan_counter == 0
            ):
                print(
                    "[LIDAR] bytes buffered={} valid_nodes={} curr_points={}"
                    .format(
                        len(buf),
                        valid_nodes_total,
                        len(curr_scan)
                    ),
                    flush=True
                )



def lidar_analyze(points):
    """
    BODY-CENTRIC LIDAR SAFETY GATE.

    Tidak menentukan "jalan ke mana" sendiri.
    LiDAR hanya menilai:
      - kiri body aman?
      - depan body aman?
      - kanan body aman?

    Orange circle = safety envelope (2x radius body).
    Yellow circle = physical body.

    Risk besar bila:
      - titik makin dekat ke yellow body
      - titik makin banyak
    """
    global lidar_target_angle

    left_pts = []
    center_pts = []
    right_pts = []

    nearest_left = float("inf")
    nearest_center = float("inf")
    nearest_right = float("inf")

    for x, y in points:
        d = math.hypot(x, y)

        if d <= 0:
            continue

        a = math.degrees(math.atan2(y, x))
        abs_a = abs(a)

        # Ignore known robot-side reflections inside body zone.
        side_self = (
            LIDAR_SELF_IGNORE_SIDE_START_DEG <= abs_a <=
            LIDAR_SELF_IGNORE_SIDE_END_DEG
        )

        if (
            d <= LIDAR_SELF_IGNORE_RADIUS_MM and
            side_self and
            x < 120.0
        ):
            continue

        # Only use forward hemisphere for forward driving safety.
        if x < 0:
            continue

        # ----------------------------------------------------
        # BODY-MATCHED SAFETY ZONE
        # Front and side reach are intentionally tighter.
        # ----------------------------------------------------

        # Front corridor: aligned with camera body width.
        in_front = (
            x <= LIDAR_FRONT_REACH_MM and
            abs(y) <= LIDAR_SIDE_HALF_WIDTH_MM
        )

        in_arrow_side = (
            in_front and
            abs(y) > 120
        )

        # Side zones: only close to robot body.
        in_left = (
            y > 120 and
            (
                d <= LIDAR_SIDE_REACH_MM or
                in_arrow_side
            )
        )

        in_right = (
            y < -120 and
            (
                d <= LIDAR_SIDE_REACH_MM or
                in_arrow_side
            )
        )

        if in_front and abs(y) <= 120:
            center_pts.append((x, y, d))
            nearest_center = min(nearest_center, d)

        elif in_left:
            left_pts.append((x, y, d))
            nearest_left = min(nearest_left, d)

        elif in_right:
            right_pts.append((x, y, d))
            nearest_right = min(nearest_right, d)

        else:
            continue

    def risk_for(items, zone_weight=1.0):
        if not items:
            return 0.0

        total = 0.0

        for x, y, d in items:
            clearance = max(
                0.0,
                d - ROBOT_RADIUS_MM
            )

            # Near yellow body -> risk rises strongly.
            denom = max(
                1.0,
                LIDAR_BODY_SAFETY_RADIUS_MM -
                ROBOT_RADIUS_MM
            )

            proximity = 1.0 - np.clip(
                clearance / denom,
                0.0,
                1.0
            )

            # Every point counts, but close points count much more.
            total += (
                0.08 +
                (proximity ** 2.2) *
                zone_weight
            )

        return float(total)

    left_score = risk_for(
        left_pts,
        1.0
    )

    center_score = risk_for(
        center_pts,
        1.35
    )

    right_score = risk_for(
        right_pts,
        1.0
    )
    left_count = len(left_pts)
    center_count = len(center_pts)
    right_count = len(right_pts)

    # Red safety circle guard: no valid LiDAR point may enter it.
    hard_left = (
        np.isfinite(nearest_left) and
        nearest_left <= LIDAR_RED_SAFETY_RADIUS_MM
    )

    hard_center = (
        np.isfinite(nearest_center) and
        nearest_center <= LIDAR_RED_SAFETY_RADIUS_MM
    )

    hard_right = (
        np.isfinite(nearest_right) and
        nearest_right <= LIDAR_RED_SAFETY_RADIUS_MM
    )

    # For UI compatibility
    front = nearest_center
    fl = nearest_left
    fr = nearest_right
    left = nearest_left
    right = nearest_right

    lidar_target_angle = 0.0

    side_margin = 0.75

    def avoid_steer(nearest_d):
        if not np.isfinite(nearest_d):
            return LIDAR_PREWARN_STEER_MIN

        if nearest_d <= ROBOT_RADIUS_MM:
            return LIDAR_PREWARN_STEER_MAX

        if nearest_d <= LIDAR_RED_SAFETY_RADIUS_MM:
            t = (
                LIDAR_RED_SAFETY_RADIUS_MM -
                nearest_d
            ) / max(
                1.0,
                LIDAR_RED_SAFETY_RADIUS_MM -
                ROBOT_RADIUS_MM
            )
            return float(
                np.clip(
                    FUSION_OVERRIDE_HARD +
                    t * (
                        LIDAR_PREWARN_STEER_MAX -
                        FUSION_OVERRIDE_HARD
                    ),
                    FUSION_OVERRIDE_HARD,
                    LIDAR_PREWARN_STEER_MAX
                )
            )

        prewarn = max(
            LIDAR_SIDE_PREWARN_MM,
            LIDAR_FRONT_PREWARN_MM
        )

        t = (
            prewarn -
            nearest_d
        ) / max(
            1.0,
            prewarn -
            LIDAR_RED_SAFETY_RADIUS_MM
        )

        return float(
            np.clip(
                LIDAR_PREWARN_STEER_MIN +
                t * (
                    FUSION_OVERRIDE_HARD -
                    LIDAR_PREWARN_STEER_MIN
                ),
                LIDAR_PREWARN_STEER_MIN,
                FUSION_OVERRIDE_HARD
            )
        )

    prewarn_left = (
        np.isfinite(nearest_left) and
        nearest_left <= LIDAR_FRONT_PREWARN_MM
    )
    prewarn_center = (
        np.isfinite(nearest_center) and
        nearest_center <= LIDAR_FRONT_PREWARN_MM
    )
    prewarn_right = (
        np.isfinite(nearest_right) and
        nearest_right <= LIDAR_FRONT_PREWARN_MM
    )

    nearest_any = min(
        v for v in (
            nearest_left,
            nearest_center,
            nearest_right
        )
        if np.isfinite(v)
    ) if any(
        np.isfinite(v)
        for v in (
            nearest_left,
            nearest_center,
            nearest_right
        )
    ) else float("inf")

    if hard_left and hard_center and hard_right:
        if nearest_any <= ROBOT_RADIUS_MM:
            action = "STOP"
            steer = 0.0
            reason = "LIDAR YELLOW BODY TOUCH"
        elif (
            right_count > left_count or
            nearest_right < nearest_left or
            left_score + side_margin < right_score
        ):
            action = "AVOID_LEFT"
            steer = -avoid_steer(nearest_any)
            reason = "LIDAR RED ALL, RIGHT WORSE -> LEFT"
        elif (
            left_count > right_count or
            nearest_left < nearest_right or
            right_score + side_margin < left_score
        ):
            action = "AVOID_RIGHT"
            steer = avoid_steer(nearest_any)
            reason = "LIDAR RED ALL, LEFT WORSE -> RIGHT"
        else:
            action = "AVOID_LEFT"
            steer = -avoid_steer(nearest_any)
            reason = "LIDAR RED ALL -> DEFAULT LEFT"
    elif hard_center:
        if (
            right_count > left_count or
            left_score + side_margin < right_score
        ):
            action = "AVOID_LEFT"
            steer = -avoid_steer(nearest_any)
            reason = "LIDAR RED CENTER, RIGHT RISK HIGH -> LEFT"
        elif (
            left_count > right_count or
            right_score + side_margin < left_score
        ):
            action = "AVOID_RIGHT"
            steer = avoid_steer(nearest_any)
            reason = "LIDAR RED CENTER, LEFT RISK HIGH -> RIGHT"
        elif (
            nearest_left if np.isfinite(nearest_left) else 9999.0
        ) >= (
            nearest_right if np.isfinite(nearest_right) else 9999.0
        ):
            action = "AVOID_LEFT"
            steer = -avoid_steer(nearest_any)
            reason = "LIDAR RED CENTER -> LEFT CLEARER"
        else:
            action = "AVOID_RIGHT"
            steer = avoid_steer(nearest_any)
            reason = "LIDAR RED CENTER -> RIGHT CLEARER"
    elif hard_left and hard_right:
        if right_count > left_count:
            action = "AVOID_LEFT"
            steer = -avoid_steer(nearest_any)
            reason = "LIDAR RED RIGHT POINTS MORE -> LEFT"
        elif left_count > right_count:
            action = "AVOID_RIGHT"
            steer = avoid_steer(nearest_any)
            reason = "LIDAR RED LEFT POINTS MORE -> RIGHT"
        elif abs(left_score - right_score) <= side_margin and not hard_center:
            action = "CLEAR"
            steer = 0.0
            reason = "LIDAR RED BOTH SIDES BALANCED -> CENTER"
        elif left_score < right_score:
            action = "AVOID_LEFT"
            steer = -avoid_steer(nearest_any)
            reason = "LIDAR RED RIGHT RISK HIGH -> LEFT"
        else:
            action = "AVOID_RIGHT"
            steer = avoid_steer(nearest_any)
            reason = "LIDAR RED LEFT RISK HIGH -> RIGHT"
    elif hard_left:
        action = "AVOID_RIGHT"
        steer = avoid_steer(nearest_left)
        reason = "LIDAR RED LEFT -> RIGHT"
    elif hard_right:
        action = "AVOID_LEFT"
        steer = -avoid_steer(nearest_right)
        reason = "LIDAR RED RIGHT -> LEFT"
    elif prewarn_center or prewarn_left or prewarn_right:
        if prewarn_center:
            if right_count > left_count or left_score + side_margin < right_score:
                action = "AVOID_LEFT"
                steer = -avoid_steer(nearest_any)
                reason = "LIDAR PREWARN FRONT, RIGHT DENSE -> LEFT"
            elif left_count > right_count or right_score + side_margin < left_score:
                action = "AVOID_RIGHT"
                steer = avoid_steer(nearest_any)
                reason = "LIDAR PREWARN FRONT, LEFT DENSE -> RIGHT"
            elif (
                nearest_left if np.isfinite(nearest_left) else 9999.0
            ) >= (
                nearest_right if np.isfinite(nearest_right) else 9999.0
            ):
                action = "AVOID_LEFT"
                steer = -avoid_steer(nearest_any)
                reason = "LIDAR PREWARN FRONT -> LEFT CLEARER"
            else:
                action = "AVOID_RIGHT"
                steer = avoid_steer(nearest_any)
                reason = "LIDAR PREWARN FRONT -> RIGHT CLEARER"
        elif prewarn_left and prewarn_right:
            if right_count > left_count or left_score + side_margin < right_score:
                action = "AVOID_LEFT"
                steer = -avoid_steer(nearest_any)
                reason = "LIDAR PREWARN RIGHT DENSE -> LEFT"
            elif left_count > right_count or right_score + side_margin < left_score:
                action = "AVOID_RIGHT"
                steer = avoid_steer(nearest_any)
                reason = "LIDAR PREWARN LEFT DENSE -> RIGHT"
            else:
                action = "CLEAR"
                steer = 0.0
                reason = "LIDAR PREWARN BOTH SIDES BALANCED -> CENTER"
        elif prewarn_left:
            action = "AVOID_RIGHT"
            steer = avoid_steer(nearest_left)
            reason = "LIDAR PREWARN LEFT -> RIGHT"
        else:
            action = "AVOID_LEFT"
            steer = -avoid_steer(nearest_right)
            reason = "LIDAR PREWARN RIGHT -> LEFT"
    else:
        action = "CLEAR"
        steer = 0.0
        reason = "BODY SAFETY L:{:.1f} C:{:.1f} R:{:.1f}".format(
            left_score,
            center_score,
            right_score
        )

    return {
        "action": action,
        "reason": reason,
        "front": front,
        "fl": fl,
        "fr": fr,
        "left": left,
        "right": right,
        "steer": float(steer),
        "target_angle": 0.0,
        "left_score": left_score,
        "center_score": center_score,
        "right_score": right_score,
        "left_count": left_count,
        "center_count": center_count,
        "right_count": right_count,
        "hard_left": bool(hard_left),
        "hard_center": bool(hard_center),
        "hard_right": bool(hard_right),
    }



def lidar_navigation_loop():
    global lidar_action, lidar_reason, lidar_steer_hint, lidar_turn_dir
    global lidar_left_score, lidar_center_score, lidar_right_score
    global lidar_left_count, lidar_center_count, lidar_right_count
    global lidar_hard_left, lidar_hard_center, lidar_hard_right
    global lidar_target_angle
    global lidar_front, lidar_front_left, lidar_front_right
    global lidar_left, lidar_right

    while running:
        age = time.monotonic() - lidar_last_scan_time

        with lidar_scan_lock:
            scan = list(lidar_latest_scan)

        if (
            not lidar_connected or
            not scan or
            age > LIDAR_TIMEOUT_SEC
        ):
            result = {
                "action": "TIMEOUT",
                "reason": "LIDAR TIMEOUT",
                "front": float("inf"),
                "fl": float("inf"),
                "fr": float("inf"),
                "left": float("inf"),
                "right": float("inf"),
                "steer": 0.0,
            }
        else:
            result = lidar_analyze(scan)

        lidar_action = result["action"]
        lidar_reason = result["reason"]
        lidar_steer_hint = float(result["steer"])
        lidar_target_angle = float(
            result.get(
                "target_angle",
                lidar_target_angle
            )
        )
        lidar_turn_dir = result.get("turn_dir", lidar_turn_dir)
        lidar_front = result["front"]
        lidar_front_left = result["fl"]
        lidar_front_right = result["fr"]
        lidar_left = result["left"]
        lidar_right = result["right"]

        # IMPORTANT:
        # Keep current body-risk state synchronized with lidar_analyze().
        lidar_left_score = float(
            result.get("left_score", 0.0)
        )
        lidar_center_score = float(
            result.get("center_score", 0.0)
        )
        lidar_right_score = float(
            result.get("right_score", 0.0)
        )
        lidar_left_count = int(
            result.get("left_count", 0)
        )
        lidar_center_count = int(
            result.get("center_count", 0)
        )
        lidar_right_count = int(
            result.get("right_count", 0)
        )

        lidar_hard_left = bool(
            result.get("hard_left", False)
        )
        lidar_hard_center = bool(
            result.get("hard_center", False)
        )
        lidar_hard_right = bool(
            result.get("hard_right", False)
        )

        time.sleep(0.02)


def lidar_fallback_steering():
    if lidar_action == "TIMEOUT":
        return None

    if lidar_action == "STOP":
        return 0.0

    return float(lidar_steer_hint)



def fmt_lidar_mm(v):
    return "---" if not np.isfinite(v) else "{:.0f}".format(v)



# ============================================================
# LIDAR MAP WINDOW - STYLE PROGRAM USER
# ============================================================

LIDAR_WINDOW_NAME = "RPLIDAR A1M8 - NAVIGATION MAP"
LIDAR_MAP_W = 820
LIDAR_MAP_H = 820
LIDAR_CENTER_X = LIDAR_MAP_W // 2
LIDAR_CENTER_Y = LIDAR_MAP_H // 2
LIDAR_VIEW_RANGE_MM = 3000.0

LIDAR_SCALE = (
    min(
        LIDAR_MAP_W,
        LIDAR_MAP_H
    ) /
    (
        LIDAR_VIEW_RANGE_MM *
        2.0
    )
)


def create_lidar_background():
    img = np.zeros(
        (
            LIDAR_MAP_H,
            LIDAR_MAP_W,
            3
        ),
        dtype=np.uint8
    )

    img[:] = (
        4,
        20,
        4
    )

    # Grid 25 cm.
    grid_px = max(
        1,
        int(
            250.0 *
            LIDAR_SCALE
        )
    )

    for x in range(
        LIDAR_CENTER_X,
        LIDAR_MAP_W,
        grid_px
    ):
        cv2.line(
            img,
            (x, 0),
            (x, LIDAR_MAP_H),
            (8, 38, 8),
            1
        )

    for x in range(
        LIDAR_CENTER_X - grid_px,
        -1,
        -grid_px
    ):
        cv2.line(
            img,
            (x, 0),
            (x, LIDAR_MAP_H),
            (8, 38, 8),
            1
        )

    for y in range(
        LIDAR_CENTER_Y,
        LIDAR_MAP_H,
        grid_px
    ):
        cv2.line(
            img,
            (0, y),
            (LIDAR_MAP_W, y),
            (8, 38, 8),
            1
        )

    for y in range(
        LIDAR_CENTER_Y - grid_px,
        -1,
        -grid_px
    ):
        cv2.line(
            img,
            (0, y),
            (LIDAR_MAP_W, y),
            (8, 38, 8),
            1
        )

    cv2.line(
        img,
        (0, LIDAR_CENTER_Y),
        (LIDAR_MAP_W, LIDAR_CENTER_Y),
        (25, 80, 25),
        1
    )

    cv2.line(
        img,
        (LIDAR_CENTER_X, 0),
        (LIDAR_CENTER_X, LIDAR_MAP_H),
        (25, 80, 25),
        1
    )

    # Body robot.
    body_radius_px = int(
        ROBOT_RADIUS_MM *
        LIDAR_SCALE
    )

    cv2.circle(
        img,
        (
            LIDAR_CENTER_X,
            LIDAR_CENTER_Y
        ),
        body_radius_px,
        (0, 255, 255),
        2,
        cv2.LINE_AA
    )

    # Orange ring = exactly 2x BODY radius.
    # Yellow body radius = ROBOT_RADIUS_MM
    # Orange radius      = 2 * ROBOT_RADIUS_MM
    danger_radius_px = int(
        LIDAR_RED_SAFETY_RADIUS_MM *
        LIDAR_SCALE
    )

    cv2.circle(
        img,
        (
            LIDAR_CENTER_X,
            LIDAR_CENTER_Y
        ),
        danger_radius_px,
        (0, 80, 255),
        1,
        cv2.LINE_AA
    )

    # Ring distance.
    for r_mm in (
        500,
        1000,
        1500,
        2000,
        2500,
        3000
    ):
        r_px = int(
            r_mm *
            LIDAR_SCALE
        )

        cv2.circle(
            img,
            (
                LIDAR_CENTER_X,
                LIDAR_CENTER_Y
            ),
            r_px,
            (20, 80, 20),
            1,
            cv2.LINE_AA
        )

    # Front arrow = +X, arah maju robot.
    arrow_len = int(
        LIDAR_FRONT_PREWARN_MM *
        LIDAR_SCALE
    )

    cv2.arrowedLine(
        img,
        (
            LIDAR_CENTER_X,
            LIDAR_CENTER_Y
        ),
        (
            LIDAR_CENTER_X +
            arrow_len,
            LIDAR_CENTER_Y
        ),
        (0, 0, 255),
        3,
        cv2.LINE_AA,
        tipLength=0.18
    )

    return img


LIDAR_STATIC_BG = create_lidar_background()


def render_lidar_map():
    """
    Render point cloud LiDAR + informasi navigasi utama.
    """
    with lidar_scan_lock:
        points = list(
            lidar_latest_scan
        )

    with ros_nav_lock:
        odom = ros_nav_odom
        path = list(ros_nav_path)

    img = LIDAR_STATIC_BG.copy()

    if odom is not None and len(path) >= 2:
        rx, ry, yaw = odom
        c = math.cos(-yaw)
        s = math.sin(-yaw)

        nearest_i = min(
            range(len(path)),
            key=lambda i: math.hypot(
                path[i][0] - rx,
                path[i][1] - ry
            )
        )

        start_i = max(0, nearest_i - ROS_PATH_DRAW_LOOKAHEAD_POINTS)
        end_i = min(len(path), nearest_i + ROS_PATH_DRAW_LOOKAHEAD_POINTS)
        draw_points = []

        for wx, wy in path[start_i:end_i]:
            dx = wx - rx
            dy = wy - ry
            local_x_m = c * dx - s * dy
            local_y_m = s * dx + c * dy

            px = int(
                LIDAR_CENTER_X +
                local_x_m * ROS_PATH_DRAW_SCALE_PX_PER_M
            )
            py = int(
                LIDAR_CENTER_Y -
                local_y_m * ROS_PATH_DRAW_SCALE_PX_PER_M
            )

            if 0 <= px < LIDAR_MAP_W and 0 <= py < LIDAR_MAP_H:
                draw_points.append((px, py))

        if len(draw_points) >= 2:
            cv2.polylines(
                img,
                [np.array(draw_points, dtype=np.int32)],
                False,
                (0, 255, 0),
                3,
                cv2.LINE_AA
            )

            cv2.circle(
                img,
                draw_points[nearest_i - start_i]
                if 0 <= nearest_i - start_i < len(draw_points)
                else draw_points[-1],
                6,
                (0, 255, 255),
                -1,
                cv2.LINE_AA
            )

    for x, y in points:
        d = math.hypot(x, y)

        px = int(
            LIDAR_CENTER_X +
            x * LIDAR_SCALE
        )

        py = int(
            LIDAR_CENTER_Y -
            y * LIDAR_SCALE
        )

        if not (
            0 <= px < LIDAR_MAP_W and
            0 <= py < LIDAR_MAP_H
        ):
            continue

        a_map = abs(
            math.degrees(
                math.atan2(
                    y,
                    x
                )
            )
        )

        side_self = (
            d <= LIDAR_SELF_IGNORE_RADIUS_MM and
            LIDAR_SELF_IGNORE_SIDE_START_DEG <= a_map <=
            LIDAR_SELF_IGNORE_SIDE_END_DEG
        )

        if side_self:
            # Abu-abu = diabaikan sebagai komponen samping robot.
            cv2.circle(
                img,
                (px, py),
                2,
                (80, 80, 80),
                -1
            )
        else:
            # Hijau = tetap aktif untuk navigasi/obstacle.
            cv2.circle(
                img,
                (px, py),
                2,
                (60, 255, 60),
                -1
            )

    # Robot center.
    cv2.circle(
        img,
        (
            LIDAR_CENTER_X,
            LIDAR_CENTER_Y
        ),
        7,
        (0, 220, 255),
        -1
    )

    # Panel info.
    overlay = img.copy()

    cv2.rectangle(
        overlay,
        (12, 12),
        (580, 245),
        (0, 10, 0),
        -1
    )

    cv2.addWeighted(
        overlay,
        0.40,
        img,
        0.60,
        0,
        img
    )

    def txt(
        text,
        y,
        color=(180, 255, 180),
        scale=0.48,
        thick=1
    ):
        cv2.putText(
            img,
            text,
            (25, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            scale,
            color,
            thick,
            cv2.LINE_AA
        )

    txt(
        "LIDAR: {} | SCAN {:.1f}Hz | #{}".format(
            "CONNECTED"
            if lidar_connected
            else "DISCONNECTED",
            lidar_scan_hz,
            lidar_scan_counter
        ),
        38,
        (
            (60, 255, 60)
            if lidar_connected
            else (0, 0, 255)
        ),
        0.52,
        2
    )

    txt(
        "ACTION: {}".format(
            lidar_action
        ),
        68,
        (
            (0, 0, 255)
            if lidar_action == "STOP"
            else (
                (0, 165, 255)
                if lidar_action in (
                    "AVOID_LEFT",
                    "AVOID_RIGHT",
                    "UTURN"
                )
                else (60, 255, 60)
            )
        ),
        0.58,
        2
    )

    txt(
        "REASON: {}".format(
            lidar_reason
        ),
        98,
        (255, 255, 255),
        0.45,
        1
    )

    txt(
        "FRONT:{}  FL:{}  FR:{}".format(
            fmt_lidar_mm(lidar_front),
            fmt_lidar_mm(lidar_front_left),
            fmt_lidar_mm(lidar_front_right)
        ),
        132
    )

    txt(
        "LEFT:{}   RIGHT:{}".format(
            fmt_lidar_mm(lidar_left),
            fmt_lidar_mm(lidar_right)
        ),
        162
    )

    txt(
        "YELLOW=BODY | ORANGE=SAFETY R=650mm",
        194,
        (255, 255, 0),
        0.44,
        1
    )

    txt(
        "ROS /scan_front | FRONT/BACK FLIP | SPACE START/STOP",
        224,
        (255, 255, 255),
        0.44,
        1
    )

    cv2.putText(
        img,
        "BODY RISK L:{:.1f} C:{:.1f} R:{:.1f}".format(
            lidar_left_score,
            lidar_center_score,
            lidar_right_score
        ),
        (25, 270),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.48,
        (255, 255, 255),
        1,
        cv2.LINE_AA
    )

    cv2.imshow(
        LIDAR_WINDOW_NAME,
        img
    )


# ============================================================
# LOCAL START / STOP
# SPACE = toggle START / STOP
# Default OFF untuk keamanan.
# ============================================================

robot_enabled = False
robot_lock = threading.Lock()

WEB_REMOTE_HOST = "0.0.0.0"
WEB_REMOTE_PORT = 8899
MANUAL_FORWARD_STEER = 0.0
MANUAL_REVERSE_RPM = MOTOR_REVERSE
MANUAL_UTURN_SEC = 3.35

manual_lock = threading.Lock()
manual_mode = False
manual_cmd = "STOP"
manual_uturn_started = 0.0
manual_last_status = "MANUAL READY"
manual_hold_mode = False

web_frame_lock = threading.Lock()
latest_web_frame = None

WINDOW_NAME = "FREEDOM - EDGE BOUNDARY ONLY + LIDAR"

# ============================================================
# LAYOUT WINDOW HDMI
# Layout window laptop.
# ============================================================
MAIN_WIN_W = 640
MAIN_WIN_H = 480


def get_screen_size():
    """
    Deteksi resolusi monitor aktif via xrandr.
    Fallback ke 1920x1080 jika xrandr tidak tersedia.
    """
    try:
        out = subprocess.check_output(
            "xrandr 2>/dev/null | grep '\\*' | head -n1",
            shell=True
        ).decode("utf-8", errors="ignore").strip()

        # Contoh: 1920x1080  60.00*
        for token in out.split():
            if "x" in token and token[0].isdigit():
                w, h = token.split("x", 1)
                return int(w), int(h)
    except Exception:
        pass

    return 1920, 1080


running = True
camera_lock = threading.Lock()
latest_camera = None
latest_camera_id = 0
latest_camera_ts = 0.0

camera_fps = 0.0
processing_fps = 0.0
processing_ms = 0.0
hough_ms = 0.0
frame_age_ms = 0.0

gst_camera = None
ROI_Y = int(FRAME_H * ROI_START_RATIO)


# ============================================================
# BODY ROBOT - PERSIS KOORDINAT DESMOS REFERENSI
# ============================================================

# ============================================================
# BODY ROBOT - GEOMETRI DESMOS FINAL
#
# Kiri belakang : (-5.00, -2.80)  # dinaikkan sedikit dari pojok bawah
# Kiri depan    : (-3.75, -2.00)  # diperlebar agar zona merah tidak terlalu sempit
#
# Sisi kanan = mirror terhadap sumbu x=0:
# Kanan belakang: ( 5.00, -2.80)
# Kanan depan   : ( 3.75, -2.00)
#
# Garis depan = hubungkan kiri depan ke kanan depan.
# Garis belakang = hubungkan kiri belakang ke kanan belakang.
# ============================================================

DESMOS_CENTER_X = 0.0
DESMOS_CENTER_Y = 0.0
DESMOS_WIDTH = 10.0
DESMOS_HEIGHT = 6.0

BODY_LEFT_BOTTOM = (-5.0, -2.80)
BODY_LEFT_TOP = (-3.75, -2.0)


def mirror_x_desmos(x):
    # Mirror persis terhadap sumbu tengah x = 0.
    return -x


def desmos_to_pixel(xd, yd, width_px, height_px):
    left_d = DESMOS_CENTER_X - (DESMOS_WIDTH / 2.0)
    top_d = DESMOS_CENTER_Y + (DESMOS_HEIGHT / 2.0)

    x = (xd - left_d) / DESMOS_WIDTH * (width_px - 1)
    y = (top_d - yd) / DESMOS_HEIGHT * (height_px - 1)

    x = int(round(max(0, min(width_px - 1, x))))
    y = int(round(max(0, min(height_px - 1, y))))

    return np.array([x, y], dtype=np.float32)


def lerp_point(p1, p2, t):
    return (1.0 - t) * p1 + t * p2


def build_body_points(width_px, height_px):
    """
    Body trapezoid final berdasarkan titik Desmos:

      kiri_depan  (-3.75, -2.00) ----- ( 3.75, -2.00) kanan_depan
                      /                         \
                     /                           \
      kiri_belakang (-5.00, -2.80) ----- ( 5.00, -2.80) kanan_belakang

    Urutan return:
      kiri_atas, kanan_atas, kanan_bawah, kiri_bawah
    """

    x_lb, y_lb = BODY_LEFT_BOTTOM
    x_lt, y_lt = BODY_LEFT_TOP

    x_rb = mirror_x_desmos(x_lb)
    x_rt = mirror_x_desmos(x_lt)

    # y kanan sama karena hasil mirror hanya pada sumbu x.
    y_rb = y_lb
    y_rt = y_lt

    kiri_bawah = desmos_to_pixel(
        x_lb, y_lb,
        width_px, height_px
    )

    kiri_atas = desmos_to_pixel(
        x_lt, y_lt,
        width_px, height_px
    )

    kanan_bawah = desmos_to_pixel(
        x_rb, y_rb,
        width_px, height_px
    )

    kanan_atas = desmos_to_pixel(
        x_rt, y_rt,
        width_px, height_px
    )

    return (
        kiri_atas,
        kanan_atas,
        kanan_bawah,
        kiri_bawah
    )

def draw_robot_body_4colors(frame, body_points, alpha=0.33):
    kiri_atas, kanan_atas, kanan_bawah, kiri_bawah = body_points
    overlay = frame.copy()

    # BGR: merah, kuning, hijau, biru
    colors = [
        (0, 0, 255),
        (0, 255, 255),
        (0, 255, 0),
        (255, 0, 0),
    ]

    for i in range(4):
        t0 = i / 4.0
        t1 = (i + 1) / 4.0

        p_l0 = lerp_point(kiri_atas, kiri_bawah, t0)
        p_l1 = lerp_point(kiri_atas, kiri_bawah, t1)
        p_r0 = lerp_point(kanan_atas, kanan_bawah, t0)
        p_r1 = lerp_point(kanan_atas, kanan_bawah, t1)

        poly = np.array([p_l0, p_r0, p_r1, p_l1], dtype=np.int32)
        cv2.fillPoly(overlay, [poly], colors[i])

    cv2.addWeighted(overlay, alpha, frame, 1.0 - alpha, 0, frame)

    outline = np.array(
        [kiri_atas, kanan_atas, kanan_bawah, kiri_bawah],
        dtype=np.int32
    )
    cv2.polylines(frame, [outline], True, (255, 255, 255), 2)

    for i in range(1, 4):
        t = i / 4.0
        p_l = lerp_point(kiri_atas, kiri_bawah, t).astype(np.int32)
        p_r = lerp_point(kanan_atas, kanan_bawah, t).astype(np.int32)
        cv2.line(frame, tuple(p_l), tuple(p_r), (255, 255, 255), 1)


# ============================================================
# STEERING -> RPM (SAMA DENGAN REFERENSI)
# ============================================================

def steering_to_rpm(steering):
    """
    Smooth differential steering - arah fisik SUDAH DIBALIK.

    Motor actual:
      97  = maju
      127 = stop
      157 = mundur

    steering:
      +100 = kanan
      -100 = kiri

    Koreksi fisik final:
      kanan -> roda KANAN diperlambat
      kiri  -> roda KIRI diperlambat
    """
    s = float(np.clip(steering, -100.0, 100.0))
    mag = abs(s) / 100.0

    full = MOTOR_FORWARD  # 97
    slow = int(round(
        MOTOR_FORWARD +
        (MOTOR_NEUTRAL - MOTOR_FORWARD) * mag
    ))

    if abs(s) < 5.0:
        return full, full

    if s > 0:
        # Belok kanan: roda kanan lebih lambat.
        return full, slow

    # Belok kiri: roda kiri lebih lambat.
    return slow, full



def motor_spin_left():
    """Putar di tempat ke kiri - mapping fisik final."""
    return MOTOR_REVERSE, MOTOR_FORWARD


def motor_spin_right():
    """Putar di tempat ke kanan - mapping fisik final."""
    return MOTOR_FORWARD, MOTOR_REVERSE


def motor_stop():
    return MOTOR_NEUTRAL, MOTOR_NEUTRAL


def motor_spin_turnback(direction="LEFT"):
    """
    Putar balik di tempat menggunakan satu roda maju
    dan satu roda mundur.
    """
    if direction == "RIGHT":
        return motor_spin_right()
    return motor_spin_left()



# ============================================================
# CAMERA
# ============================================================

def stop_all(*_):
    global running
    running = False

    # Fail-safe: setiap program berhenti, motor STOP.
    try:
        control_send_rpm(
            MOTOR_NEUTRAL,
            MOTOR_NEUTRAL,
            force=True
        )
    except Exception:
        pass

    if gst_camera is not None:
        try:
            gst_camera.terminate()
        except Exception:
            pass

    lidar_close()
    control_disconnect()


def read_exact(fd, size):
    chunks = []
    remaining = size

    while running and remaining > 0:
        try:
            data = os.read(fd, remaining)
        except InterruptedError:
            continue
        except Exception:
            return None

        if not data:
            return None

        chunks.append(data)
        remaining -= len(data)

    if remaining:
        return None

    return b"".join(chunks)


def camera_loop():
    """
    Kamera HP -> laptop.

    Urutan:
      1. Coba OpenCV/FFmpeg langsung MJPEG.
      2. Kalau gagal, fallback ke pipeline GStreamer lama.
      3. Reconnect otomatis.

    latest_camera hanya di-update setelah frame valid.
    """
    global gst_camera
    global latest_camera
    global latest_camera_id
    global latest_camera_ts
    global camera_fps
    global latest_web_frame

    while running:
        print(
            "[CAM] connecting {}".format(
                MJPEG_URL
            ),
            flush=True
        )

        # ----------------------------------------------------
        # METHOD 1: OpenCV VideoCapture
        # ----------------------------------------------------
        cap = None

        try:
            cap = cv2.VideoCapture(
                MJPEG_URL
            )

            cap.set(
                cv2.CAP_PROP_BUFFERSIZE,
                1
            )

            if cap.isOpened():
                print(
                    "[CAM] ACTIVE via OpenCV",
                    flush=True
                )

                count = 0
                t0 = time.monotonic()
                consecutive_fail = 0

                while running:
                    ok, frame = cap.read()

                    if (
                        not ok or
                        frame is None or
                        frame.size == 0
                    ):
                        consecutive_fail += 1

                        if consecutive_fail >= 20:
                            print(
                                "[CAM] OpenCV stream lost",
                                flush=True
                            )
                            break

                        time.sleep(0.02)
                        continue

                    consecutive_fail = 0

                    if (
                        frame.shape[1] != FRAME_W or
                        frame.shape[0] != FRAME_H
                    ):
                        frame = cv2.resize(
                            frame,
                            (FRAME_W, FRAME_H),
                            interpolation=cv2.INTER_LINEAR
                        )

                    now = time.monotonic()

                    with camera_lock:
                        latest_camera = frame.copy()
                        latest_camera_id += 1
                        latest_camera_ts = now

                    with web_frame_lock:
                        latest_web_frame = frame.copy()

                    count += 1

                    if now - t0 >= 1.0:
                        camera_fps = (
                            count /
                            max(
                                0.001,
                                now - t0
                            )
                        )

                        count = 0
                        t0 = now

                try:
                    cap.release()
                except Exception:
                    pass

                if running:
                    time.sleep(0.3)

                continue

        except Exception as e:
            print(
                "[CAM] OpenCV error:",
                e,
                flush=True
            )

        if cap is not None:
            try:
                cap.release()
            except Exception:
                pass

        # ----------------------------------------------------
        # METHOD 2: GStreamer fallback
        # ----------------------------------------------------
        print(
            "[CAM] OpenCV gagal, fallback GStreamer",
            flush=True
        )

        cmd = (
            "/usr/bin/gst-launch-1.0 -q "
            "souphttpsrc location={} is-live=true do-timestamp=true ! "
            "multipartdemux ! image/jpeg ! jpegparse ! "
            "queue leaky=downstream max-size-buffers=1 "
            "max-size-bytes=0 max-size-time=0 ! "
            "avdec_mjpeg ! videoconvert ! "
            "videoscale ! "
            "video/x-raw,format=BGR,width={},height={} ! "
            "fdsink fd=1 sync=false"
        ).format(
            MJPEG_URL,
            FRAME_W,
            FRAME_H
        )

        try:
            gst_camera = subprocess.Popen(
                cmd,
                shell=True,
                executable="/bin/bash",
                stdout=subprocess.PIPE,
                stderr=None,
                bufsize=0
            )

            print(
                "[CAM] ACTIVE via GStreamer",
                flush=True
            )

            fd = gst_camera.stdout.fileno()

            count = 0
            t0 = time.monotonic()

            while running:
                raw = read_exact(
                    fd,
                    FRAME_BYTES
                )

                if raw is None:
                    print(
                        "[CAM] GStreamer disconnected",
                        flush=True
                    )
                    break

                frame = np.frombuffer(
                    raw,
                    dtype=np.uint8
                ).reshape(
                    (FRAME_H, FRAME_W, 3)
                ).copy()

                now = time.monotonic()

                with camera_lock:
                    latest_camera = frame
                    latest_camera_id += 1
                    latest_camera_ts = now

                with web_frame_lock:
                    latest_web_frame = frame.copy()

                count += 1

                if now - t0 >= 1.0:
                    camera_fps = (
                        count /
                        max(
                            0.001,
                            now - t0
                        )
                    )

                    count = 0
                    t0 = now

        except Exception as e:
            print(
                "[CAM] GStreamer error:",
                e,
                flush=True
            )

        try:
            if gst_camera is not None:
                gst_camera.terminate()
        except Exception:
            pass

        if running:
            time.sleep(0.5)




def toggle_robot():
    """
    SPACE toggle:
      OFF -> ON
      ON  -> OFF

    Saat OFF, langsung kirim command STOP 127/127.
    """
    global robot_enabled

    with robot_lock:
        robot_enabled = not robot_enabled
        enabled = robot_enabled

    if enabled:
        print("[LOCAL] START", flush=True)
    else:
        control_send_rpm(
            MOTOR_NEUTRAL,
            MOTOR_NEUTRAL,
            force=True
        )
        print("[LOCAL] STOP", flush=True)

    return enabled


def set_auto_enabled(enabled):
    global robot_enabled
    global manual_mode
    global manual_cmd
    global manual_last_status

    enabled = bool(enabled)

    with robot_lock:
        robot_enabled = enabled

    with manual_lock:
        manual_mode = False
        manual_cmd = "STOP"
        manual_last_status = (
            "AUTO ON"
            if enabled
            else "AUTO OFF"
        )

    if not enabled:
        control_send_rpm(
            MOTOR_NEUTRAL,
            MOTOR_NEUTRAL,
            force=True
        )

    print(
        "[WEB] {}".format(
            "AUTO START"
            if enabled
            else "AUTO STOP"
        ),
        flush=True
    )


def set_manual_command(cmd):
    global robot_enabled
    global manual_mode
    global manual_cmd
    global manual_uturn_started
    global manual_last_status
    global manual_hold_mode

    cmd = str(cmd).upper()

    with robot_lock:
        robot_enabled = False

    with manual_lock:
        manual_mode = True
        manual_cmd = cmd
        manual_hold_mode = cmd in (
            "FORWARD",
            "BACKWARD",
            "LEFT",
            "RIGHT"
        )
        manual_last_status = "MANUAL {}".format(cmd)

        if cmd == "UTURN_LEFT":
            manual_uturn_started = time.monotonic()
        elif cmd == "UTURN_RIGHT":
            manual_uturn_started = time.monotonic()
        else:
            manual_uturn_started = 0.0

    if cmd == "STOP":
        with manual_lock:
            manual_mode = False
            manual_hold_mode = False
            manual_last_status = "MANUAL STOP"

        control_send_rpm(
            MOTOR_NEUTRAL,
            MOTOR_NEUTRAL,
            force=True
        )

    print(
        "[WEB] MANUAL {}".format(cmd),
        flush=True
    )


def release_manual_hold():
    global manual_mode
    global manual_cmd
    global manual_hold_mode
    global manual_last_status

    with manual_lock:
        if not manual_hold_mode:
            return

        manual_mode = False
        manual_hold_mode = False
        manual_cmd = "STOP"
        manual_last_status = "MANUAL RELEASE STOP"

    control_send_rpm(
        MOTOR_NEUTRAL,
        MOTOR_NEUTRAL,
        force=True
    )
    print("[WEB] MANUAL RELEASE STOP", flush=True)


def get_manual_override(now):
    global manual_mode
    global manual_cmd
    global manual_last_status

    with manual_lock:
        active = manual_mode
        cmd = manual_cmd
        started = manual_uturn_started

    if not active:
        return None

    if cmd == "FORWARD":
        return steering_to_rpm(MANUAL_FORWARD_STEER), "MANUAL MAJU"

    if cmd == "BACKWARD":
        return (
            (MANUAL_REVERSE_RPM, MANUAL_REVERSE_RPM),
            "MANUAL MUNDUR"
        )

    if cmd == "LEFT":
        return (
            (MOTOR_NEUTRAL, MOTOR_FORWARD),
            "MANUAL KIRI SATU RODA"
        )

    if cmd == "RIGHT":
        return (
            (MOTOR_FORWARD, MOTOR_NEUTRAL),
            "MANUAL KANAN SATU RODA"
        )

    if cmd == "UTURN_LEFT":
        elapsed = now - started

        if elapsed < MANUAL_UTURN_SEC:
            return (
                motor_spin_turnback("LEFT"),
                "MANUAL PUTAR KIRI {:.1f}/{:.1f}s".format(
                    elapsed,
                    MANUAL_UTURN_SEC
                )
            )

        set_manual_command("STOP")
        return motor_stop(), "MANUAL UTURN SELESAI"

    if cmd == "UTURN_RIGHT":
        elapsed = now - started

        if elapsed < MANUAL_UTURN_SEC:
            return (
                motor_spin_turnback("RIGHT"),
                "MANUAL PUTAR KANAN {:.1f}/{:.1f}s".format(
                    elapsed,
                    MANUAL_UTURN_SEC
                )
            )

        set_manual_command("STOP")
        return motor_stop(), "MANUAL UTURN SELESAI"

    return motor_stop(), "MANUAL STOP"


def web_remote_state():
    with robot_lock:
        auto_on = robot_enabled

    with manual_lock:
        manual_on = manual_mode
        cmd = manual_cmd
        status = manual_last_status
        started = manual_uturn_started
        hold = manual_hold_mode

    elapsed = (
        time.monotonic() - started
        if cmd in ("UTURN_LEFT", "UTURN_RIGHT") and started > 0.0
        else 0.0
    )

    return {
        "build": BUILD_ID,
        "auto": auto_on,
        "manual": manual_on,
        "hold": hold,
        "cmd": cmd,
        "status": status,
        "uturn_sec": MANUAL_UTURN_SEC,
        "uturn_elapsed": elapsed,
        "bridge": "{}:{}".format(
            CONTROL_HP_IP,
            CONTROL_TCP_PORT
        ),
    }


def web_remote_html():
    return """<!doctype html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Robot Remote</title>
  <style>
    body { margin: 0; font-family: Arial, sans-serif; background: #111; color: #eee; }
    main { max-width: 560px; margin: 0 auto; padding: 16px; }
    h1 { font-size: 24px; margin: 8px 0 12px; }
    .panel { background: #202020; border: 1px solid #444; padding: 12px; margin-bottom: 12px; }
    .status { color: #65ff7a; font-size: 21px; font-weight: 700; }
    img { width: 100%; background: #000; display: block; margin-bottom: 12px; border: 1px solid #444; }
    button { width: 100%; padding: 17px; margin: 6px 0; border: 0; color: white; font-size: 20px; font-weight: 700; }
    .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; }
    .auto { background: #087d30; }
    .stop { background: #b00020; }
    .manual { background: #2456d8; }
    .turn { background: #8a36c9; }
    code { color: #ffd166; }
  </style>
</head>
<body>
  <main>
    <h1>ROBOT REMOTE</h1>
    <img src="/stream" alt="camera">
    <div class="panel">
      <div class="status" id="status">...</div>
      <div>AUTO: <code id="auto">...</code></div>
      <div>MANUAL: <code id="manual">...</code></div>
      <div>HOLD: <code id="hold">...</code></div>
      <div>CMD: <code id="cmd">...</code></div>
      <div>U-TURN: <code id="uturn">...</code></div>
      <div>Bridge: <code id="bridge">...</code></div>
    </div>
    <div class="grid">
      <button class="auto" onclick="cmd('/auto/start')">AUTO START</button>
      <button class="stop" onclick="cmd('/auto/stop')">AUTO STOP</button>
    </div>
    <div class="grid">
      <button class="manual hold" data-path="/manual/forward">MAJU LURUS</button>
      <button class="manual hold" data-path="/manual/backward">MUNDUR</button>
    </div>
    <div class="grid">
      <button class="manual hold" data-path="/manual/left">KIRI</button>
      <button class="manual hold" data-path="/manual/right">KANAN</button>
    </div>
    <div class="grid">
      <button class="turn" onclick="cmd('/manual/uturn_left')">PUTAR KIRI 3.35s</button>
      <button class="turn" onclick="cmd('/manual/uturn_right')">PUTAR KANAN 3.35s</button>
    </div>
    <button class="stop" onclick="cmd('/manual/stop')">STOP MANUAL</button>
  </main>
  <script>
    async function cmd(path) {
      await fetch(path, { method: 'POST' });
      await refresh();
    }
    async function refresh() {
      const r = await fetch('/state');
      const s = await r.json();
      document.getElementById('status').textContent = s.status;
      document.getElementById('auto').textContent = s.auto ? 'ON' : 'OFF';
      document.getElementById('manual').textContent = s.manual ? 'ON' : 'OFF';
      document.getElementById('hold').textContent = s.hold ? 'ON' : 'OFF';
      document.getElementById('cmd').textContent = s.cmd;
      document.getElementById('uturn').textContent = s.uturn_elapsed.toFixed(1) + '/' + s.uturn_sec.toFixed(2) + 's';
      document.getElementById('bridge').textContent = s.bridge;
    }
    function bindHoldButtons() {
      document.querySelectorAll('.hold').forEach((b) => {
        const start = async (e) => {
          e.preventDefault();
          await cmd(b.dataset.path);
        };
        const stop = async (e) => {
          e.preventDefault();
          await cmd('/manual/release');
        };
        b.addEventListener('touchstart', start, { passive: false });
        b.addEventListener('touchend', stop, { passive: false });
        b.addEventListener('touchcancel', stop, { passive: false });
        b.addEventListener('mousedown', start);
        b.addEventListener('mouseup', stop);
        b.addEventListener('mouseleave', stop);
      });
    }
    bindHoldButtons();
    setInterval(refresh, 250);
    refresh();
  </script>
</body>
</html>
"""


class WebRemoteHandler(BaseHTTPRequestHandler):
    def _send(self, code, body, content_type="text/plain"):
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "no-store")
        self.end_headers()

        if isinstance(body, str):
            body = body.encode("utf-8")

        self.wfile.write(body)

    def do_GET(self):
        path = urlparse(self.path).path

        if path == "/":
            self._send(200, web_remote_html(), "text/html")
            return

        if path == "/stream":
            self.send_response(200)
            self.send_header(
                "Content-Type",
                "multipart/x-mixed-replace; boundary=frame"
            )
            self.send_header("Cache-Control", "no-store")
            self.end_headers()

            while running:
                with web_frame_lock:
                    frame = (
                        None
                        if latest_web_frame is None
                        else latest_web_frame.copy()
                    )

                if frame is None:
                    frame = np.zeros(
                        (FRAME_H, FRAME_W, 3),
                        dtype=np.uint8
                    )
                    cv2.putText(
                        frame,
                        "WAIT CAMERA...",
                        (170, 240),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        (0, 255, 255),
                        2,
                        cv2.LINE_AA
                    )

                ok, jpg = cv2.imencode(
                    ".jpg",
                    frame,
                    [int(cv2.IMWRITE_JPEG_QUALITY), 70]
                )

                if not ok:
                    time.sleep(0.05)
                    continue

                try:
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n\r\n")
                    self.wfile.write(jpg.tobytes())
                    self.wfile.write(b"\r\n")
                except Exception:
                    break

                time.sleep(0.08)

            return

        if path == "/state":
            self._send(
                200,
                json.dumps(web_remote_state()),
                "application/json"
            )
            return

        self._send(404, "not found")

    def do_POST(self):
        path = urlparse(self.path).path

        if path == "/auto/start":
            set_auto_enabled(True)
            self._send(200, "ok")
            return

        if path == "/auto/stop":
            set_auto_enabled(False)
            self._send(200, "ok")
            return

        if path == "/manual/release":
            release_manual_hold()
            self._send(200, "ok")
            return

        manual_map = {
            "/manual/forward": "FORWARD",
            "/manual/backward": "BACKWARD",
            "/manual/left": "LEFT",
            "/manual/right": "RIGHT",
            "/manual/uturn_left": "UTURN_LEFT",
            "/manual/uturn_right": "UTURN_RIGHT",
            "/manual/stop": "STOP",
        }

        if path in manual_map:
            set_manual_command(manual_map[path])
            self._send(200, "ok")
            return

        self._send(404, "not found")

    def log_message(self, fmt, *args):
        return


def get_laptop_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "127.0.0.1"


def web_remote_loop():
    try:
        server = ThreadingHTTPServer(
            (WEB_REMOTE_HOST, WEB_REMOTE_PORT),
            WebRemoteHandler
        )
    except Exception as e:
        print(
            "[WEB] start failed: {}".format(e),
            flush=True
        )
        return

    print(
        "[WEB] Remote HP: http://{}:{}".format(
            get_laptop_ip(),
            WEB_REMOTE_PORT
        ),
        flush=True
    )

    while running:
        server.handle_request()


def manual_control_loop():
    while running:
        now = time.monotonic()
        override = get_manual_override(now)

        if override is not None:
            (rpm_kiri, rpm_kanan), _ = override
            control_send_rpm(
                rpm_kiri,
                rpm_kanan
            )

        time.sleep(SEND_INTERVAL)


def motor_keepalive_loop():
    while running:
        with _drive_cmd_lock:
            cmd = _drive_cmd

        if cmd is not None:
            control_send_rpm(
                cmd[0],
                cmd[1]
            )

        time.sleep(SEND_INTERVAL)



def update_uturn_state(no_safe_path):
    """
    Non-blocking 360-degree recovery state machine.

    Returns:
      active, rpm_left, rpm_right, status
    """
    global _uturn_state
    global _uturn_started
    global _uturn_direction
    global _uturn_last_direction

    now = time.monotonic()

    # Start a U-turn only when all safe directions are unavailable.
    if (
        _uturn_state == "IDLE" and
        no_safe_path
    ):
        _uturn_state = "STOPPING"
        _uturn_started = now

        # Alternate direction each recovery.
        _uturn_direction = (
            "LEFT"
            if _uturn_last_direction == "RIGHT"
            else "RIGHT"
        )

        _uturn_last_direction = _uturn_direction

    if _uturn_state == "IDLE":
        return False, MOTOR_NEUTRAL, MOTOR_NEUTRAL, "IDLE"

    if _uturn_state == "STOPPING":
        if (
            now - _uturn_started
            >= UTURN_STOP_BEFORE_SEC
        ):
            _uturn_state = "SPINNING"
            _uturn_started = now

        return (
            True,
            MOTOR_NEUTRAL,
            MOTOR_NEUTRAL,
            "U-TURN PRE-STOP"
        )

    if _uturn_state == "SPINNING":
        elapsed = (
            now - _uturn_started
        )

        if elapsed >= UTURN_SPIN_DURATION_SEC:
            _uturn_state = "SETTLING"
            _uturn_started = now

            return (
                True,
                MOTOR_NEUTRAL,
                MOTOR_NEUTRAL,
                "U-TURN COMPLETE"
            )

        rpm_left, rpm_right = motor_spin_turnback(
            _uturn_direction
        )

        return (
            True,
            rpm_left,
            rpm_right,
            "360 SPIN {} {:.1f}/{:.1f}s".format(
                _uturn_direction,
                elapsed,
                UTURN_SPIN_DURATION_SEC
            )
        )

    if _uturn_state == "SETTLING":
        if (
            now - _uturn_started
            >= UTURN_SETTLE_SEC
        ):
            _uturn_state = "IDLE"
            _uturn_started = now

            return (
                False,
                MOTOR_NEUTRAL,
                MOTOR_NEUTRAL,
                "U-TURN DONE"
            )

        return (
            True,
            MOTOR_NEUTRAL,
            MOTOR_NEUTRAL,
            "U-TURN SETTLING"
        )

    _uturn_state = "IDLE"

    return (
        False,
        MOTOR_NEUTRAL,
        MOTOR_NEUTRAL,
        "IDLE"
    )



def finite_min(*values):
    valid = [
        float(v)
        for v in values
        if np.isfinite(v)
    ]
    return (
        min(valid)
        if valid
        else float("inf")
    )


def cooperative_camera_lidar_fusion(
    road_steer,
    cam_guard_action,
    cam_guard_steer,
    cam_guard_level,
    cam_left_px,
    cam_center_px,
    cam_right_px
):
    """
    TRUE COOPERATIVE FUSION

    Camera and LiDAR are equal safety contributors.

    CAMERA:
      - HSV road gives base direction only
      - YOLO/body guard can independently force avoidance

    LIDAR:
      - safety boundary is LIDAR_RED_SAFETY_RADIUS_MM
      - can independently force avoidance

    Neither sensor is absolute master.
    """
    road_steer = float(
        np.clip(
            road_steer,
            -PATH_GENTLE_STEER_MAX,
            PATH_GENTLE_STEER_MAX
        )
    )

    # --------------------------------------------------------
    # CAMERA SAFETY VECTOR + SEVERITY
    # --------------------------------------------------------
    cam_safety_steer = 0.0
    cam_severity = 0.0

    if cam_guard_action == "STOP":
        cam_severity = 1.0

    elif cam_guard_action in (
        "GUARD_LEFT",
        "GUARD_RIGHT"
    ):
        cam_safety_steer = float(
            np.clip(
                cam_guard_steer,
                -FUSION_MAX_SAFETY_STEER,
                FUSION_MAX_SAFETY_STEER
            )
        )

        if cam_guard_level == "HARD":
            cam_severity = 0.85
        elif cam_guard_level == "WARN":
            cam_severity = 0.50
        else:
            cam_severity = 0.35

        # More overlap with body = more urgent.
        total_cam = (
            cam_left_px +
            cam_center_px +
            cam_right_px
        )

        cam_severity = float(
            np.clip(
                cam_severity +
                min(0.15, total_cam / 4000.0),
                0.0,
                1.0
            )
        )

    # --------------------------------------------------------
    # LIDAR SAFETY VECTOR + SEVERITY
    # --------------------------------------------------------
    left_d = finite_min(
        lidar_left,
        lidar_front_left
    )

    right_d = finite_min(
        lidar_right,
        lidar_front_right
    )

    center_d = float(
        lidar_front
    )

    left_close = (
        np.isfinite(left_d) and
        left_d <= LIDAR_FRONT_PREWARN_MM
    )

    right_close = (
        np.isfinite(right_d) and
        right_d <= LIDAR_FRONT_PREWARN_MM
    )

    center_close = (
        np.isfinite(center_d) and
        center_d <= LIDAR_FRONT_PREWARN_MM
    )
    left_red = (
        np.isfinite(left_d) and
        left_d <= LIDAR_RED_SAFETY_RADIUS_MM
    )
    right_red = (
        np.isfinite(right_d) and
        right_d <= LIDAR_RED_SAFETY_RADIUS_MM
    )
    center_red = (
        np.isfinite(center_d) and
        center_d <= LIDAR_RED_SAFETY_RADIUS_MM
    )

    lidar_safety_steer = 0.0
    lidar_severity = 0.0

    left_space = (
        left_d
        if np.isfinite(left_d)
        else 9999.0
    )

    right_space = (
        right_d
        if np.isfinite(right_d)
        else 9999.0
    )

    nearest_lidar = min(
        v for v in (
            left_d,
            center_d,
            right_d
        )
        if np.isfinite(v)
    ) if any(
        np.isfinite(v)
        for v in (
            left_d,
            center_d,
            right_d
        )
    ) else float("inf")

    def lidar_avoid_strength(nearest_d):
        if not np.isfinite(nearest_d):
            return LIDAR_PREWARN_STEER_MIN

        if nearest_d <= ROBOT_RADIUS_MM:
            return FUSION_MAX_SAFETY_STEER

        if nearest_d <= LIDAR_RED_SAFETY_RADIUS_MM:
            t = (
                LIDAR_RED_SAFETY_RADIUS_MM -
                nearest_d
            ) / max(
                1.0,
                LIDAR_RED_SAFETY_RADIUS_MM -
                ROBOT_RADIUS_MM
            )
            return float(
                np.clip(
                    FUSION_OVERRIDE_HARD +
                    t * (
                        FUSION_MAX_SAFETY_STEER -
                        FUSION_OVERRIDE_HARD
                    ),
                    FUSION_OVERRIDE_HARD,
                    FUSION_MAX_SAFETY_STEER
                )
            )

        t = (
            max(
                LIDAR_SIDE_PREWARN_MM,
                LIDAR_FRONT_PREWARN_MM
            ) -
            nearest_d
        ) / max(
            1.0,
            max(
                LIDAR_SIDE_PREWARN_MM,
                LIDAR_FRONT_PREWARN_MM
            ) -
            LIDAR_RED_SAFETY_RADIUS_MM
        )
        return float(
            np.clip(
                LIDAR_PREWARN_STEER_MIN +
                t * (
                    FUSION_OVERRIDE_HARD -
                    LIDAR_PREWARN_STEER_MIN
                ),
                LIDAR_PREWARN_STEER_MIN,
                FUSION_OVERRIDE_HARD
            )
        )

    def lidar_sparse_side():
        if (
            lidar_right_count > lidar_left_count or
            lidar_left_score + 0.75 < lidar_right_score
        ):
            return -1.0, "LEFT"

        if (
            lidar_left_count > lidar_right_count or
            lidar_right_score + 0.75 < lidar_left_score
        ):
            return 1.0, "RIGHT"

        if left_space > right_space + 60.0:
            return -1.0, "LEFT"

        if right_space > left_space + 60.0:
            return 1.0, "RIGHT"

        return 0.0, "CENTER"

    if right_close or left_close or center_close:
        magnitude = lidar_avoid_strength(nearest_lidar)
        lidar_severity = float(
            np.clip(
                magnitude / max(1.0, FUSION_MAX_SAFETY_STEER),
                0.35,
                1.0
            )
        )

        if right_close and not left_close and not center_red:
            lidar_safety_steer = -magnitude
        elif left_close and not right_close and not center_red:
            lidar_safety_steer = magnitude
        else:
            side_sign, sparse_side = lidar_sparse_side()

            if side_sign == 0.0:
                if right_close and not left_close:
                    side_sign = -1.0
                    sparse_side = "LEFT"
                elif left_close and not right_close:
                    side_sign = 1.0
                    sparse_side = "RIGHT"

            lidar_safety_steer = side_sign * magnitude

            if sparse_side == "CENTER" and center_close:
                lidar_severity = min(1.0, lidar_severity + 0.20)

    # --------------------------------------------------------
    # HARD STOP CONDITIONS
    # --------------------------------------------------------
    camera_hard_stop = (
        cam_guard_action == "STOP"
    )

    lidar_hard_stop = (
        nearest_lidar <= ROBOT_RADIUS_MM and
        center_red and
        left_red and
        right_red
    )

    if camera_hard_stop and lidar_hard_stop:
        return (
            "STOP",
            0.0,
            "CAMERA + LIDAR: ALL UNSAFE"
        )

    # --------------------------------------------------------
    # COMBINE SAFETY VECTORS EQUALLY
    # --------------------------------------------------------
    cam_vec = (
        cam_safety_steer *
        FUSION_CAMERA_SAFETY_WEIGHT
    )

    lidar_vec = (
        lidar_safety_steer *
        FUSION_LIDAR_SAFETY_WEIGHT
    )

    # Same direction -> reinforce each other.
    same_direction = (
        abs(cam_vec) > 0.1 and
        abs(lidar_vec) > 0.1 and
        np.sign(cam_vec) == np.sign(lidar_vec)
    )

    # Opposite directions -> compare severity, no sensor favoritism.
    conflicting = (
        abs(cam_vec) > 0.1 and
        abs(lidar_vec) > 0.1 and
        np.sign(cam_vec) != np.sign(lidar_vec)
    )

    if same_direction:
        safety_steer = float(
            np.clip(
                cam_vec + lidar_vec,
                -FUSION_MAX_SAFETY_STEER,
                FUSION_MAX_SAFETY_STEER
            )
        )

        reason = "CAMERA + LIDAR AGREE"

    elif conflicting:
        diff = (
            cam_severity -
            lidar_severity
        )

        if abs(diff) <= FUSION_CONFLICT_MARGIN:
            # Similar severity:
            # use the direction with lower opposite-side risk.
            if (
                lidar_right_count > lidar_left_count or
                lidar_left_score < lidar_right_score
            ):
                safety_steer = -max(
                    abs(cam_vec),
                    abs(lidar_vec)
                )
                reason = "CAM/LIDAR CONFLICT -> SAFER LEFT"

            elif (
                lidar_left_count > lidar_right_count or
                lidar_right_score < lidar_left_score
            ):
                safety_steer = +max(
                    abs(cam_vec),
                    abs(lidar_vec)
                )
                reason = "CAM/LIDAR CONFLICT -> SAFER RIGHT"

            else:
                safety_steer = lidar_vec
                reason = "CAM/LIDAR CONFLICT -> LIDAR FRONT"

        elif cam_severity > lidar_severity:
            safety_steer = cam_vec
            reason = "CAMERA HAZARD STRONGER"

        else:
            safety_steer = lidar_vec
            reason = "LIDAR HAZARD STRONGER"

    elif abs(cam_vec) > 0.1:
        safety_steer = cam_vec
        reason = "CAMERA SAFETY AVOID"

    elif abs(lidar_vec) > 0.1:
        safety_steer = lidar_vec
        reason = "LIDAR SAFETY AVOID"

    else:
        safety_steer = 0.0
        reason = "BOTH SENSORS SAFE"

    # --------------------------------------------------------
    # Camera road = base recommendation only.
    # Safety from either sensor always has authority.
    # --------------------------------------------------------
    if abs(safety_steer) > 0.1:
        final_steer = float(
            np.clip(
                road_steer * 0.35 +
                safety_steer,
                -70.0,
                70.0
            )
        )
    else:
        final_steer = road_steer

    if final_steer < -8.0:
        final_dir = "LEFT"
    elif final_steer > 8.0:
        final_dir = "RIGHT"
    else:
        final_dir = "CENTER"

    return (
        final_dir,
        final_steer,
        "{} | CAMsev:{:.2f} LIDsev:{:.2f}".format(
            reason,
            cam_severity,
            lidar_severity
        )
    )



def _body_side_x_at_y(p_top, p_bottom, y):
    """
    Return x coordinate of a body side line at a given image y.
    """
    x1, y1 = float(p_top[0]), float(p_top[1])
    x2, y2 = float(p_bottom[0]), float(p_bottom[1])

    dy = y2 - y1

    if abs(dy) < 1e-6:
        return x1

    t = (
        float(y) - y1
    ) / dy

    return (
        x1 +
        t * (x2 - x1)
    )


def true_dead_end_for_uturn(
    lidar_has_scan,
    cam_guard_action,
    line_action,
    final_direction
):
    """
    Auto-360 only for a real dead-end.

    Ordinary obstacle avoidance MUST NOT start a U-turn.

    A true trap requires:
      - LiDAR scan active
      - camera body guard asks STOP
      - LiDAR LEFT + CENTER + RIGHT are blocked
    """
    global _true_trap_since

    now = time.monotonic()

    lidar_all_blocked = (
        lidar_has_scan and
        (
            (
                lidar_hard_left and
                lidar_hard_center and
                lidar_hard_right
            )
            or
            (
                lidar_left_score >= LIDAR_BLOCK_RISK and
                lidar_center_score >= LIDAR_BLOCK_RISK and
                lidar_right_score >= LIDAR_BLOCK_RISK
            )
        )
    )

    camera_lidar_full_trap = (
        lidar_all_blocked and
        cam_guard_action == "STOP"
    )

    trapped = camera_lidar_full_trap

    if not trapped:
        _true_trap_since = 0.0
        return False

    if _true_trap_since <= 0.0:
        _true_trap_since = now
        return False

    return (
        now -
        _true_trap_since
        >= UTURN_TRUE_TRAP_CONFIRM_SEC
    )



def contour_x_at_y(
    contour,
    y_target,
    side_name
):
    """
    Estimate contour boundary x near a target y.

    LEFT  -> use right-most contour points (inner lane boundary)
    RIGHT -> use left-most contour points (inner lane boundary)
    """
    if contour is None:
        return None

    pts = contour.reshape(-1, 2)

    if len(pts) < CONTOUR_LANE_MIN_POINTS:
        return None

    # Use points near the requested row.
    tolerance = 24

    near = pts[
        np.abs(
            pts[:, 1] -
            int(y_target)
        ) <= tolerance
    ]

    if len(near) < 3:
        # fallback: nearest rows
        order = np.argsort(
            np.abs(
                pts[:, 1] -
                int(y_target)
            )
        )

        near = pts[
            order[
                :min(
                    12,
                    len(order)
                )
            ]
        ]

    if len(near) == 0:
        return None

    xs = near[:, 0].astype(
        np.float32
    )

    if side_name == "LEFT":
        # inner edge of left object/wall
        return float(
            np.percentile(
                xs,
                80
            )
        )

    # inner edge of right object/wall
    return float(
        np.percentile(
            xs,
            20
        )
    )


def lidar_side_confirmation():
    """
    Returns:
      left_confirmed, right_confirmed

    A side is confirmed when LiDAR sees something inside the
    active safety radius on that same side/front-diagonal.
    """
    left_d = finite_min(
        lidar_left,
        lidar_front_left
    )

    right_d = finite_min(
        lidar_right,
        lidar_front_right
    )

    left_ok = (
        np.isfinite(left_d) and
        left_d <= LIDAR_CONFIRM_RADIUS_MM
    )

    right_ok = (
        np.isfinite(right_d) and
        right_d <= LIDAR_CONFIRM_RADIUS_MM
    )

    return (
        bool(left_ok),
        bool(right_ok),
        float(left_d),
        float(right_d)
    )


def yolo_side_confirmation(
    obj_left_px,
    obj_center_px,
    obj_right_px
):
    """
    Side confirmation from YOLO/body overlap.
    """
    left_ok = (
        obj_left_px >= YOLO_CONFIRM_OVERLAP_PX or
        (
            obj_center_px >= YOLO_CONFIRM_OVERLAP_PX and
            obj_left_px > obj_right_px
        )
    )

    right_ok = (
        obj_right_px >= YOLO_CONFIRM_OVERLAP_PX or
        (
            obj_center_px >= YOLO_CONFIRM_OVERLAP_PX and
            obj_right_px > obj_left_px
        )
    )

    return bool(left_ok), bool(right_ok)


def detect_edge_road_only(frame):
    """
    Edge-only road boundary detector.

    Tidak memakai HSV floor, area warna jalan, marker,
    jalur melengkung, atau panah.
    """
    global _edge_left_state
    global _edge_right_state
    global _edge_width_state
    global _edge_road_center_state

    h, w = frame.shape[:2]
    y_top = int(h * EDGE_ONLY_ROI_TOP_RATIO)
    y_bottom = int(h * EDGE_ONLY_ROI_BOTTOM_RATIO)
    y_ref = int(h * 0.82)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(
        gray,
        EDGE_ONLY_CANNY_LOW,
        EDGE_ONLY_CANNY_HIGH
    )

    roi = np.zeros_like(edges)
    roi[y_top:y_bottom, :] = 255
    edges = cv2.bitwise_and(edges, roi)

    lines = cv2.HoughLinesP(
        edges,
        1,
        np.pi / 180.0,
        threshold=EDGE_ONLY_HOUGH_THRESHOLD,
        minLineLength=EDGE_ONLY_MIN_LINE_LENGTH,
        maxLineGap=EDGE_ONLY_MAX_LINE_GAP
    )

    left_candidates = []
    right_candidates = []
    image_center = w / 2.0

    if lines is not None:
        lines_arr = np.asarray(lines).reshape(-1, 4)

        for item in lines_arr:
            x1, y1, x2, y2 = [
                int(v)
                for v in item
            ]
            dy = y2 - y1
            dx = x2 - x1

            if abs(dy) < 24:
                continue

            y_min = min(y1, y2)
            y_max = max(y1, y2)

            if y_max < h * EDGE_ONLY_MIN_BOTTOM_Y_RATIO:
                continue

            if y1 <= y2:
                x_top = x1
                y_top_line = y1
                x_bottom = x2
                y_bottom_line = y2
            else:
                x_top = x2
                y_top_line = y2
                x_bottom = x1
                y_bottom_line = y1

            dy_down = y_bottom_line - y_top_line
            dx_down = x_bottom - x_top

            if dy_down < 24:
                continue

            slope = dx_down / float(dy_down)
            abs_slope = abs(slope)

            if (
                abs(dx_down) < EDGE_ONLY_MIN_DX_DOWN_PX or
                abs_slope < EDGE_ONLY_MIN_ABS_SLOPE or
                abs_slope > EDGE_ONLY_MAX_ABS_SLOPE
            ):
                continue

            t = (y_ref - y1) / float(dy)
            x_ref = x1 + t * dx

            if x_ref < 0 or x_ref >= w:
                continue

            length = math.hypot(dx, dy)
            score = (y_max - y_min) + length * 0.35 + y_max * 0.20
            line = (float(x1), float(y1), float(x2), float(y2), float(x_ref))

            if (
                x_ref < image_center - EDGE_ONLY_SIDE_MARGIN_PX and
                dx_down < 0
            ):
                left_candidates.append((score, line))
            elif (
                x_ref > image_center + EDGE_ONLY_SIDE_MARGIN_PX and
                dx_down > 0
            ):
                right_candidates.append((score, line))

    def pick(candidates):
        if not candidates:
            return None
        candidates.sort(key=lambda v: v[0], reverse=True)
        return candidates[0][1]

    left = pick(left_candidates)
    right = pick(right_candidates)
    left_estimated = False
    right_estimated = False

    if left is not None and right is not None:
        width_now = right[4] - left[4]
        if width_now >= EDGE_ONLY_MIN_WIDTH_PX:
            _edge_width_state = (
                EDGE_ONLY_WIDTH_SMOOTH * _edge_width_state +
                (1.0 - EDGE_ONLY_WIDTH_SMOOTH) * width_now
            )
        else:
            left = None
            right = None

    if left is None and right is not None:
        lx = right[4] - _edge_width_state
        left = (
            lx,
            float(y_bottom),
            lx,
            float(y_top),
            lx
        )
        left_estimated = True

    if right is None and left is not None:
        rx = left[4] + _edge_width_state
        right = (
            rx,
            float(y_bottom),
            rx,
            float(y_top),
            rx
        )
        right_estimated = True

    valid = left is not None and right is not None

    if valid:
        if _edge_left_state is None:
            _edge_left_state = np.array(left, dtype=np.float32)
        else:
            _edge_left_state = (
                EDGE_ONLY_SMOOTH * _edge_left_state +
                (1.0 - EDGE_ONLY_SMOOTH) * np.array(left, dtype=np.float32)
            )

        if _edge_right_state is None:
            _edge_right_state = np.array(right, dtype=np.float32)
        else:
            _edge_right_state = (
                EDGE_ONLY_SMOOTH * _edge_right_state +
                (1.0 - EDGE_ONLY_SMOOTH) * np.array(right, dtype=np.float32)
            )

        left = tuple(float(v) for v in _edge_left_state)
        right = tuple(float(v) for v in _edge_right_state)

        if left_estimated and not right_estimated:
            desired_right_x = w * 0.86
            right_too_close = max(
                0.0,
                desired_right_x - right[4]
            )
            target_x = float(
                np.clip(
                    image_center - right_too_close,
                    0.0,
                    w - 1.0
                )
            )
            steering = float(
                np.clip(
                    -max(
                        EDGE_ONLY_ONE_SIDE_MIN_STEER,
                        (right_too_close / image_center) *
                        EDGE_ONLY_STEER_GAIN *
                        1.35
                    ),
                    -EDGE_ONLY_MAX_STEER,
                    0.0
                )
            )
        elif right_estimated and not left_estimated:
            desired_left_x = w * 0.14
            left_too_close = max(
                0.0,
                left[4] - desired_left_x
            )
            target_x = float(
                np.clip(
                    image_center + left_too_close,
                    0.0,
                    w - 1.0
                )
            )
            steering = float(
                np.clip(
                    max(
                        EDGE_ONLY_ONE_SIDE_MIN_STEER,
                        (left_too_close / image_center) *
                        EDGE_ONLY_STEER_GAIN *
                        1.35
                    ),
                    0.0,
                    EDGE_ONLY_MAX_STEER
                )
            )
        else:
            midpoint = (left[4] + right[4]) / 2.0
            _edge_road_center_state = (
                EDGE_ONLY_SMOOTH * _edge_road_center_state +
                (1.0 - EDGE_ONLY_SMOOTH) * midpoint
            )
            target_x = float(_edge_road_center_state)
            error_px = target_x - image_center

            if abs(error_px) < EDGE_ONLY_CENTER_DEADBAND_PX:
                steering = 0.0
            else:
                steering = float(
                    np.clip(
                        (error_px / image_center) * EDGE_ONLY_STEER_GAIN,
                        -EDGE_ONLY_MAX_STEER,
                        EDGE_ONLY_MAX_STEER
                    )
                )

        confidence = 1.0
        near_width = float(max(0.0, right[4] - left[4]))
    else:
        target_x = float(_edge_road_center_state)
        steering = 0.0
        confidence = 0.0
        near_width = 0.0

    return {
        "valid": bool(valid),
        "left": left,
        "right": right,
        "target_x": target_x,
        "target_y": y_ref,
        "steering": steering,
        "confidence": confidence,
        "near_width": near_width,
        "left_estimated": bool(left_estimated),
        "right_estimated": bool(right_estimated),
        "edges": edges,
    }


def draw_edge_road_only(frame, edge_road):
    if not edge_road.get("valid", False):
        return frame

    out = frame
    h, w = out.shape[:2]
    left = edge_road.get("left")
    right = edge_road.get("right")
    left_estimated = edge_road.get("left_estimated", False)
    right_estimated = edge_road.get("right_estimated", False)

    def draw_line(line, color):
        x1, y1, x2, y2, _ = line
        cv2.line(
            out,
            (int(x1), int(y1)),
            (int(x2), int(y2)),
            color,
            2,
            cv2.LINE_AA
        )

    if left is not None and not left_estimated:
        draw_line(left, (0, 255, 255))

    if right is not None and not right_estimated:
        draw_line(right, (0, 255, 255))

    if (
        left is not None and
        right is not None and
        not left_estimated and
        not right_estimated
    ):
        lx = int(left[4])
        rx = int(right[4])
        cx = int((lx + rx) / 2)
        y1 = int(h * 0.46)
        y2 = int(h * 0.90)
        cv2.line(
            out,
            (cx, y1),
            (cx, y2),
            (200, 200, 200),
            1,
            cv2.LINE_AA
        )

    return out


# ============================================================
# MAIN
# ============================================================

def main():
    print("[BUILD]", BUILD_ID, flush=True)
    global processing_fps, processing_ms, frame_age_ms
    global yolo_frame_counter, yolo_input_frame
    global latest_web_frame
    global _camera_only_steer_state
    global _camera_only_last_valid_ts

    signal.signal(signal.SIGINT, stop_all)
    signal.signal(signal.SIGTERM, stop_all)

    print("")
    print("==========================================")
    print(" WEB + LOCAL CONTROL")
    print("==========================================")
    print("WEB   : AUTO START/STOP + MANUAL")
    print("SPACE : START / STOP cadangan")
    print("Q/ESC : EXIT")
    print("==========================================")
    print("")

    # Serial STM32 dihubungkan otomatis langsung dari program ini.
    # Jangan blok startup kamera/display karena port kontrol HP.
    print("")
    print("==========================================")
    print(" CONTROL")
    print("==========================================")
    print("WEB   = START/STOP + MANUAL")
    print("SPACE = START / STOP toggle cadangan")
    print("Q / ESC = keluar")
    print("==========================================")
    print("")

    print("[WEB] Kontrol robot dari HP: AUTO + MANUAL", flush=True)
    print("[LOCAL] SPACE tetap bisa START/STOP cadangan", flush=True)
    print("[CTRL] Mode: TCP HP bridge {}:{}".format(CONTROL_HP_IP, CONTROL_TCP_PORT))

    threading.Thread(
        target=camera_loop,
        daemon=True
    ).start()

    threading.Thread(
        target=control_reconnect_loop,
        daemon=True
    ).start()

    threading.Thread(
        target=web_remote_loop,
        daemon=True
    ).start()

    threading.Thread(
        target=ros_bridge_loop,
        daemon=True
    ).start()

    threading.Thread(
        target=manual_control_loop,
        daemon=True
    ).start()

    threading.Thread(
        target=motor_keepalive_loop,
        daemon=True
    ).start()

    threading.Thread(
        target=yolo_worker_loop,
        daemon=True
    ).start()

    threading.Thread(
        target=lidar_navigation_loop,
        daemon=True
    ).start()

    print("")
    print("==========================================")
    print(" FREEDOM - LIDAR PRIMARY NAVIGATION + CAMERA BACKUP")
    print("==========================================")
    print("Camera :", MJPEG_URL)
    print("YOLO   : semantic obstacle pada zona merah")
    print("LiDAR  : ROS /scan_front + body-clearance guard")
    print("Orient : LiDAR FRONT/BACK FLIP AKTIF")
    print("Filter : side body ignored, front/diagonal remains active")
    print("Mode   : LiDAR-only / Camera-only / Fusion dua sensor")
    print("Camera : edge boundary navigation + YOLO")
    print("Motor  : Laptop -> TCP HP:8888 -> USB OTG -> STM32")
    print("Road   : edge left/right boundary only")
    print("Control: WEB HP = AUTO/MANUAL | SPACE cadangan")
    print("Window : 1 window utama saja")
    print("ESC/Q  : keluar")
    print("")

    # ========================================================
    # WINDOW 1: hasil kamera + body + rekomendasi
    # WINDOW 2: garis pembatas
    # Window processing pada laptop.
    # ========================================================
    screen_w, screen_h = get_screen_size()

    # Satu window saja. Posisi kiri-tengah monitor.
    main_x = 0
    main_y = max(0, int((screen_h - MAIN_WIN_H) / 2))

    print(
        "[DISPLAY] Screen {}x{} | MAIN=({}, {})"
        .format(
            screen_w,
            screen_h,
            main_x,
            main_y
        )
    )

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(
        WINDOW_NAME,
        MAIN_WIN_W,
        MAIN_WIN_H
    )
    cv2.moveWindow(
        WINDOW_NAME,
        main_x,
        main_y
    )

    cv2.namedWindow(
        LIDAR_WINDOW_NAME,
        cv2.WINDOW_NORMAL
    )

    cv2.resizeWindow(
        LIDAR_WINDOW_NAME,
        720,
        720
    )

    cv2.moveWindow(
        LIDAR_WINDOW_NAME,
        760,
        max(0, main_y - 120)
    )


    # Body persis dari koordinat Desmos pada frame 640x480.
    body_points = build_body_points(FRAME_W, FRAME_H)

    camera_guard_masks = build_camera_body_guard_masks(
        body_points
    )

    last_id = -1
    count = 0
    fps_t0 = time.monotonic()

    obstacle_state = "FOLLOW"
    obstacle_state_started = 0.0
    last_turn_finished = -999.0

    # LiDAR primary U-turn state.
    lidar_uturn_state = "NONE"   # NONE / STOPPING / TURNING
    lidar_uturn_started = 0.0
    lidar_uturn_direction = "LEFT"
    LIDAR_UTURN_STOP_SEC = 0.35
    LIDAR_UTURN_TURN_SEC = 3.35

    # Escape state for true LiDAR trap.
    lidar_escape_state = "NONE"   # NONE / STOPPING / REVERSING
    lidar_escape_started = 0.0
    lidar_escape_direction = "LEFT"
    LIDAR_ESCAPE_STOP_SEC = 0.30

    while running:
        with camera_lock:
            fid = latest_camera_id
            src = latest_camera
            src_ts = latest_camera_ts

        if src is None or fid == last_id:
            now_control = time.monotonic()
            lidar_now_action = lidar_action
            lidar_now_reason = lidar_reason
            lidar_now_steer = lidar_steer_hint

            with lidar_scan_lock:
                lidar_points_for_path = list(lidar_latest_scan)

            lidar_has_scan = (
                lidar_now_action != "TIMEOUT" and
                len(lidar_points_for_path) >= 20
            )

            with robot_lock:
                enabled_now = robot_enabled

            manual_override = get_manual_override(now_control)

            if manual_override is not None:
                (rpm_kiri, rpm_kanan), status_text = manual_override
                control_send_rpm(
                    rpm_kiri,
                    rpm_kanan
                )
            elif not enabled_now:
                rpm_kiri, rpm_kanan = motor_stop()
                status_text = "AUTO OFF"
                control_send_rpm(
                    rpm_kiri,
                    rpm_kanan
                )
            elif lidar_has_scan:
                if lidar_now_action in ("AVOID_LEFT", "AVOID_RIGHT"):
                    rpm_kiri, rpm_kanan = steering_to_rpm(
                        lidar_now_steer
                    )
                    status_text = "CAM LOST -> LIDAR {}".format(
                        lidar_now_action
                    )
                    control_send_rpm(
                        rpm_kiri,
                        rpm_kanan
                    )
                elif lidar_now_action == "STOP":
                    rpm_kiri, rpm_kanan = motor_stop()
                    status_text = "CAM LOST -> LIDAR STOP"
                    control_send_rpm(
                        rpm_kiri,
                        rpm_kanan,
                        force=True
                    )
                else:
                    rpm_kiri, rpm_kanan = steering_to_rpm(0.0)
                    status_text = "CAM LOST -> LIDAR CLEAR"
                    control_send_rpm(
                        rpm_kiri,
                        rpm_kanan
                    )
            else:
                rpm_kiri, rpm_kanan = motor_stop()
                status_text = "CAM + LIDAR LOST"
                control_send_rpm(
                    rpm_kiri,
                    rpm_kanan,
                    force=True
                )

            # LiDAR map tetap hidup walaupun kamera belum connect.
            render_lidar_map()

            if src is None:
                blank = np.zeros(
                    (FRAME_H, FRAME_W, 3),
                    dtype=np.uint8
                )

                cv2.putText(
                    blank,
                    "CAMERA CONNECTING...",
                    (140, 220),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 255),
                    2,
                    cv2.LINE_AA
                )

                cv2.putText(
                    blank,
                    MJPEG_URL,
                    (100, 255),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.48,
                    (255, 255, 255),
                    1,
                        cv2.LINE_AA
                    )

                cv2.putText(
                    blank,
                    status_text,
                    (110, 300),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.58,
                    (0, 255, 0)
                    if lidar_has_scan
                    else (0, 0, 255),
                    2,
                    cv2.LINE_AA
                )

                cv2.putText(
                    blank,
                    "LIDAR {} | {}".format(
                        lidar_now_action,
                        lidar_now_reason[:38]
                    ),
                    (70, 330),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.43,
                    (0, 255, 255),
                    1,
                    cv2.LINE_AA
                )

                cv2.imshow(
                    WINDOW_NAME,
                    blank
                )

            key = cv2.waitKey(1) & 0xFF

            if key == 32:  # SPACE
                toggle_robot()

            elif key == 27 or key == ord("q"):
                break

            time.sleep(0.001)
            continue

        frame = src.copy()
        last_id = fid

        total_t0 = time.perf_counter()

        frame_hasil = frame.copy()

        # ====================================================
        # YOLO ASYNC - PERSON DI DEPAN
        # ====================================================
        yolo_frame_counter += 1

        if yolo_frame_counter % YOLO_EVERY_N_FRAMES == 0:
            with yolo_input_lock:
                yolo_input_frame = frame.copy()

        with yolo_result_lock:
            yolo_result_now = yolo_last_result
            yolo_infer_ms_now = yolo_last_infer_ms

        front_zone_masks = build_red_zone_masks(
            FRAME_W,
            FRAME_H,
            body_points
        )

        edge_road = detect_edge_road_only(frame)

        (
            obstacle_action,
            obstacle_label,
            obstacle_left_px,
            obstacle_center_px,
            obstacle_right_px
        ) = yolo_segmentation_obstacle(
            yolo_result_now,
            frame_hasil,
            front_zone_masks
        )

        (
            cam_guard_action,
            cam_guard_steer,
            cam_guard_level,
            cam_guard_label,
            cam_guard_left_px,
            cam_guard_center_px,
            cam_guard_right_px
        ) = camera_body_guard(
            yolo_result_now,
            frame_hasil,
            camera_guard_masks,
            float(edge_road["steering"])
        )

        frame_hasil = draw_edge_road_only(
            frame_hasil,
            edge_road
        )

        path_info = {
            "valid": bool(edge_road["valid"]),
            "confidence": float(edge_road["confidence"]),
            "steering": float(edge_road["steering"]),
            "target_x": float(edge_road["target_x"]),
            "near_x": float(edge_road["target_x"]),
            "poly": None,
            "path_points": [],
            "left_points": [],
            "right_points": [],
            "road_mask": None,
            "near_width": float(edge_road["near_width"]),
        }

        ros_follow = ros_path_follow_info()

        if ros_follow["valid"]:
            path_info["valid"] = True
            path_info["confidence"] = float(ros_follow["confidence"])
            path_info["steering"] = float(ros_follow["steering"])
            path_info["target_x"] = float(ros_follow["target_x"])
            path_info["target_y"] = float(ros_follow["target_y"])
            path_info["near_x"] = float(ros_follow["target_x"])
            path_info["near_width"] = 0.0
            path_info["path_points"] = []

        line_action = "NONE"
        line_severity = 0.0
        line_left_clearance = None
        line_right_clearance = None
        contour_lane_valid = False
        contour_lane_steer = 0.0
        predictive_body_steer = 0.0
        hsv_road_steer = 0.0
        lane_fusion_reason = "EDGE ONLY"
        far_lane_valid = False
        far_lane_steer = 0.0
        far_lane_target_x = edge_road["target_x"]
        far_lane_target_y = edge_road["target_y"]
        far_lane_reason = "EDGE ONLY"

        # Cross-sensor confirmation for side hazards.
        (
            lidar_confirm_left,
            lidar_confirm_right,
            lidar_confirm_left_mm,
            lidar_confirm_right_mm
        ) = lidar_side_confirmation()

        (
            yolo_confirm_left,
            yolo_confirm_right
        ) = yolo_side_confirmation(
            cam_guard_left_px,
            cam_guard_center_px,
            cam_guard_right_px
        )

        # Visual guard envelope around physical body.
        cv2.polylines(
            frame_hasil,
            [
                camera_guard_masks["warn_poly"]
            ],
            True,
            (255, 0, 255),
            1,
            cv2.LINE_AA
        )

        cv2.polylines(
            frame_hasil,
            [
                camera_guard_masks["hard_poly"]
            ],
            True,
            (0, 165, 255),
            1,
            cv2.LINE_AA
        )


        # BODY 4 WARNA PERSIS REFERENSI.
        draw_robot_body_4colors(
            frame_hasil,
            body_points,
            alpha=0.33
        )

        steering = float(path_info["steering"])

        if ros_follow["valid"]:
            status_jalan = "ROS GREEN PATH"
            perintah_robot = ros_follow["reason"]
            warna_status = (0, 255, 0)
        elif path_info["valid"]:
            status_jalan = "EDGE ROAD"
            perintah_robot = "MIDPOINT EDGE"
            warna_status = (0, 255, 0)
        else:
            status_jalan = "EDGE ROAD LOST"
            perintah_robot = "WAIT EDGE"
            warna_status = (0, 165, 255)

        # ====================================================
        # EKSEKUSI KE STM32 + OBSTACLE 3-ZONE
        # ====================================================
        now_control = time.monotonic()

        rpm_kiri, rpm_kanan = steering_to_rpm(steering)

        # ====================================================
        # SENSOR FUSION - LIDAR PRIMARY
        #
        # LiDAR sehat:
        #   navigator utama = LiDAR
        #
        # LiDAR timeout:
        #   backup = kamera edge boundary + YOLO
        # ====================================================
        lidar_now_action = lidar_action
        lidar_now_reason = lidar_reason
        lidar_now_steer = lidar_steer_hint

        with lidar_scan_lock:
            lidar_points_for_path = list(lidar_latest_scan)

        camera_backup_active = (
            lidar_now_action == "TIMEOUT" and
            path_info["valid"]
        )

        # ====================================================
        # COOPERATIVE CAMERA + LIDAR FUSION
        #
        # Camera road = edge midpoint only
        # Camera YOLO/body = independent safety
        # LiDAR = independent safety
        # ====================================================

        lidar_has_scan = (
            lidar_now_action != "TIMEOUT" and
            len(lidar_points_for_path) >= 20
        )

        road_steer_now = float(path_info["steering"])
        lane_fusion_reason = "EDGE ONLY"

        unified_reason = "EDGE ONLY"

        if lidar_has_scan:
            (
                final_direction,
                steering,
                fusion_reason
            ) = cooperative_camera_lidar_fusion(
                road_steer_now,
                cam_guard_action,
                cam_guard_steer,
                cam_guard_level,
                cam_guard_left_px,
                cam_guard_center_px,
                cam_guard_right_px
            )
            fusion_reason = "EDGE ONLY | " + fusion_reason

        elif path_info["valid"]:
            # LiDAR unavailable:
            # Camera still keeps visual safety active.
            if cam_guard_action == "STOP":
                final_direction = "STOP"
                steering = 0.0
                fusion_reason = "LIDAR LOST -> CAMERA BODY STOP"
            else:
                steering = float(
                    np.clip(
                        road_steer_now +
                        cam_guard_steer,
                        -55.0,
                        55.0
                    )
                )

                if steering < -8.0:
                    final_direction = "LEFT"
                elif steering > 8.0:
                    final_direction = "RIGHT"
                else:
                    final_direction = "CENTER"

                fusion_reason = "LIDAR LOST -> CAMERA ACTIVE"

        else:
            final_direction = "STOP"
            steering = 0.0
            fusion_reason = "NO VALID CAMERA OR LIDAR"

        if final_direction == "STOP":
            rpm_kiri, rpm_kanan = motor_stop()
        else:
            rpm_kiri, rpm_kanan = steering_to_rpm(
                steering
            )

        # ====================================================
        # APPLY THIRD SAFETY AFTER CAMERA+LIDAR COOPERATIVE FUSION
        # ====================================================
        # Contour safety already fused above.

        if final_direction == "STOP":
            rpm_kiri, rpm_kanan = motor_stop()
        else:
            rpm_kiri, rpm_kanan = steering_to_rpm(
                steering
            )

        # ====================================================
        # AUTO 360 RECOVERY
        # If LiDAR reports no safe path, stop then spin ~2 sec.
        # Sensors continue updating during the maneuver.
        # ====================================================
        no_safe_path_for_uturn = true_dead_end_for_uturn(
            lidar_has_scan,
            cam_guard_action,
            line_action,
            final_direction
        )

        (
            uturn_active,
            uturn_rpm_kiri,
            uturn_rpm_kanan,
            uturn_status
        ) = update_uturn_state(
            no_safe_path_for_uturn
        )

        if uturn_active:
            final_direction = "UTURN"
            steering = 0.0
            rpm_kiri = uturn_rpm_kiri
            rpm_kanan = uturn_rpm_kanan
            fusion_reason = uturn_status

        with robot_lock:
            enabled_now = robot_enabled

        # ====================================================
        # STATUS VISUAL YOLO
        # Ditentukan SEBELUM eksekusi motor supaya tulisan atas
        # tidak lagi tetap menampilkan KOREKSI KIRI/KANAN ketika
        # YOLO sebenarnya sudah mengambil keputusan.
        # ====================================================
        if obstacle_action == "TURN_BACK":
            status_jalan = "OBSTACLE TENGAH: {}".format(
                obstacle_label.upper()
            )
            perintah_robot = "PUTAR BALIK"
            warna_status = (0, 0, 255)

        elif obstacle_action == "AVOID_LEFT":
            status_jalan = "OBSTACLE KANAN: {}".format(
                obstacle_label.upper()
            )
            perintah_robot = "HINDAR KE KIRI"
            warna_status = (0, 255, 255)

        elif obstacle_action == "AVOID_RIGHT":
            status_jalan = "OBSTACLE KIRI: {}".format(
                obstacle_label.upper()
            )
            perintah_robot = "HINDAR KE KANAN"
            warna_status = (0, 255, 255)

        # Trigger putar balik HANYA jika obstacle benar-benar berada di tengah.
        if (
            enabled_now and
            obstacle_state == "FOLLOW" and
            obstacle_action == "TURN_BACK" and
            (now_control - last_turn_finished) >= TURN_COOLDOWN_SECONDS
        ):
            obstacle_state = "STOPPING"
            obstacle_state_started = now_control

            print(
                "[YOLO] CENTER BLOCK: {} | L:{} C:{} R:{} -> STOP"
                .format(
                    obstacle_label,
                    obstacle_left_px,
                    obstacle_center_px,
                    obstacle_right_px
                )
            )

        manual_override = get_manual_override(now_control)

        if manual_override is not None:
            (rpm_kiri, rpm_kanan), manual_status = manual_override
            lidar_uturn_state = "NONE"
            lidar_escape_state = "NONE"
            obstacle_state = "FOLLOW"

            status_jalan = "WEB MANUAL"
            perintah_robot = manual_status
            warna_status = (0, 255, 255)

            control_send_rpm(
                rpm_kiri,
                rpm_kanan
            )

        elif not enabled_now:
            lidar_uturn_state = "NONE"
            lidar_escape_state = "NONE"
            obstacle_state = "FOLLOW"

            rpm_kiri, rpm_kanan = motor_stop()
            status_jalan = "LOCAL OFF"
            perintah_robot = "STOP"
            warna_status = (0, 0, 255)

            control_send_rpm(
                rpm_kiri,
                rpm_kanan
            )

        # ====================================================
        # LIDAR PRIMARY
        # ====================================================
        elif lidar_uturn_state == "STOPPING":
            rpm_kiri, rpm_kanan = motor_stop()

            status_jalan = "LIDAR DEAD END"
            perintah_robot = "STOP SEBELUM PUTAR BALIK"
            warna_status = (0, 0, 255)

            control_send_rpm(
                rpm_kiri,
                rpm_kanan,
                force=True
            )

            if (
                now_control -
                lidar_uturn_started
            ) >= LIDAR_UTURN_STOP_SEC:
                lidar_uturn_state = "TURNING"
                lidar_uturn_started = now_control

        elif lidar_uturn_state == "TURNING":
            elapsed = (
                now_control -
                lidar_uturn_started
            )

            if elapsed < LIDAR_UTURN_TURN_SEC:
                rpm_kiri, rpm_kanan = motor_spin_turnback(
                    lidar_uturn_direction
                )

                status_jalan = "LIDAR DEAD END"
                perintah_robot = "PUTAR BALIK {}".format(
                    lidar_uturn_direction
                )
                warna_status = (0, 165, 255)

                control_send_rpm(
                    rpm_kiri,
                    rpm_kanan,
                    force=True
                )
            else:
                rpm_kiri, rpm_kanan = motor_stop()

                control_send_rpm(
                    rpm_kiri,
                    rpm_kanan,
                    force=True
                )

                lidar_uturn_state = "NONE"

                status_jalan = "PUTAR BALIK SELESAI"
                perintah_robot = "SCAN JALAN BARU"
                warna_status = (0, 255, 0)

        elif lidar_now_action == "UTURN":
            lidar_uturn_state = "STOPPING"
            lidar_uturn_started = now_control
            lidar_uturn_direction = lidar_turn_dir

            rpm_kiri, rpm_kanan = motor_stop()

            status_jalan = "LIDAR DEAD END"
            perintah_robot = "SIAP PUTAR BALIK {}".format(
                lidar_uturn_direction
            )
            warna_status = (0, 0, 255)

            control_send_rpm(
                rpm_kiri,
                rpm_kanan,
                force=True
            )

        elif lidar_now_action == "TIMEOUT":
            if path_info["valid"]:
                _camera_only_last_valid_ts = now_control

                if cam_guard_action == "STOP":
                    if cam_guard_right_px > cam_guard_left_px:
                        steering = -FUSION_OVERRIDE_HARD
                        final_direction = "LEFT"
                    elif cam_guard_left_px > cam_guard_right_px:
                        steering = FUSION_OVERRIDE_HARD
                        final_direction = "RIGHT"
                    else:
                        steering = float(
                            np.clip(
                                road_steer_now,
                                -FUSION_OVERRIDE_HARD,
                                FUSION_OVERRIDE_HARD
                            )
                        )

                    if abs(steering) < 8.0:
                        steering = (
                            -FUSION_OVERRIDE_HARD
                            if path_info["target_x"] <= FRAME_W / 2
                            else FUSION_OVERRIDE_HARD
                        )

                    status_jalan = "CAMERA ONLY BODY"
                    perintah_robot = "HINDAR {}".format(
                        "KIRI"
                        if steering < 0
                        else "KANAN"
                    )
                    warna_status = (0, 255, 255)

                elif obstacle_action == "TURN_BACK":
                    if obstacle_right_px > obstacle_left_px:
                        steering = -FUSION_OVERRIDE_HARD
                    elif obstacle_left_px > obstacle_right_px:
                        steering = FUSION_OVERRIDE_HARD
                    else:
                        steering = float(
                            np.clip(
                                road_steer_now,
                                -FUSION_OVERRIDE_HARD,
                                FUSION_OVERRIDE_HARD
                            )
                        )

                    if abs(steering) < 8.0:
                        steering = (
                            -FUSION_OVERRIDE_HARD
                            if path_info["target_x"] <= FRAME_W / 2
                            else FUSION_OVERRIDE_HARD
                        )

                    status_jalan = "CAMERA ONLY YOLO"
                    perintah_robot = "HINDAR {}".format(
                        "KIRI"
                        if steering < 0
                        else "KANAN"
                    )
                    warna_status = (0, 255, 255)

                elif obstacle_action == "AVOID_RIGHT":
                    steering = FUSION_OVERRIDE_HARD
                    status_jalan = "CAMERA ONLY YOLO"
                    perintah_robot = "HINDAR KANAN"
                    warna_status = (0, 255, 255)

                elif obstacle_action == "AVOID_LEFT":
                    steering = -FUSION_OVERRIDE_HARD
                    status_jalan = "CAMERA ONLY YOLO"
                    perintah_robot = "HINDAR KIRI"
                    warna_status = (0, 255, 255)

                else:
                    steering = float(
                        np.clip(
                            road_steer_now + cam_guard_steer * 0.55,
                            -EDGE_ONLY_MAX_STEER,
                            EDGE_ONLY_MAX_STEER
                        )
                    )
                    status_jalan = "CAMERA ONLY"
                    perintah_robot = "EDGE BACKUP JALAN"
                    warna_status = (255, 255, 0)

                _camera_only_steer_state = (
                    CAMERA_ONLY_STEER_ALPHA *
                    _camera_only_steer_state +
                    (1.0 - CAMERA_ONLY_STEER_ALPHA) *
                    float(steering)
                )
                steering = float(_camera_only_steer_state)

                rpm_kiri, rpm_kanan = steering_to_rpm(
                    steering
                )

                control_send_rpm(
                    rpm_kiri,
                    rpm_kanan
                )

            elif (
                now_control - _camera_only_last_valid_ts
                <= CAMERA_ONLY_EDGE_LOST_HOLD_SEC
            ):
                steering = float(_camera_only_steer_state)
                rpm_kiri, rpm_kanan = steering_to_rpm(
                    steering
                )
                status_jalan = "CAMERA ONLY HOLD"
                perintah_robot = "EDGE KEDIP - LANJUT"
                warna_status = (0, 165, 255)

                control_send_rpm(
                    rpm_kiri,
                    rpm_kanan
                )

            else:
                steering = float(
                    np.clip(
                        _camera_only_steer_state,
                        -EDGE_ONLY_MAX_STEER,
                        EDGE_ONLY_MAX_STEER
                    )
                )

                rpm_kiri, rpm_kanan = steering_to_rpm(
                    steering
                )

                status_jalan = "CAMERA ONLY SEARCH"
                perintah_robot = "JALAN TERUS - CARI EDGE"
                warna_status = (0, 165, 255)

                control_send_rpm(
                    rpm_kiri,
                    rpm_kanan
                )

        # ====================================================
        # CAMERA YOLO OBSTACLE - saling melengkapi LiDAR.
        # Aktif kalau segmentation benar-benar menyentuh zona merah.
        # ====================================================
        elif cam_guard_action == "STOP" and final_direction != "UTURN":
            if lidar_has_scan and not (
                lidar_now_action == "STOP"
            ):
                if final_direction == "CENTER":
                    if (
                        lidar_right_count > lidar_left_count or
                        lidar_left_score < lidar_right_score
                    ):
                        steering = -FUSION_OVERRIDE_HARD
                        final_direction = "LEFT"
                    elif (
                        lidar_left_count > lidar_right_count or
                        lidar_right_score < lidar_left_score
                    ):
                        steering = FUSION_OVERRIDE_HARD
                        final_direction = "RIGHT"
                    elif lidar_front_left >= lidar_front_right:
                        steering = -FUSION_OVERRIDE_HARD
                        final_direction = "LEFT"
                    else:
                        steering = FUSION_OVERRIDE_HARD
                        final_direction = "RIGHT"

                rpm_kiri, rpm_kanan = steering_to_rpm(steering)

                status_jalan = "CAMERA BODY + LIDAR"
                perintah_robot = "HINDAR {}".format(final_direction)
                warna_status = (0, 255, 255)

                control_send_rpm(
                    rpm_kiri,
                    rpm_kanan
                )
            else:
                rpm_kiri, rpm_kanan = motor_stop()

                status_jalan = "CAMERA BODY GUARD"
                perintah_robot = "STOP - TIDAK ADA SISI AMAN"
                warna_status = (0, 0, 255)

                control_send_rpm(
                    rpm_kiri,
                    rpm_kanan,
                    force=True
                )

        elif obstacle_action == "TURN_BACK" and final_direction != "UTURN":
            if lidar_has_scan and not (
                lidar_now_action == "STOP"
            ):
                if (
                    lidar_right_count > lidar_left_count or
                    lidar_left_score < lidar_right_score
                ):
                    steering = -FUSION_OVERRIDE_HARD
                    final_direction = "LEFT"
                elif (
                    lidar_left_count > lidar_right_count or
                    lidar_right_score < lidar_left_score
                ):
                    steering = FUSION_OVERRIDE_HARD
                    final_direction = "RIGHT"
                elif lidar_front_left >= lidar_front_right:
                    steering = -FUSION_OVERRIDE_HARD
                    final_direction = "LEFT"
                else:
                    steering = FUSION_OVERRIDE_HARD
                    final_direction = "RIGHT"

                rpm_kiri, rpm_kanan = steering_to_rpm(steering)

                status_jalan = "YOLO DEPAN + LIDAR"
                perintah_robot = "HINDAR {}".format(final_direction)
                warna_status = (0, 255, 255)

                control_send_rpm(
                    rpm_kiri,
                    rpm_kanan
                )
            else:
                rpm_kiri, rpm_kanan = motor_stop()

                status_jalan = "YOLO OBSTACLE TENGAH"
                perintah_robot = "STOP - TIDAK ADA SISI AMAN"
                warna_status = (0, 0, 255)

                control_send_rpm(
                    rpm_kiri,
                    rpm_kanan,
                    force=True
                )

        elif obstacle_action == "AVOID_RIGHT":
            steering = +72.0
            rpm_kiri, rpm_kanan = steering_to_rpm(
                steering
            )

            status_jalan = "YOLO OBSTACLE KIRI"
            perintah_robot = "HINDAR KANAN"
            warna_status = (0, 255, 255)

            control_send_rpm(
                rpm_kiri,
                rpm_kanan
            )

        elif obstacle_action == "AVOID_LEFT":
            steering = -72.0
            rpm_kiri, rpm_kanan = steering_to_rpm(
                steering
            )

            status_jalan = "YOLO OBSTACLE KANAN"
            perintah_robot = "HINDAR KIRI"
            warna_status = (0, 255, 255)

            control_send_rpm(
                rpm_kiri,
                rpm_kanan
            )

        elif lidar_escape_state == "STOPPING":
            rpm_kiri, rpm_kanan = motor_stop()

            status_jalan = "LIDAR TERKEPUNG"
            perintah_robot = "STOP SEBELUM MUNDUR"
            warna_status = (0, 0, 255)

            control_send_rpm(
                rpm_kiri,
                rpm_kanan,
                force=True
            )

            if (
                now_control -
                lidar_escape_started
            ) >= LIDAR_ESCAPE_STOP_SEC:
                lidar_escape_state = "REVERSING"
                lidar_escape_started = now_control

        elif lidar_escape_state == "REVERSING":
            elapsed_escape = (
                now_control -
                lidar_escape_started
            )

            if elapsed_escape < LIDAR_REVERSE_ESCAPE_SEC:
                # Reverse + turn toward remembered open side.
                if lidar_escape_direction == "LEFT":
                    # reverse left arc
                    rpm_kiri = MOTOR_REVERSE
                    rpm_kanan = MOTOR_NEUTRAL + 15
                else:
                    rpm_kiri = MOTOR_NEUTRAL + 15
                    rpm_kanan = MOTOR_REVERSE

                status_jalan = "ESCAPE MODE"
                perintah_robot = "MUNDUR {}".format(
                    lidar_escape_direction
                )
                warna_status = (0, 165, 255)

                control_send_rpm(
                    rpm_kiri,
                    rpm_kanan,
                    force=True
                )
            else:
                rpm_kiri, rpm_kanan = motor_stop()

                control_send_rpm(
                    rpm_kiri,
                    rpm_kanan,
                    force=True
                )

                lidar_escape_state = "NONE"

        elif lidar_now_action == "TRAPPED":
            lidar_escape_state = "STOPPING"
            lidar_escape_started = now_control

            # Read direction from current LiDAR recommendation.
            lidar_escape_direction = (
                "LEFT"
                if lidar_now_steer < 0
                else "RIGHT"
            )

            rpm_kiri, rpm_kanan = motor_stop()

            status_jalan = "LIDAR TERKEPUNG"
            perintah_robot = "SIAP ESCAPE {}".format(
                lidar_escape_direction
            )
            warna_status = (0, 0, 255)

            control_send_rpm(
                rpm_kiri,
                rpm_kanan,
                force=True
            )

        elif final_direction == "UTURN":
            status_jalan = "AUTO 360 U-TURN"
            perintah_robot = fusion_reason
            warna_status = (0, 165, 255)

            control_send_rpm(
                rpm_kiri,
                rpm_kanan,
                force=True
            )

        elif final_direction == "STOP":
            rpm_kiri, rpm_kanan = motor_stop()

            status_jalan = "FUSION STOP"
            perintah_robot = fusion_reason
            warna_status = (0, 0, 255)

            control_send_rpm(
                rpm_kiri,
                rpm_kanan,
                force=True
            )

        elif lidar_now_action == "STOP":
            rpm_kiri, rpm_kanan = motor_stop()

            status_jalan = "LIDAR EMERGENCY"
            perintah_robot = lidar_now_reason
            warna_status = (0, 0, 255)

            control_send_rpm(
                rpm_kiri,
                rpm_kanan,
                force=True
            )

        elif lidar_now_action in ("AVOID_LEFT", "AVOID_RIGHT"):
            steering = lidar_now_steer
            rpm_kiri, rpm_kanan = steering_to_rpm(
                steering
            )

            status_jalan = "LIDAR {}".format(
                lidar_now_action
            )
            perintah_robot = lidar_now_reason
            warna_status = (0, 165, 255)

            control_send_rpm(
                rpm_kiri,
                rpm_kanan
            )

        elif lidar_now_action == "CLEAR":
            status_jalan = "LIDAR VPATH {}".format(
                final_direction
            )
            perintah_robot = fusion_reason

            warna_status = (
                (0, 255, 0)
                if final_direction != "STOP"
                else (0, 0, 255)
            )

            control_send_rpm(
                rpm_kiri,
                rpm_kanan
            )

        # ====================================================
        # CAMERA / YOLO BACKUP ONLY
        # ====================================================
        elif lidar_now_action == "TIMEOUT":
            if obstacle_action == "TURN_BACK":
                # Backup kamera melihat tengah tertutup.
                rpm_kiri, rpm_kanan = motor_stop()

                status_jalan = "LIDAR OFF - CAMERA BACKUP"
                perintah_robot = "OBSTACLE TENGAH -> STOP"
                warna_status = (0, 0, 255)

                control_send_rpm(
                    rpm_kiri,
                    rpm_kanan,
                    force=True
                )

            elif obstacle_action == "AVOID_RIGHT":
                steering = +75.0
                rpm_kiri, rpm_kanan = steering_to_rpm(
                    steering
                )

                status_jalan = "CAMERA BACKUP"
                perintah_robot = "HINDAR KANAN"
                warna_status = (0, 255, 255)

                control_send_rpm(
                    rpm_kiri,
                    rpm_kanan
                )

            elif obstacle_action == "AVOID_LEFT":
                steering = -75.0
                rpm_kiri, rpm_kanan = steering_to_rpm(
                    steering
                )

                status_jalan = "CAMERA BACKUP"
                perintah_robot = "HINDAR KIRI"
                warna_status = (0, 255, 255)

                control_send_rpm(
                    rpm_kiri,
                    rpm_kanan
                )

            elif camera_backup_active:
                status_jalan = "LIDAR TIMEOUT"
                perintah_robot = "CAMERA BACKUP SAFETY"
                warna_status = (255, 255, 0)

                control_send_rpm(
                    rpm_kiri,
                    rpm_kanan
                )

            else:
                rpm_kiri, rpm_kanan = motor_stop()

                status_jalan = "LIDAR + CAMERA LOST"
                perintah_robot = "FAILSAFE STOP"
                warna_status = (0, 0, 255)

                control_send_rpm(
                    rpm_kiri,
                    rpm_kanan,
                    force=True
                )

        processing_ms = (
            time.perf_counter() -
            total_t0
        ) * 1000.0

        now = time.monotonic()
        frame_age_ms = (now - src_ts) * 1000.0

        count += 1

        if now - fps_t0 >= 1.0:
            processing_fps = count / (now - fps_t0)
            count = 0
            fps_t0 = now

            print(
                "[CAM {:.1f}] [PROC {:.1f}] "
                "[P {:.1f}ms] [PATH {:.1f}ms] "
                "[AGE {:.1f}ms] "
                "[STEER {:+.1f}] [CMD L:{} R:{}]"
                .format(
                    camera_fps,
                    processing_fps,
                    processing_ms,
                    processing_ms,
                    frame_age_ms,
                    steering,
                    rpm_kiri,
                    rpm_kanan
                )
            )

        # ====================================================
        # TEKS STATUS
        # ====================================================

        cv2.putText(
            frame_hasil,
            status_jalan,
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            warna_status,
            2
        )

        cv2.putText(
            frame_hasil,
            perintah_robot,
            (20, 64),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.68,
            warna_status,
            2
        )

        cv2.putText(
            frame_hasil,
            "EDGE:{:.2f} WIDTH:{:.0f}px MID:{:.0f}".format(
                path_info["confidence"],
                path_info["near_width"],
                path_info["target_x"]
            ),
            (20, 94),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (255, 255, 255),
            2
        )

        cv2.putText(
            frame_hasil,
            "STEER: {:+.1f}".format(steering),
            (20, 121),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (255, 255, 255),
            2
        )

        cv2.putText(
            frame_hasil,
            "CMD L:{:03d}  R:{:03d}".format(rpm_kiri, rpm_kanan),
            (20, 148),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (255, 255, 255),
            2
        )

        cv2.putText(
            frame_hasil,
            "LOCAL: {}".format(
                "ON" if enabled_now else "OFF"
            ),
            (20, 174),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (0, 255, 0) if enabled_now else (0, 0, 255),
            2
        )

        cv2.putText(
            frame_hasil,
            "CAM {:.1f} PROC {:.1f} P {:.1f}ms".format(
                camera_fps,
                processing_fps,
                processing_ms
            ),
            (20, 201),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.43,
            (255, 255, 0),
            1
        )

        cv2.putText(
            frame_hasil,
            "YOLO {:.0f}ms | {} | L:{} C:{} R:{}".format(yolo_infer_ms_now, obstacle_action, obstacle_left_px, obstacle_center_px, obstacle_right_px),
            (20, 225),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.43,
            (255, 255, 0),
            1
        )

        cv2.putText(
            frame_hasil,
            "LIDAR {} | F:{} FL:{} FR:{} L:{} R:{}".format(
                lidar_now_action,
                fmt_lidar_mm(lidar_front),
                fmt_lidar_mm(lidar_front_left),
                fmt_lidar_mm(lidar_front_right),
                fmt_lidar_mm(lidar_left),
                fmt_lidar_mm(lidar_right)
            ),
            (20, 248),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.40,
            (0, 255, 255) if lidar_now_action == "CLEAR" else (0, 165, 255),
            1
        )

        cv2.putText(
            frame_hasil,
            "BODY FILTER <= {:.0f}mm IGNORE".format(
                LIDAR_SELF_IGNORE_RADIUS_MM
            ),
            (20, 272),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.38,
            (180, 180, 180),
            1
        )

        cv2.putText(
            frame_hasil,
            "CAM GUARD:{} {} STEER:{:+.1f} L:{} C:{} R:{}".format(
                cam_guard_level,
                cam_guard_label,
                cam_guard_steer,
                cam_guard_left_px,
                cam_guard_center_px,
                cam_guard_right_px
            ),
            (20, 294),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.38,
            (255, 0, 255)
            if cam_guard_level == "WARN"
            else (
                (0, 165, 255)
                if cam_guard_level == "HARD"
                else (180, 180, 180)
            ),
            1
        )

        cv2.putText(
            frame_hasil,
            "CONF L[LID:{} YOLO:{}] R[LID:{} YOLO:{}]".format(
                int(lidar_confirm_left),
                int(yolo_confirm_left),
                int(lidar_confirm_right),
                int(yolo_confirm_right)
            ),
            (20, 334),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.38,
            (0, 255, 255),
            1
        )

        render_lidar_map()

        cv2.imshow(
            WINDOW_NAME,
            frame_hasil
        )

        key = cv2.waitKey(1) & 0xFF

        if key == 32:  # SPACE
            toggle_robot()

        elif key == 27 or key == ord("q"):
            break

    stop_all()

    try:
        cv2.destroyAllWindows()
    except Exception:
        pass


if __name__ == "__main__":
    main()