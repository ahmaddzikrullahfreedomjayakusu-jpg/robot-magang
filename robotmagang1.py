#!/usr/bin/env python3
import os
import time
import signal
import threading
import socket
import subprocess

import cv2
import numpy as np
from ultralytics import YOLO

# ============================================================
# FREEDOM - LAPTOP + HP MJPEG + BODY ROBOT + LOGIKA GARIS
# PROCESSING LAPTOP + YOLO SEGMENTATION + LOCAL SPACE CONTROL
# ============================================================

HP_IP = "192.168.0.149"
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
YOLO_IMGSZ = 320
YOLO_CONF = 0.40
YOLO_EVERY_N_FRAMES = 2

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
# 127 = STOP / 0 RPM
# 157 = maju sekitar +30 dari netral
# 97  = mundur sekitar -30 dari netral
# ============================================================
MOTOR_NEUTRAL = 127
MOTOR_STEP = 30

MOTOR_FORWARD = MOTOR_NEUTRAL + MOTOR_STEP   # 157
MOTOR_REVERSE = MOTOR_NEUTRAL - MOTOR_STEP   # 97

# Koreksi ringan: roda dalam tetap maju tapi lebih pelan.
MOTOR_FORWARD_SLOW = MOTOR_NEUTRAL + 20      # 147

# Putar di tempat:
# kiri maju + kanan mundur atau sebaliknya.
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
#   157 = MAJU (+30)
#    97 = MUNDUR (-30)
# ============================================================

CONTROL_HP_IP = "192.168.0.149"
CONTROL_TCP_PORT = 8888

SEND_INTERVAL = 0.05

sock = None
sock_lock = threading.Lock()

_last_send = 0.0
_last_cmd = None


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
# LOCAL START / STOP
# SPACE = toggle START / STOP
# Default OFF untuk keamanan.
# ============================================================

robot_enabled = False
robot_lock = threading.Lock()

WINDOW_NAME = "FREEDOM - BODY + GARIS"

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
    Mapping fisik SUDAH DIBALIK sesuai robot nyata.

    steering > 0 = koreksi kanan
    steering < 0 = koreksi kiri

    Karena motor kiri/kanan sebelumnya terbalik,
    command roda dibalik di sini.
    """
    if steering >= 70.0:
        # KANAN tajam
        return MOTOR_NEUTRAL, MOTOR_FORWARD

    if steering <= -70.0:
        # KIRI tajam
        return MOTOR_FORWARD, MOTOR_NEUTRAL

    if steering > 8.0:
        # KANAN ringan
        return MOTOR_FORWARD_SLOW, MOTOR_FORWARD

    if steering < -8.0:
        # KIRI ringan
        return MOTOR_FORWARD, MOTOR_FORWARD_SLOW

    return MOTOR_FORWARD, MOTOR_FORWARD


def motor_stop():
    return MOTOR_NEUTRAL, MOTOR_NEUTRAL


def motor_spin_turnback():
    """
    Putar di tempat.
    Left reverse (97), Right forward (157).
    """
    return SPIN_LEFT_CMD, SPIN_RIGHT_CMD


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
    global gst_camera
    global latest_camera, latest_camera_id, latest_camera_ts
    global camera_fps

    cmd = (
        "/usr/bin/gst-launch-1.0 -q "
        "souphttpsrc location={} is-live=true do-timestamp=true ! "
        "multipartdemux ! image/jpeg ! jpegparse ! "
        "queue leaky=downstream max-size-buffers=1 "
        "max-size-bytes=0 max-size-time=0 ! "
        "avdec_mjpeg ! videoconvert ! "
        "video/x-raw,format=BGR,width={},height={} ! "
        "fdsink fd=1 sync=false"
    ).format(MJPEG_URL, FRAME_W, FRAME_H)

    while running:
        print("[CAM] connecting {}".format(MJPEG_URL))

        try:
            gst_camera = subprocess.Popen(
                cmd,
                shell=True,
                executable="/bin/bash",
                stdout=subprocess.PIPE,
                stderr=None,
                bufsize=0
            )
        except Exception as e:
            print("[CAM] start error:", e)
            time.sleep(1.0)
            continue

        print("[CAM] ACTIVE")
        fd = gst_camera.stdout.fileno()

        count = 0
        t0 = time.monotonic()

        while running:
            raw = read_exact(fd, FRAME_BYTES)

            if raw is None:
                if running:
                    print("[CAM] disconnected")
                break

            frame = np.frombuffer(raw, dtype=np.uint8).reshape(
                (FRAME_H, FRAME_W, 3)
            ).copy()

            now = time.monotonic()

            with camera_lock:
                latest_camera = frame
                latest_camera_id += 1
                latest_camera_ts = now

            count += 1

            if now - t0 >= 1.0:
                camera_fps = count / (now - t0)
                count = 0
                t0 = now

        try:
            gst_camera.terminate()
        except Exception:
            pass

        if running:
            time.sleep(0.5)


# ============================================================
# GARIS HOUGH
# ============================================================

def draw_hough_boundary(frame_target, data_garis, warna):
    if data_garis is None:
        return None

    rho, theta, vote = data_garis

    c = np.cos(theta)
    s = np.sin(theta)

    if abs(c) < 1e-6:
        return None

    y1 = ROI_Y
    y2 = FRAME_H - 1

    x1 = int((rho - y1 * s) / c)
    x2 = int((rho - y2 * s) / c)

    p1 = (x1, y1)
    p2 = (x2, y2)

    cv2.line(frame_target, p1, p2, warna, 4)

    return p1, p2


def detect_boundaries(frame):
    global hough_ms

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    gx = cv2.Sobel(gray, cv2.CV_16S, 1, 0, ksize=3)
    gy = cv2.Sobel(gray, cv2.CV_16S, 0, 1, ksize=3)

    abs_gx = cv2.convertScaleAbs(gx)
    abs_gy = cv2.convertScaleAbs(gy)

    magnitude = cv2.addWeighted(abs_gx, 0.5, abs_gy, 0.5, 0)

    _, edge_binary = cv2.threshold(
        magnitude,
        EDGE_THRESHOLD,
        255,
        cv2.THRESH_BINARY
    )

    # Full-size ROI agar rho/theta memakai koordinat frame penuh.
    roi = np.zeros_like(edge_binary)
    roi[ROI_Y:, :] = edge_binary[ROI_Y:, :]

    ht0 = time.perf_counter()

    lines = cv2.HoughLines(
        roi,
        1,
        np.pi / 180.0,
        HOUGH_THRESHOLD
    )

    hough_ms = (time.perf_counter() - ht0) * 1000.0

    garis_kiri = None
    garis_kanan = None

    if lines is not None:
        for item in lines[:100]:
            rho = float(item[0][0])
            theta = float(item[0][1])
            deg = float(np.degrees(theta))

            if garis_kiri is None:
                if abs(deg - LEFT_THETA_DEG) <= THETA_TOLERANCE:
                    garis_kiri = (rho, theta, HOUGH_THRESHOLD)

            if garis_kanan is None:
                if abs(deg - RIGHT_THETA_DEG) <= THETA_TOLERANCE:
                    garis_kanan = (rho, theta, HOUGH_THRESHOLD)

            if garis_kiri is not None and garis_kanan is not None:
                break

    return edge_binary, garis_kiri, garis_kanan


# ============================================================
# CLEARANCE BODY TERHADAP GARIS
# ============================================================

def hough_x_at_y(data_garis, y):
    if data_garis is None:
        return None

    rho, theta, vote = data_garis

    c = np.cos(theta)
    s = np.sin(theta)

    if abs(c) < 1e-6:
        return None

    return float((rho - y * s) / c)


def hitung_clearance_body(body_points, garis_kiri, garis_kanan, samples=20):
    kiri_atas, kanan_atas, kanan_bawah, kiri_bawah = body_points

    jarak_kiri_semua = []
    jarak_kanan_semua = []

    for t in np.linspace(0.0, 1.0, samples):
        body_kiri = lerp_point(kiri_atas, kiri_bawah, t)
        body_kanan = lerp_point(kanan_atas, kanan_bawah, t)

        y = float((body_kiri[1] + body_kanan[1]) / 2.0)

        x_line_kiri = hough_x_at_y(garis_kiri, y)
        if x_line_kiri is not None:
            jarak_kiri_semua.append(
                float(body_kiri[0] - x_line_kiri)
            )

        x_line_kanan = hough_x_at_y(garis_kanan, y)
        if x_line_kanan is not None:
            jarak_kanan_semua.append(
                float(x_line_kanan - body_kanan[0])
            )

    clearance_kiri = (
        min(jarak_kiri_semua)
        if jarak_kiri_semua
        else None
    )

    clearance_kanan = (
        min(jarak_kanan_semua)
        if jarak_kanan_semua
        else None
    )

    return clearance_kiri, clearance_kanan


# ============================================================
# LOGIKA JALAN - SAMA DENGAN REFERENSI, TANPA YOLO
# ============================================================

MARGIN_KOREKSI = 45.0
MARGIN_BAHAYA = 15.0


def decision_from_clearance(clearance_kiri, clearance_kanan):
    perintah_robot = "JALAN LURUS"
    status_jalan = "GARIS AMAN"
    warna_status = (0, 255, 0)
    steering = 0.0

    if clearance_kiri is None and clearance_kanan is None:
        status_jalan = "GARIS PEMBATAS TIDAK TERDETEKSI"
        perintah_robot = "TAHAN / LURUS"
        warna_status = (0, 255, 255)

    else:
        if clearance_kiri is not None and clearance_kiri <= 0:
            status_jalan = "BODY SENTUH GARIS KIRI"
            perintah_robot = "KOREKSI KANAN KUAT"
            warna_status = (0, 0, 255)
            steering = 100.0

        elif clearance_kanan is not None and clearance_kanan <= 0:
            status_jalan = "BODY SENTUH GARIS KANAN"
            perintah_robot = "KOREKSI KIRI KUAT"
            warna_status = (0, 0, 255)
            steering = -100.0

        elif clearance_kiri is not None and clearance_kiri <= MARGIN_BAHAYA:
            status_jalan = "TERLALU DEKAT KIRI"
            perintah_robot = "KOREKSI KANAN KUAT"
            warna_status = (0, 165, 255)
            steering = 80.0

        elif clearance_kanan is not None and clearance_kanan <= MARGIN_BAHAYA:
            status_jalan = "TERLALU DEKAT KANAN"
            perintah_robot = "KOREKSI KIRI KUAT"
            warna_status = (0, 165, 255)
            steering = -80.0

        elif clearance_kiri is not None and clearance_kiri < MARGIN_KOREKSI:
            status_jalan = "DEKAT GARIS KIRI"
            perintah_robot = "KOREKSI KANAN"
            warna_status = (0, 255, 255)

            if clearance_kanan is not None:
                error = clearance_kanan - clearance_kiri
                steering = max(20.0, min(70.0, 0.7 * error))
            else:
                steering = 40.0

        elif clearance_kanan is not None and clearance_kanan < MARGIN_KOREKSI:
            status_jalan = "DEKAT GARIS KANAN"
            perintah_robot = "KOREKSI KIRI"
            warna_status = (0, 255, 255)

            if clearance_kiri is not None:
                error = clearance_kanan - clearance_kiri
                steering = min(-20.0, max(-70.0, 0.7 * error))
            else:
                steering = -40.0

        elif clearance_kiri is not None and clearance_kanan is not None:
            error = clearance_kanan - clearance_kiri
            steering = max(-35.0, min(35.0, 0.35 * error))

            if steering > 8.0:
                status_jalan = "AMAN - KOREKSI KANAN SEDIKIT"
                perintah_robot = "JALAN KANAN SEDIKIT"

            elif steering < -8.0:
                status_jalan = "AMAN - KOREKSI KIRI SEDIKIT"
                perintah_robot = "JALAN KIRI SEDIKIT"

            else:
                status_jalan = "POSISI TENGAH - AMAN"
                perintah_robot = "JALAN LURUS"
                steering = 0.0

        elif clearance_kiri is not None:
            status_jalan = "GARIS KIRI TERDETEKSI - AMAN"
            perintah_robot = "JALAN LURUS"
            steering = 0.0

        elif clearance_kanan is not None:
            status_jalan = "GARIS KANAN TERDETEKSI - AMAN"
            perintah_robot = "JALAN LURUS"
            steering = 0.0

    return status_jalan, perintah_robot, warna_status, steering



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


# ============================================================
# MAIN
# ============================================================

def main():
    global processing_fps, processing_ms, frame_age_ms
    global yolo_frame_counter, yolo_input_frame

    signal.signal(signal.SIGINT, stop_all)
    signal.signal(signal.SIGTERM, stop_all)

    print("")
    print("==========================================")
    print(" LOCAL KEYBOARD CONTROL")
    print("==========================================")
    print("SPACE : START / STOP")
    print("Q/ESC : EXIT")
    print("==========================================")
    print("")

    # Serial STM32 dihubungkan otomatis langsung dari program ini.
    # Jangan blok startup kamera/display karena port kontrol HP.
    print("")
    print("==========================================")
    print(" LOCAL CONTROL")
    print("==========================================")
    print("SPACE = START / STOP toggle")
    print("Q / ESC = keluar")
    print("==========================================")
    print("")

    print("[LOCAL] Kontrol robot dari keyboard laptop: SPACE toggle START/STOP", flush=True)
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
        target=yolo_worker_loop,
        daemon=True
    ).start()

    print("")
    print("==========================================")
    print(" FREEDOM - LAPTOP -> HP BRIDGE -> STM32 + BODY + GARIS")
    print("==========================================")
    print("Camera :", MJPEG_URL)
    print("YOLO   : AKTIF HANYA SAAT MASK MENYENTUH ZONA MERAH")
    print("Motor  : Laptop -> TCP HP:8888 -> USB OTG -> STM32")
    print("Body   : (-5,-2.80)->(-3.75,-2), kanan mirror x=0")
    print("Control: SPACE = START/STOP toggle")
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


    # Body persis dari koordinat Desmos pada frame 640x480.
    body_points = build_body_points(FRAME_W, FRAME_H)

    last_id = -1
    count = 0
    fps_t0 = time.monotonic()

    obstacle_state = "FOLLOW"   # FOLLOW / STOPPING / TURNING
    obstacle_state_started = 0.0
    last_turn_finished = -999.0

    while running:
        with camera_lock:
            fid = latest_camera_id
            src = latest_camera
            src_ts = latest_camera_ts

        if src is None or fid == last_id:
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

        edge_binary, garis_kiri, garis_kanan = detect_boundaries(frame)

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


        draw_hough_boundary(
            frame_hasil,
            garis_kiri,
            (0, 255, 0)
        )

        draw_hough_boundary(
            frame_hasil,
            garis_kanan,
            (0, 0, 255)
        )

        # BODY 4 WARNA PERSIS REFERENSI.
        draw_robot_body_4colors(
            frame_hasil,
            body_points,
            alpha=0.33
        )

        clearance_kiri, clearance_kanan = hitung_clearance_body(
            body_points,
            garis_kiri,
            garis_kanan,
            samples=20
        )

        (
            status_jalan,
            perintah_robot,
            warna_status,
            steering
        ) = decision_from_clearance(
            clearance_kiri,
            clearance_kanan
        )

        # ====================================================
        # EKSEKUSI KE STM32 + OBSTACLE 3-ZONE
        #
        # CENTER -> STOP -> PUTAR BALIK
        # LEFT   -> HINDAR KE KANAN
        # RIGHT  -> HINDAR KE KIRI
        # NONE   -> LINE FOLLOWING NORMAL
        # ====================================================
        now_control = time.monotonic()

        rpm_kiri, rpm_kanan = steering_to_rpm(steering)

        no_boundary = (
            clearance_kiri is None and
            clearance_kanan is None
        )

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

        if not enabled_now:
            obstacle_state = "FOLLOW"

            rpm_kiri, rpm_kanan = motor_stop()

            # Remote OFF = motor tetap STOP, tetapi keputusan YOLO
            # tetap ditampilkan di layar untuk pengujian.
            if obstacle_action == "NONE":
                status_jalan = status_jalan
                perintah_robot = perintah_robot

            control_send_rpm(
                rpm_kiri,
                rpm_kanan
            )

        elif obstacle_state == "STOPPING":
            rpm_kiri, rpm_kanan = motor_stop()

            status_jalan = "OBSTACLE TENGAH: {}".format(
                obstacle_label.upper()
            )
            perintah_robot = "STOP DULU"
            warna_status = (0, 0, 255)

            control_send_rpm(
                rpm_kiri,
                rpm_kanan,
                force=True
            )

            if (
                now_control - obstacle_state_started
            ) >= OBSTACLE_STOP_SECONDS:
                obstacle_state = "TURNING"
                obstacle_state_started = now_control

                print("[ROBOT] MULAI PUTAR BALIK")

        elif obstacle_state == "TURNING":
            elapsed_turn = (
                now_control -
                obstacle_state_started
            )

            if elapsed_turn < TURN_BACK_SECONDS:
                rpm_kiri, rpm_kanan = motor_spin_turnback()

                status_jalan = "OBSTACLE TENGAH"
                perintah_robot = "PUTAR BALIK"
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

                obstacle_state = "FOLLOW"
                last_turn_finished = now_control

                status_jalan = "PUTAR BALIK SELESAI"
                perintah_robot = "LANJUT FOLLOW GARIS"
                warna_status = (0, 255, 0)

                print("[ROBOT] PUTAR BALIK SELESAI")

        # Object di KIRI depan -> hindari ke KANAN.
        elif obstacle_action == "AVOID_RIGHT":
            # Fisik robot: command roda dibalik agar benar-benar ke KANAN.
            rpm_kiri = MOTOR_NEUTRAL
            rpm_kanan = MOTOR_FORWARD

            status_jalan = "OBSTACLE KIRI: {}".format(
                obstacle_label.upper()
            )
            perintah_robot = "HINDAR KE KANAN"
            warna_status = (0, 255, 255)

            control_send_rpm(
                rpm_kiri,
                rpm_kanan
            )

        # Object di KANAN depan -> hindari ke KIRI.
        elif obstacle_action == "AVOID_LEFT":
            # Fisik robot: command roda dibalik agar benar-benar ke KIRI.
            rpm_kiri = MOTOR_FORWARD
            rpm_kanan = MOTOR_NEUTRAL

            status_jalan = "OBSTACLE KANAN: {}".format(
                obstacle_label.upper()
            )
            perintah_robot = "HINDAR KE KIRI"
            warna_status = (0, 255, 255)

            control_send_rpm(
                rpm_kiri,
                rpm_kanan
            )

        elif no_boundary:
            rpm_kiri, rpm_kanan = motor_stop()

            status_jalan = "GARIS HILANG - FAILSAFE STOP"
            perintah_robot = "STOP"
            warna_status = (0, 0, 255)

            control_send_rpm(
                rpm_kiri,
                rpm_kanan,
                force=True
            )

        else:
            # Tidak ada obstacle yang masuk area body depan.
            # Tetap pakai hasil line following.
            control_send_rpm(
                rpm_kiri,
                rpm_kanan
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
                "[P {:.1f}ms] [H {:.1f}ms] "
                "[AGE {:.1f}ms] "
                "[STEER {:+.1f}] [CMD L:{} R:{}]"
                .format(
                    camera_fps,
                    processing_fps,
                    processing_ms,
                    hough_ms,
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

        teks_kiri = (
            "LEFT: --"
            if clearance_kiri is None
            else "LEFT: {:.1f}px".format(clearance_kiri)
        )

        teks_kanan = (
            "RIGHT: --"
            if clearance_kanan is None
            else "RIGHT: {:.1f}px".format(clearance_kanan)
        )

        cv2.putText(
            frame_hasil,
            "{}  {}".format(teks_kiri, teks_kanan),
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
            "CAM {:.1f} PROC {:.1f} P {:.1f}ms H {:.1f}ms".format(
                camera_fps,
                processing_fps,
                processing_ms,
                hough_ms
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

        cv2.imshow(WINDOW_NAME, frame_hasil)

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