#!/usr/bin/env python3
import os
import time
import signal
import threading
import subprocess

import cv2
import numpy as np
from flask import Flask, Response

# ============================================================
# FREEDOM - LAPTOP + HP MJPEG + BODY ROBOT + LOGIKA GARIS
# PROCESSING DI LAPTOP, YOLO OFF
# ============================================================

HP_IP = "192.168.0.149"
MJPEG_URL = "http://{}:8080/mjpeg".format(HP_IP)

FRAME_W = 640
FRAME_H = 480
FRAME_BYTES = FRAME_W * FRAME_H * 3

ROI_START_RATIO = 0.55
EDGE_THRESHOLD = 80

# OpenCV HoughLines theta = 0..180.
# Program referensi: kiri +63, kanan -63.
LEFT_THETA_DEG = 63.0
RIGHT_THETA_DEG = 117.0   # ekuivalen -63 derajat
THETA_TOLERANCE = 8.0
HOUGH_THRESHOLD = 40

RPM_NORMAL = 35
RPM_KOREKSI = 30
RPM_TAJAM = 35

# ============================================================
# LAPTOP -> STM32 TCP BRIDGE
#
# Jalur kamera:
#   HP -> Wi-Fi MJPEG -> LAPTOP
#
# Jalur kontrol:
#   PROCESSING LAPTOP -> TCP localhost:5006 -> stm_bridge.py
#   -> USB Serial -> STM32
#
# Jalankan stm_bridge.py di terminal lain sebelum program ini.
#
# Format paket tetap:
# [0] 0xAA
# [1] RPM kanan
# [2] RPM kiri
# [3] checksum = (0xAA + kanan + kiri) & 0xFF
# [4] 0x55
# ============================================================

HP_BRIDGE_IP = "127.0.0.1"
HP_BRIDGE_PORT = 5006

SEND_INTERVAL = 0.05  # maksimum 20 Hz

_control_sock = None
_control_lock = threading.Lock()

_last_send = 0.0
_last_cmd = None
_last_connect_attempt = 0.0

CONTROL_RECONNECT_INTERVAL = 1.0


def control_disconnect():
    global _control_sock

    with _control_lock:
        if _control_sock is not None:
            try:
                _control_sock.close()
            except Exception:
                pass

        _control_sock = None


def control_connect(force=False):
    """
    Hubungkan program processing laptop ke stm_bridge.py lokal.

    Return:
      True  -> socket aktif
      False -> belum terhubung
    """
    global _control_sock
    global _last_connect_attempt

    now = time.monotonic()

    with _control_lock:
        if _control_sock is not None:
            return True

        if (
            not force and
            (now - _last_connect_attempt) < CONTROL_RECONNECT_INTERVAL
        ):
            return False

        _last_connect_attempt = now

        try:
            import socket

            sock = socket.socket(
                socket.AF_INET,
                socket.SOCK_STREAM
            )

            # Jangan biarkan control loop macet lama.
            sock.settimeout(0.35)

            sock.connect((
                HP_BRIDGE_IP,
                HP_BRIDGE_PORT
            ))

            # Setelah connect, timeout tetap pendek agar gagal cepat
            # jika HP bridge putus.
            sock.settimeout(0.20)

            _control_sock = sock

            print(
                "[CTRL] Terhubung ke STM bridge lokal {}:{}"
                .format(
                    HP_BRIDGE_IP,
                    HP_BRIDGE_PORT
                )
            )

            return True

        except Exception as e:
            try:
                sock.close()
            except Exception:
                pass

            _control_sock = None

            print(
                "[CTRL] Belum terhubung ke STM bridge lokal: {}"
                .format(e)
            )

            return False


def control_send_rpm(rpm_kiri, rpm_kanan, force=False):
    """
    Kirim RPM dari processing laptop ke stm_bridge.py.

    Packet:
      0xAA
      RPM kanan
      RPM kiri
      checksum
      0x55
    """
    global _last_send
    global _last_cmd
    global _control_sock

    rpm_kiri = max(
        0,
        min(255, int(round(rpm_kiri)))
    )

    rpm_kanan = max(
        0,
        min(255, int(round(rpm_kanan)))
    )

    now = time.monotonic()
    cmd = (
        rpm_kiri,
        rpm_kanan
    )

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

    _last_send = now
    _last_cmd = cmd

    checksum = (
        0xAA +
        rpm_kanan +
        rpm_kiri
    ) & 0xFF

    packet = bytes([
        0xAA,
        rpm_kanan,
        rpm_kiri,
        checksum,
        0x55
    ])

    if not control_connect():
        return False

    with _control_lock:
        sock = _control_sock

        if sock is None:
            return False

        try:
            sock.sendall(packet)
            return True

        except Exception as e:
            print(
                "[CTRL] Kirim gagal: {}"
                .format(e)
            )

            try:
                sock.close()
            except Exception:
                pass

            _control_sock = None
            return False


# ============================================================
# REMOTE START / STOP
# Default OFF untuk keamanan.
# ============================================================

WEB_PORT = 5000
robot_enabled = False
robot_lock = threading.Lock()

app = Flask(__name__)

HTML_REMOTE = """#!/usr/bin/env python3
import os
import time
import signal
import threading
import subprocess

import cv2
import numpy as np
from flask import Flask, Response

# ============================================================
# FREEDOM - LAPTOP + HP MJPEG + BODY ROBOT + LOGIKA GARIS
# PROCESSING DI LAPTOP, YOLO OFF
# ============================================================

HP_IP = "192.168.0.147"
MJPEG_URL = "http://{}:8080/mjpeg".format(HP_IP)

FRAME_W = 640
FRAME_H = 480
FRAME_BYTES = FRAME_W * FRAME_H * 3

ROI_START_RATIO = 0.55
EDGE_THRESHOLD = 80

# OpenCV HoughLines theta = 0..180.
# Program referensi: kiri +63, kanan -63.
LEFT_THETA_DEG = 63.0
RIGHT_THETA_DEG = 117.0   # ekuivalen -63 derajat
THETA_TOLERANCE = 8.0
HOUGH_THRESHOLD = 40

RPM_NORMAL = 35
RPM_KOREKSI = 30
RPM_TAJAM = 35

# ============================================================
# LAPTOP -> STM32 TCP BRIDGE
#
# Jalur kamera:
#   HP -> Wi-Fi MJPEG -> LAPTOP
#
# Jalur kontrol:
#   PROCESSING LAPTOP -> TCP localhost:5006 -> stm_bridge.py
#   -> USB Serial -> STM32
#
# Jalankan stm_bridge.py di terminal lain sebelum program ini.
#
# Format paket tetap:
# [0] 0xAA
# [1] RPM kanan
# [2] RPM kiri
# [3] checksum = (0xAA + kanan + kiri) & 0xFF
# [4] 0x55
# ============================================================

HP_BRIDGE_IP = "127.0.0.1"
HP_BRIDGE_PORT = 5006

SEND_INTERVAL = 0.05  # maksimum 20 Hz

_control_sock = None
_control_lock = threading.Lock()

_last_send = 0.0
_last_cmd = None
_last_connect_attempt = 0.0

CONTROL_RECONNECT_INTERVAL = 1.0


def control_disconnect():
    global _control_sock

    with _control_lock:
        if _control_sock is not None:
            try:
                _control_sock.close()
            except Exception:
                pass

        _control_sock = None


def control_connect(force=False):
    """
    Hubungkan program processing laptop ke stm_bridge.py lokal.

    Return:
      True  -> socket aktif
      False -> belum terhubung
    """
    global _control_sock
    global _last_connect_attempt

    now = time.monotonic()

    with _control_lock:
        if _control_sock is not None:
            return True

        if (
            not force and
            (now - _last_connect_attempt) < CONTROL_RECONNECT_INTERVAL
        ):
            return False

        _last_connect_attempt = now

        try:
            import socket

            sock = socket.socket(
                socket.AF_INET,
                socket.SOCK_STREAM
            )

            # Jangan biarkan control loop macet lama.
            sock.settimeout(0.35)

            sock.connect((
                HP_BRIDGE_IP,
                HP_BRIDGE_PORT
            ))

            # Setelah connect, timeout tetap pendek agar gagal cepat
            # jika HP bridge putus.
            sock.settimeout(0.20)

            _control_sock = sock

            print(
                "[CTRL] Terhubung ke STM bridge lokal {}:{}"
                .format(
                    HP_BRIDGE_IP,
                    HP_BRIDGE_PORT
                )
            )

            return True

        except Exception as e:
            try:
                sock.close()
            except Exception:
                pass

            _control_sock = None

            print(
                "[CTRL] Belum terhubung ke STM bridge lokal: {}"
                .format(e)
            )

            return False


def control_send_rpm(rpm_kiri, rpm_kanan, force=False):
    """
    Kirim RPM dari processing laptop ke stm_bridge.py.

    Packet:
      0xAA
      RPM kanan
      RPM kiri
      checksum
      0x55
    """
    global _last_send
    global _last_cmd
    global _control_sock

    rpm_kiri = max(
        0,
        min(255, int(round(rpm_kiri)))
    )

    rpm_kanan = max(
        0,
        min(255, int(round(rpm_kanan)))
    )

    now = time.monotonic()
    cmd = (
        rpm_kiri,
        rpm_kanan
    )

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

    _last_send = now
    _last_cmd = cmd

    checksum = (
        0xAA +
        rpm_kanan +
        rpm_kiri
    ) & 0xFF

    packet = bytes([
        0xAA,
        rpm_kanan,
        rpm_kiri,
        checksum,
        0x55
    ])

    if not control_connect():
        return False

    with _control_lock:
        sock = _control_sock

        if sock is None:
            return False

        try:
            sock.sendall(packet)
            return True

        except Exception as e:
            print(
                "[CTRL] Kirim gagal: {}"
                .format(e)
            )

            try:
                sock.close()
            except Exception:
                pass

            _control_sock = None
            return False


# ============================================================
# REMOTE START / STOP
# Default OFF untuk keamanan.
# ============================================================

WEB_PORT = 5000
robot_enabled = False
robot_lock = threading.Lock()

app = Flask(__name__)

HTML_REMOTE = """
<!doctype html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Freedom Robot Remote</title>
<style>
body{
    font-family:Arial;
    background:#111;
    color:white;
    text-align:center;
    margin:0;
    padding:30px
}
h1{font-size:28px}
.status{font-size:24px;margin:20px}
button{
    width:80%;
    max-width:420px;
    height:90px;
    font-size:30px;
    font-weight:bold;
    border:0;
    border-radius:18px;
    margin:12px
}
.start{background:#18c964;color:white}
.stop{background:#f31260;color:white}
</style>
</head>
<body>
<h1>FREEDOM ROBOT REMOTE</h1>
<div class="status">Status: <b id="st">OFF</b></div>
<button class="start" onclick="cmd('/start')">START</button><br>
<button class="stop" onclick="cmd('/stop')">STOP</button>

<script>
async function cmd(url){
    const r = await fetch(url,{method:'POST'});
    document.getElementById('st').innerText = await r.text();
}
async function refresh(){
    try{
        const r = await fetch('/status');
        document.getElementById('st').innerText = await r.text();
    }catch(e){}
}
setInterval(refresh,500);
refresh();
</script>
</body>
</html>
"""



@app.route("/")
def web_home():
    return Response(
        HTML_REMOTE,
        mimetype="text/html"
    )


@app.route("/start", methods=["POST"])
def web_start():
    global robot_enabled

    with robot_lock:
        robot_enabled = True

    print("[REMOTE] START")
    return "ON"


@app.route("/stop", methods=["POST"])
def web_stop():
    global robot_enabled

    with robot_lock:
        robot_enabled = False

    control_send_rpm(
        0,
        0,
        force=True
    )

    print("[REMOTE] STOP")
    return "OFF"


@app.route("/status")
def web_status():
    with robot_lock:
        return "ON" if robot_enabled else "OFF"


def start_web_server():
    app.run(
        host="0.0.0.0",
        port=WEB_PORT,
        debug=False,
        use_reloader=False,
        threaded=True
    )

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
    if steering >= 70.0:
        return RPM_TAJAM, 0

    if steering <= -70.0:
        return 0, RPM_TAJAM

    if steering > 8.0:
        return RPM_NORMAL, RPM_KOREKSI

    if steering < -8.0:
        return RPM_KOREKSI, RPM_NORMAL

    return RPM_NORMAL, RPM_NORMAL


# ============================================================
# CAMERA
# ============================================================

def stop_all(*_):
    global running
    running = False

    # Fail-safe: setiap program berhenti, motor STOP.
    try:
        control_send_rpm(
            0,
            0,
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


# ============================================================
# MAIN
# ============================================================

def main():
    global processing_fps, processing_ms, frame_age_ms

    signal.signal(signal.SIGINT, stop_all)
    signal.signal(signal.SIGTERM, stop_all)

    # stm_bridge.py lokal akan dihubungkan otomatis saat perintah motor dikirim.
    # Jangan blok startup kamera/display karena port kontrol HP.
    threading.Thread(
        target=start_web_server,
        daemon=True
    ).start()

    print("[REMOTE] Buka dari HP: http://IP_CORAL:{}".format(WEB_PORT))
    print(
        "[CTRL] Target STM bridge lokal: {}:{}"
        .format(
            HP_BRIDGE_IP,
            HP_BRIDGE_PORT
        )
    )

    threading.Thread(
        target=camera_loop,
        daemon=True
    ).start()

    print("")
    print("==========================================")
    print(" FREEDOM - LAPTOP PROCESSING + BODY + GARIS")
    print("==========================================")
    print("Camera :", MJPEG_URL)
    print("YOLO   : OFF")
    print("Motor  : Laptop -> localhost:5006 -> stm_bridge.py -> STM32")
    print("Body   : (-5,-2.80)->(-3.75,-2), kanan mirror x=0")
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

    while running:
        with camera_lock:
            fid = latest_camera_id
            src = latest_camera
            src_ts = latest_camera_ts

        if src is None or fid == last_id:
            key = cv2.waitKey(1) & 0xFF

            if key == 27 or key == ord("q"):
                break

            time.sleep(0.001)
            continue

        frame = src.copy()
        last_id = fid

        total_t0 = time.perf_counter()

        edge_binary, garis_kiri, garis_kanan = detect_boundaries(frame)

        frame_hasil = frame.copy()


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
        # EKSEKUSI KE STM32
        # ====================================================
        rpm_kiri, rpm_kanan = steering_to_rpm(steering)

        # Fail-safe utama:
        # Kalau dua garis sama-sama hilang, robot STOP.
        no_boundary = (
            clearance_kiri is None and
            clearance_kanan is None
        )

        with robot_lock:
            enabled_now = robot_enabled

        if no_boundary:
            rpm_kiri = 0
            rpm_kanan = 0

            status_jalan = "GARIS HILANG - FAILSAFE STOP"
            perintah_robot = "STOP"
            warna_status = (0, 0, 255)

            control_send_rpm(
                0,
                0,
                force=True
            )

        elif enabled_now:
            control_send_rpm(
                rpm_kiri,
                rpm_kanan
            )

        else:
            # Remote OFF -> motor selalu 0.
            rpm_kiri = 0
            rpm_kanan = 0

            control_send_rpm(
                0,
                0
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
                "[STEER {:+.1f}] [RPM L:{} R:{}]"
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
            "RPM L:{:02d}  R:{:02d}".format(rpm_kiri, rpm_kanan),
            (20, 148),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (255, 255, 255),
            2
        )

        cv2.putText(
            frame_hasil,
            "REMOTE: {}".format(
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
            "AGE {:.1f}ms | YOLO OFF | LAPTOP PROCESSING".format(frame_age_ms),
            (20, 225),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.43,
            (255, 255, 0),
            1
        )

        cv2.imshow(WINDOW_NAME, frame_hasil)

        key = cv2.waitKey(1) & 0xFF

        if key == 27 or key == ord("q"):
            break

    stop_all()

    try:
        cv2.destroyAllWindows()
    except Exception:
        pass


if __name__ == "__main__":
    main()
<!doctype html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Freedom Robot Remote</title>
<style>
body{
    font-family:Arial;
    background:#111;
    color:white;
    text-align:center;
    margin:0;
    padding:30px
}
h1{font-size:28px}
.status{font-size:24px;margin:20px}
button{
    width:80%;
    max-width:420px;
    height:90px;
    font-size:30px;
    font-weight:bold;
    border:0;
    border-radius:18px;
    margin:12px
}
.start{background:#18c964;color:white}
.stop{background:#f31260;color:white}
</style>
</head>
<body>
<h1>FREEDOM ROBOT REMOTE</h1>
<div class="status">Status: <b id="st">OFF</b></div>
<button class="start" onclick="cmd('/start')">START</button><br>
<button class="stop" onclick="cmd('/stop')">STOP</button>

<script>
async function cmd(url){
    const r = await fetch(url,{method:'POST'});
    document.getElementById('st').innerText = await r.text();
}
async function refresh(){
    try{
        const r = await fetch('/status');
        document.getElementById('st').innerText = await r.text();
    }catch(e){}
}
setInterval(refresh,500);
refresh();
</script>
</body>
</html>
"""



@app.route("/")
def web_home():
    return Response(
        HTML_REMOTE,
        mimetype="text/html"
    )


@app.route("/start", methods=["POST"])
def web_start():
    global robot_enabled

    with robot_lock:
        robot_enabled = True

    print("[REMOTE] START")
    return "ON"


@app.route("/stop", methods=["POST"])
def web_stop():
    global robot_enabled

    with robot_lock:
        robot_enabled = False

    control_send_rpm(
        0,
        0,
        force=True
    )

    print("[REMOTE] STOP")
    return "OFF"


@app.route("/status")
def web_status():
    with robot_lock:
        return "ON" if robot_enabled else "OFF"


def start_web_server():
    app.run(
        host="0.0.0.0",
        port=WEB_PORT,
        debug=False,
        use_reloader=False,
        threaded=True
    )

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
    if steering >= 70.0:
        return RPM_TAJAM, 0

    if steering <= -70.0:
        return 0, RPM_TAJAM

    if steering > 8.0:
        return RPM_NORMAL, RPM_KOREKSI

    if steering < -8.0:
        return RPM_KOREKSI, RPM_NORMAL

    return RPM_NORMAL, RPM_NORMAL


# ============================================================
# CAMERA
# ============================================================

def stop_all(*_):
    global running
    running = False

    # Fail-safe: setiap program berhenti, motor STOP.
    try:
        control_send_rpm(
            0,
            0,
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


# ============================================================
# MAIN
# ============================================================

def main():
    global processing_fps, processing_ms, frame_age_ms

    signal.signal(signal.SIGINT, stop_all)
    signal.signal(signal.SIGTERM, stop_all)

    # stm_bridge.py lokal akan dihubungkan otomatis saat perintah motor dikirim.
    # Jangan blok startup kamera/display karena port kontrol HP.
    threading.Thread(
        target=start_web_server,
        daemon=True
    ).start()

    print("[REMOTE] Buka dari HP: http://IP_CORAL:{}".format(WEB_PORT))
    print(
        "[CTRL] Target STM bridge lokal: {}:{}"
        .format(
            HP_BRIDGE_IP,
            HP_BRIDGE_PORT
        )
    )

    threading.Thread(
        target=camera_loop,
        daemon=True
    ).start()

    print("")
    print("==========================================")
    print(" FREEDOM - LAPTOP PROCESSING + BODY + GARIS")
    print("==========================================")
    print("Camera :", MJPEG_URL)
    print("YOLO   : OFF")
    print("Motor  : Laptop -> localhost:5006 -> stm_bridge.py -> STM32")
    print("Body   : (-5,-2.80)->(-3.75,-2), kanan mirror x=0")
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

    while running:
        with camera_lock:
            fid = latest_camera_id
            src = latest_camera
            src_ts = latest_camera_ts

        if src is None or fid == last_id:
            key = cv2.waitKey(1) & 0xFF

            if key == 27 or key == ord("q"):
                break

            time.sleep(0.001)
            continue

        frame = src.copy()
        last_id = fid

        total_t0 = time.perf_counter()

        edge_binary, garis_kiri, garis_kanan = detect_boundaries(frame)

        frame_hasil = frame.copy()


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
        # EKSEKUSI KE STM32
        # ====================================================
        rpm_kiri, rpm_kanan = steering_to_rpm(steering)

        # Fail-safe utama:
        # Kalau dua garis sama-sama hilang, robot STOP.
        no_boundary = (
            clearance_kiri is None and
            clearance_kanan is None
        )

        with robot_lock:
            enabled_now = robot_enabled

        if no_boundary:
            rpm_kiri = 0
            rpm_kanan = 0

            status_jalan = "GARIS HILANG - FAILSAFE STOP"
            perintah_robot = "STOP"
            warna_status = (0, 0, 255)

            control_send_rpm(
                0,
                0,
                force=True
            )

        elif enabled_now:
            control_send_rpm(
                rpm_kiri,
                rpm_kanan
            )

        else:
            # Remote OFF -> motor selalu 0.
            rpm_kiri = 0
            rpm_kanan = 0

            control_send_rpm(
                0,
                0
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
                "[STEER {:+.1f}] [RPM L:{} R:{}]"
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
            "RPM L:{:02d}  R:{:02d}".format(rpm_kiri, rpm_kanan),
            (20, 148),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (255, 255, 255),
            2
        )

        cv2.putText(
            frame_hasil,
            "REMOTE: {}".format(
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
            "AGE {:.1f}ms | YOLO OFF | LAPTOP PROCESSING".format(frame_age_ms),
            (20, 225),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.43,
            (255, 255, 0),
            1
        )

        cv2.imshow(WINDOW_NAME, frame_hasil)

        key = cv2.waitKey(1) & 0xFF

        if key == 27 or key == ord("q"):
            break

    stop_all()

    try:
        cv2.destroyAllWindows()
    except Exception:
        pass


if __name__ == "__main__":
    main()
