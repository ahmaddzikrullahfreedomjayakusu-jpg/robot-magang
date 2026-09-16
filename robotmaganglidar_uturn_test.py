#!/usr/bin/env python3
"""
UTURN CALIBRATION ONLY

HP buka web dari laptop, tekan START untuk tes putar balik.
Script ini tidak menjalankan kamera, YOLO, LiDAR, edge road, atau autonomous.
Tujuannya hanya mencari durasi putar balik yang pas.
"""

import json
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


BUILD_ID = "UTURN_WEB_CALIBRATION_ONLY_2026_09_07"

# Laptop -> HP TCP bridge -> STM32
CONTROL_HP_IP = "192.168.0.149"
CONTROL_TCP_PORT = 8888

# Web remote dibuka dari HP: http://IP_LAPTOP:8899
WEB_HOST = "0.0.0.0"
WEB_PORT = 8899

# Motor command:
# 97  = maju
# 127 = stop
# 157 = mundur
MOTOR_NEUTRAL = 127
MOTOR_FORWARD = 97
MOTOR_REVERSE = 157

SEND_INTERVAL_SEC = 0.05

DEFAULT_PRE_STOP_SEC = 0.25
DEFAULT_DIRECTION = "LEFT"

sock = None
sock_lock = threading.Lock()

state_lock = threading.Lock()
running = True
spin_active = False
spin_direction = DEFAULT_DIRECTION
spin_started_at = 0.0
spin_finished_at = 0.0
spin_elapsed = 0.0
spin_last_result = "IDLE"

pre_stop_sec = DEFAULT_PRE_STOP_SEC


def make_packet(kiri, kanan):
    kiri = max(0, min(255, int(round(kiri))))
    kanan = max(0, min(255, int(round(kanan))))
    checksum = (0xAA + kanan + kiri) & 0xFF
    return bytes([0xAA, kanan, kiri, checksum, 0x55])


def control_connect():
    global sock

    with sock_lock:
        if sock is not None:
            return True

        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            s.settimeout(1.0)
            s.connect((CONTROL_HP_IP, CONTROL_TCP_PORT))
            s.settimeout(0.3)
            sock = s
            print(
                "[CTRL] Connected {}:{}".format(
                    CONTROL_HP_IP,
                    CONTROL_TCP_PORT
                ),
                flush=True
            )
            return True
        except Exception as e:
            sock = None
            print("[CTRL] Connect failed:", e, flush=True)
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
    global sock

    if not control_connect():
        return False

    pkt = make_packet(rpm_kiri, rpm_kanan)

    with sock_lock:
        try:
            sock.sendall(pkt)
            print(
                "[SEND] L:{:03d} R:{:03d}".format(
                    int(round(rpm_kiri)),
                    int(round(rpm_kanan))
                ),
                flush=True
            )
            return True
        except Exception as e:
            print("[CTRL] Send failed:", e, flush=True)
            try:
                sock.close()
            except Exception:
                pass
            sock = None
            return False


def motor_stop():
    return MOTOR_NEUTRAL, MOTOR_NEUTRAL


def motor_spin(direction):
    if direction == "RIGHT":
        return MOTOR_FORWARD, MOTOR_REVERSE
    return MOTOR_REVERSE, MOTOR_FORWARD


def start_spin():
    global spin_active
    global spin_started_at
    global spin_finished_at
    global spin_elapsed
    global spin_last_result

    with state_lock:
        spin_active = True
        spin_started_at = time.monotonic()
        spin_finished_at = 0.0
        spin_elapsed = 0.0
        spin_last_result = "PRE-STOP"

    control_send_rpm(*motor_stop(), force=True)
    print(
        "[UTURN] START HOLD dir={} pre-stop={:.2f}s | tekan STOP untuk berhenti".format(
            spin_direction,
            pre_stop_sec
        ),
        flush=True
    )


def stop_spin(reason="STOP"):
    global spin_active
    global spin_finished_at
    global spin_last_result

    with state_lock:
        spin_active = False
        spin_finished_at = time.monotonic()
        spin_last_result = reason

    control_send_rpm(*motor_stop(), force=True)
    print("[UTURN] {}".format(reason), flush=True)


def spin_loop():
    global spin_elapsed
    global spin_last_result

    last_send = 0.0

    while running:
        now = time.monotonic()

        with state_lock:
            active = spin_active
            started = spin_started_at
            direction = spin_direction
            pre_stop = pre_stop_sec

        if not active:
            time.sleep(0.03)
            continue

        elapsed = now - started
        spin_elapsed = elapsed

        if elapsed < pre_stop:
            rpm = motor_stop()
            spin_last_result = "PRE-STOP {:.2f}/{:.2f}s".format(
                elapsed,
                pre_stop
            )
        else:
            spin_elapsed_only = elapsed - pre_stop
            rpm = motor_spin(direction)
            spin_last_result = "SPIN {} {:.2f}s | tekan STOP".format(
                direction,
                spin_elapsed_only
            )

        if now - last_send >= SEND_INTERVAL_SEC:
            control_send_rpm(*rpm)
            last_send = now

        time.sleep(0.01)


def get_laptop_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "127.0.0.1"


def state_json():
    with state_lock:
        data = {
            "build": BUILD_ID,
            "active": spin_active,
            "direction": spin_direction,
            "pre_stop": pre_stop_sec,
            "elapsed": spin_elapsed,
            "status": spin_last_result,
            "bridge": "{}:{}".format(CONTROL_HP_IP, CONTROL_TCP_PORT),
        }
    return data


def page_html():
    return """<!doctype html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>UTURN TEST</title>
  <style>
    body { font-family: Arial, sans-serif; margin: 0; background: #111; color: #eee; }
    main { max-width: 560px; margin: 0 auto; padding: 18px; }
    h1 { font-size: 24px; margin: 10px 0 4px; }
    .status { background: #202020; border: 1px solid #444; padding: 14px; margin: 14px 0; }
    .big { font-size: 22px; font-weight: 700; color: #64ff7a; }
    button {
      width: 100%; padding: 18px; margin: 8px 0; border: 0;
      font-size: 22px; font-weight: 700; color: white; background: #333;
    }
    .start { background: #078b2f; }
    .stop { background: #b00020; }
    .smallrow { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }
    .smallrow button { font-size: 18px; padding: 14px; }
    .left { background: #2456d8; }
    .right { background: #8a36c9; }
    .minus { background: #555; }
    .plus { background: #666; }
    code { color: #ffd166; }
  </style>
</head>
<body>
  <main>
    <h1>UTURN CALIBRATION</h1>
    <div>START membuat robot muter terus. Tekan STOP saat sudutnya pas.</div>
    <div class="status">
      <div class="big" id="status">...</div>
      <div>Direction: <code id="direction">...</code></div>
      <div>Pre-stop: <code id="prestop">...</code>s</div>
      <div>Elapsed: <code id="elapsed">...</code>s</div>
      <div>Bridge: <code id="bridge">...</code></div>
    </div>
    <button class="start" onclick="cmd('/start')">START U-TURN</button>
    <button class="stop" onclick="cmd('/stop')">STOP</button>
    <div class="smallrow">
      <button class="left" onclick="cmd('/dir?value=LEFT')">LEFT</button>
      <button class="right" onclick="cmd('/dir?value=RIGHT')">RIGHT</button>
    </div>
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
      document.getElementById('direction').textContent = s.direction;
      document.getElementById('prestop').textContent = s.pre_stop.toFixed(2);
      document.getElementById('elapsed').textContent = s.elapsed.toFixed(2);
      document.getElementById('bridge').textContent = s.bridge;
    }
    setInterval(refresh, 200);
    refresh();
  </script>
</body>
</html>
"""


class Handler(BaseHTTPRequestHandler):
    def _send(self, code, body, content_type="text/plain"):
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        if isinstance(body, str):
            body = body.encode("utf-8")
        self.wfile.write(body)

    def do_GET(self):
        if self.path == "/":
            self._send(200, page_html(), "text/html")
        elif self.path == "/state":
            self._send(
                200,
                json.dumps(state_json()),
                "application/json"
            )
        else:
            self._send(404, "not found")

    def do_POST(self):
        global spin_direction

        if self.path.startswith("/start"):
            start_spin()
            self._send(200, "ok")
            return

        if self.path.startswith("/stop"):
            stop_spin("MANUAL STOP")
            self._send(200, "ok")
            return

        if self.path.startswith("/dir?value="):
            value = self.path.split("value=", 1)[1].upper()
            if value in ("LEFT", "RIGHT"):
                with state_lock:
                    spin_direction = value
                print("[UTURN] DIR {}".format(value), flush=True)
                self._send(200, "ok")
            else:
                self._send(400, "bad direction")
            return

        self._send(404, "not found")

    def log_message(self, fmt, *args):
        return


def main():
    global running

    print("[BUILD]", BUILD_ID, flush=True)
    print("[MODE] UTURN CALIBRATION ONLY", flush=True)
    print(
        "[CTRL] Bridge {}:{}".format(
            CONTROL_HP_IP,
            CONTROL_TCP_PORT
        ),
        flush=True
    )
    print(
        "[UTURN] default dir={} pre-stop={:.2f}s | START spin terus sampai STOP".format(
            DEFAULT_DIRECTION,
            DEFAULT_PRE_STOP_SEC
        ),
        flush=True
    )

    control_connect()
    control_send_rpm(*motor_stop(), force=True)

    threading.Thread(
        target=spin_loop,
        daemon=True
    ).start()

    laptop_ip = get_laptop_ip()
    print(
        "[WEB] HP buka: http://{}:{}".format(
            laptop_ip,
            WEB_PORT
        ),
        flush=True
    )
    print("[WEB] CTRL+C untuk keluar", flush=True)

    server = ThreadingHTTPServer((WEB_HOST, WEB_PORT), Handler)

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[EXIT] stopping", flush=True)
    finally:
        running = False
        stop_spin("EXIT STOP")
        server.server_close()
        control_disconnect()


if __name__ == "__main__":
    main()
