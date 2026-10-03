#!/usr/bin/env python3
import argparse
import select
import serial
import serial.tools.list_ports
import sys
import termios
import time
import tty

NEUTRAL = 127
FORWARD = 97
REVERSE = 157
BAUD = 115200
STM32_VID_PID = (0x0483, 0x5740)


def find_stm32_port():
    for port in serial.tools.list_ports.comports():
        if port.vid == STM32_VID_PID[0] and port.pid == STM32_VID_PID[1]:
            return port.device
    return None


def make_packet(left, right):
    left = max(0, min(255, int(left)))
    right = max(0, min(255, int(right)))
    checksum = (0xAA + right + left) & 0xFF
    return bytes([0xAA, right, left, checksum, 0x55])


def send_motor(ser, left, right):
    ser.write(make_packet(left, right))
    ser.flush()


COMMANDS = {
    "UP": ("MAJU dua roda", FORWARD, FORWARD),
    "DOWN": ("MUNDUR dua roda", REVERSE, REVERSE),
    "LEFT": ("SPIN KIRI", REVERSE, FORWARD),
    "RIGHT": ("SPIN KANAN", FORWARD, REVERSE),
    "x": ("STOP", NEUTRAL, NEUTRAL),
    " ": ("STOP", NEUTRAL, NEUTRAL),
}


HELP = """
Manual motor test CDX

Keyboard:
  panah atas     maju dua roda        [97, 97]
  panah bawah    mundur dua roda      [157, 157]
  panah kiri     spin kiri            [157, 97]
  panah kanan    spin kanan           [97, 157]
  x/space stop            [127, 127]
  h  tampilkan bantuan
  Ctrl+C keluar aman

Catatan:
  Tahan tombol panah untuk jalan. Saat tombol dilepas, script otomatis
  mengirim STOP setelah timeout pendek.
"""


def read_key():
    ch = sys.stdin.read(1)
    if ch != "\x1b":
        return ch

    readable, _, _ = select.select([sys.stdin], [], [], 0.02)
    if not readable:
        return ch
    second = sys.stdin.read(1)
    if second != "[":
        return ch + second

    readable, _, _ = select.select([sys.stdin], [], [], 0.02)
    if not readable:
        return ch + second
    third = sys.stdin.read(1)
    return {
        "A": "UP",
        "B": "DOWN",
        "C": "RIGHT",
        "D": "LEFT",
    }.get(third, ch + second + third)


def main():
    parser = argparse.ArgumentParser(description="Manual serial motor test for STM32 hoverboard driver.")
    parser.add_argument("--port", default="", help="Serial port STM32, contoh: /dev/ttyACM1")
    parser.add_argument("--rate", type=float, default=20.0, help="Repeat send rate Hz")
    parser.add_argument("--release-timeout", type=float, default=0.25, help="Stop if no key repeat arrives for this many seconds")
    args = parser.parse_args()

    port = args.port or find_stm32_port()
    if not port:
        print("STM32 tidak ditemukan. Cek: python3 -m serial.tools.list_ports -v", file=sys.stderr)
        return 1

    print(f"[CONNECT] {port} @ {BAUD}")
    ser = serial.Serial(port=port, baudrate=BAUD, timeout=0.02, write_timeout=0.5)

    current_name = "STOP"
    left = NEUTRAL
    right = NEUTRAL
    period = 1.0 / max(args.rate, 1.0)
    last_send = 0.0
    last_key_time = 0.0

    old_settings = termios.tcgetattr(sys.stdin)
    print(HELP)
    print(f"[CMD] {current_name}: kiri={left} kanan={right}", flush=True)

    try:
        tty.setcbreak(sys.stdin.fileno())
        while True:
            readable, _, _ = select.select([sys.stdin], [], [], 0.02)
            if readable:
                ch = read_key()
                if ch == "\x03":
                    raise KeyboardInterrupt
                if ch == "h":
                    print(HELP)
                    continue
                if ch in COMMANDS:
                    current_name, left, right = COMMANDS[ch]
                    last_key_time = time.monotonic()
                    print(f"[CMD] {current_name}: kiri={left} kanan={right}", flush=True)

            now = time.monotonic()
            if (left, right) != (NEUTRAL, NEUTRAL) and now - last_key_time > args.release_timeout:
                current_name = "STOP"
                left = NEUTRAL
                right = NEUTRAL
                print(f"[CMD] {current_name}: kiri={left} kanan={right}", flush=True)

            if now - last_send >= period:
                send_motor(ser, left, right)
                last_send = now
    except KeyboardInterrupt:
        print("\n[EXIT] stop motor")
    finally:
        try:
            send_motor(ser, NEUTRAL, NEUTRAL)
            time.sleep(0.1)
            send_motor(ser, NEUTRAL, NEUTRAL)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            ser.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
