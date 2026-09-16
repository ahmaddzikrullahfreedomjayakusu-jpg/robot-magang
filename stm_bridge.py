import time
import socket
import threading
import serial
import serial.tools.list_ports

SERIAL_PORT = "/dev/serial/by-path/platform-xhci-hcd.1.auto-usb-0:1:1.0"
BAUD        = 115200

TCP_HOST    = "0.0.0.0"
TCP_PORT    = 5006

clients      = []
clients_lock = threading.Lock()

ser      = None
ser_lock = threading.Lock()


# ─── Serial: open & reconnect ─────────────────────────────────────────────────

def serial_open():
    global ser
    while True:
        try:
            s = serial.Serial(
                port        = SERIAL_PORT,
                baudrate    = BAUD,
                bytesize    = serial.EIGHTBITS,
                parity      = serial.PARITY_NONE,
                stopbits    = serial.STOPBITS_ONE,
                timeout     = 0,        # non-blocking read
                write_timeout = 1.0,
                xonxoff     = False,
                rtscts      = False,
                dsrdtr      = False,
            )
            with ser_lock:
                ser = s
            print("[STM32] Serial terbuka:", SERIAL_PORT)
            return
        except Exception as e:
            print(f"[STM32] Gagal buka serial: {e}, retry 2s...")
            time.sleep(2.0)


def serial_reconnect_loop():
    """Cek tiap 2 detik, reconnect kalau serial mati."""
    while True:
        time.sleep(2.0)
        with ser_lock:
            ok = ser is not None and ser.is_open
        if not ok:
            print("[STM32] Serial putus, reconnect...")
            serial_open()


# ─── Serial → semua TCP client ────────────────────────────────────────────────

def serial_read_loop():
    """Baca dari STM32, broadcast ke semua client."""
    while True:
        with ser_lock:
            s = ser

        if s is None or not s.is_open:
            time.sleep(0.05)
            continue

        try:
            data = s.read(512)
        except serial.SerialException as e:
            print(f"[STM32] Read error: {e}")
            with ser_lock:
                try:
                    ser.close()
                except:
                    pass
            time.sleep(0.1)
            continue

        if not data:
            time.sleep(0.001)
            continue

        # Broadcast ke semua client
        mati = []
        with clients_lock:
            for c in clients:
                try:
                    c["sock"].sendall(data)
                except Exception:
                    mati.append(c)

            for c in mati:
                _drop_client(c)


# ─── TCP client → serial ──────────────────────────────────────────────────────

def client_read_loop(c):
    """Satu thread per client: terima paket RPM, forward ke serial."""
    sock = c["sock"]
    buf  = bytearray()

    try:
        addr = sock.getpeername()
    except:
        addr = ("?", 0)

    while True:
        try:
            chunk = sock.recv(256)
        except Exception:
            chunk = b""

        if not chunk:
            break

        buf += chunk

        while len(buf) >= 5:
            if buf[0] != 0xAA:
                buf.pop(0)
                continue

            kanan    = buf[1]
            kiri     = buf[2]
            checksum = buf[3]
            stop     = buf[4]

            valid = (
                checksum == ((0xAA + kanan + kiri) & 0xFF) and
                stop == 0x55
            )

            if valid:
                serial_write(bytes(buf[:5]))
            else:
                print(f"[WARN] Paket invalid dari {addr[0]}: {buf[:5].hex()}")

            buf = buf[5:]

    print(f"[CLIENT DISCONNECT] {addr[0]}:{addr[1]}")
    with clients_lock:
        _drop_client(c)


def serial_write(data):
    """Tulis ke serial dengan error handling + reconnect."""
    global ser
    with ser_lock:
        s = ser

    if s is None or not s.is_open:
        return

    try:
        s.write(data)
    except serial.SerialTimeoutException:
        print("[STM32] Write timeout, skip paket ini")
    except serial.SerialException as e:
        print(f"[STM32] Write error: {e}, reconnect...")
        with ser_lock:
            try:
                ser.close()
            except:
                pass
            ser = None


# ─── Helper client ────────────────────────────────────────────────────────────

def _drop_client(c):
    """Tutup dan hapus client dari list. Harus dipanggil dalam clients_lock."""
    try:
        c["sock"].close()
    except:
        pass
    if c in clients:
        clients.remove(c)


def accept_clients(server):
    while True:
        client, addr = server.accept()
        client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        entry = {"sock": client, "buf": bytearray()}
        with clients_lock:
            clients.append(entry)
        print(f"[CLIENT CONNECT] {addr[0]}:{addr[1]}")
        threading.Thread(target=client_read_loop, args=(entry,), daemon=True).start()


# ─── Main ─────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    print("======================================")
    print(" STM32 CDC BRIDGE  (pyserial)")
    print("======================================")
    print("TCP Port :", TCP_PORT)
    print("Serial   :", SERIAL_PORT)
    print()

    serial_open()
    threading.Thread(target=serial_reconnect_loop, daemon=True).start()
    threading.Thread(target=serial_read_loop,      daemon=True).start()

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((TCP_HOST, TCP_PORT))
    server.listen(10)
    print("Menunggu client...\n")

    try:
        accept_clients(server)
    except KeyboardInterrupt:
        print("\n[SYSTEM] STOP")
    finally:
        with clients_lock:
            for c in clients:
                try:
                    c["sock"].close()
                except:
                    pass
        with ser_lock:
            if ser and ser.is_open:
                ser.close()
        server.close()
        print("[STM32] Serial tutup")
        print("[SERVER] OFF")