#!/usr/bin/env bash
# One-shot bringup: (RPLidar+TF+SLAM) untuk mapping, atau
# (RPLidar+TF+coverage_planner_node) untuk ngepel -- otomatis (coverage)
# atau cuma rekomendasi doang sambil robotnya didorong tangan (push). Ada
# juga varian "-live" dari coverage/push yang skip peta tersimpan sama
# sekali dan pakai SLAM hidup langsung (lihat bawah).
#
# Jalur ke STM32 beda per mode:
#   mapping  -> robot1.py, lewat HP/Android (stm_bridge.py + TCP/WiFi) --
#               dipertahankan apa adanya karena lebih praktis dipakai jalan
#               kaki manual sambil dorong robot pas mapping.
#   lainnya  -> robot1_usb.py, USB langsung dari laptop ke STM32, TIDAK
#               lewat HP sama sekali -- lihat robot1_usb.py untuk detail.
#
# coverage vs coverage-live (sama-sama motor jalan sendiri):
#   coverage      -> pakai peta yang udah disimpan (map_server + AMCL),
#                    butuh klik "2D Pose Estimate" di RViz dulu.
#   coverage-live -> TIDAK pakai peta tersimpan sama sekali -- slam_toolbox
#                    bikin peta live sambil jalan (kayak mode mapping),
#                    posisi diambil langsung dari TF map->base_footprint,
#                    petanya TIDAK pernah disimpan ke disk. Gak perlu klik
#                    apa-apa di RViz, tinggal tunggu SLAM dapet beberapa
#                    scan pertama.
#
# push / push-live -> sama seperti coverage/coverage-live, tapi
#               coverage_planner_node jalan dalam mode advisory_only:
#               state machine & label keputusan (LURUS/BELOK.../BERHENTI)
#               tetap jalan dan kelihatan di RViz, tapi TIDAK PERNAH
#               ngirim apa pun ke /motor_rpm -- buat coba-coba lihat
#               rekomendasinya dulu sebelum percaya sistemnya nyetir sendiri.
#
# Usage:
#   ./start.sh mapping         (default kalau tanpa argumen)
#   ./start.sh coverage
#   ./start.sh push
#   ./start.sh coverage-live
#   ./start.sh push-live
#
# TIDAK menyalakan RViz -- buka manual di terminal lain, config sudah jadi:
#   rviz2 -d ".../robotpel/rviz/mapping.rviz"     (mode mapping)
#   rviz2 -d ".../robotpel/rviz/coverage.rviz"    (mode coverage/push/*-live)
# Mode mapping TIDAK menyalakan stm_bridge.py -- itu jalan di HP/Android,
# nyalain manual di sana dulu. Mode lainnya tidak butuh HP sama sekali.
set -eo pipefail
# NOTE: no "set -u" -- /opt/ros/jazzy/setup.bash itself references unset
# variables, so nounset mode breaks sourcing it.

MODE="${1:-mapping}"
case "$MODE" in
    mapping|coverage|push|coverage-live|push-live) ;;
    *)
        echo "Usage: $0 [mapping|coverage|push|coverage-live|push-live]"
        exit 1
        ;;
esac

IS_LIVE_SLAM=""
if [ "$MODE" = "coverage-live" ] || [ "$MODE" = "push-live" ]; then
    IS_LIVE_SLAM="1"
fi
IS_PUSH=""
if [ "$MODE" = "push" ] || [ "$MODE" = "push-live" ]; then
    IS_PUSH="1"
fi

# ===== EDIT SESUAI ROBOT KAMU (kalibrasi LiDAR, lihat README) =====
SERIAL_PORT="/dev/ttyUSB0"
LASER_FRAME="laser"
LASER_X="0.0"
LASER_Y="0.0"
LASER_Z="0.15"
LASER_ROLL="0.0"
LASER_PITCH="0.0"
LASER_YAW="3.14159"       # LiDAR menghadap belakang robot (robot jalan maju = belakang LiDAR)
LASER_INVERTED="false"    # LiDAR terpasang terbalik (scan kiri/kanan kebalik kalau salah) -- lihat README
EXCLUDE_RADIUS_M="0.35"        # lingkaran diameter 70cm dari titik tengah body (LiDAR di laser_x=0,laser_y=0 = titik tengah) -- badan robot sendiri (roda, laptop) diabaikan
EXCLUDE_ANGLE_MIN_DEG="0.0"    # opsional: sektor sudut tetap tambahan yang dibuang; min==max = nonaktif
EXCLUDE_ANGLE_MAX_DEG="0.0"
MAP_YAML="/home/freedom/Documents/Robot magang/robotpel/maps/room.yaml"  # dipakai kalau MODE=coverage atau push
STM32_USB_PORT=""  # kosong = auto-detect (pilih port USB selain LiDAR). Isi manual cuma kalau auto-detect salah pilih.
# ====================================================================

ROBOT1_PY="/home/freedom/Documents/Robot magang/robot1.py"
ROBOT1_USB_PY="/home/freedom/Documents/Robot magang/robot1_usb.py"
HP_PORT=8888
HP_CANDIDATES=("192.168.0.147" "192.168.0.148" "192.168.0.149")

if { [ "$MODE" = "coverage" ] || [ "$MODE" = "push" ]; } && [ ! -f "$MAP_YAML" ]; then
    echo "[ERROR] Map tidak ditemukan: $MAP_YAML"
    echo "        Jalankan './start.sh mapping' dulu dan simpan map-nya (lihat README bagian 4)."
    exit 1
fi

STALE_PATTERN="lib/robotpel/coverage_planner_node|lib/robotpel/cmd_vel_to_motor_bridge|lib/robotpel/scan_blind_spot_filter|opt/ros/jazzy/lib/nav2_|opt/ros/jazzy/lib/rplidar_ros|opt/ros/jazzy/lib/slam_toolbox|opt/ros/jazzy/lib/tf2_ros/static_transform_publisher|python3 .*robot1\.py|python3 .*robot1_usb\.py"
stale_pids=$(pgrep -f -- "$STALE_PATTERN" || true)
if [ -n "$stale_pids" ]; then
    echo "[CLEANUP] Ada sisa proses dari sesi sebelumnya yang belum mati bersih, dimatikan dulu:"
    echo "$stale_pids" | xargs -r ps -o pid,cmd -p 2>/dev/null
    echo "$stale_pids" | xargs -r kill -TERM 2>/dev/null || true
    sleep 1
    echo "$stale_pids" | xargs -r kill -KILL 2>/dev/null || true
fi

if [ "$MODE" = "mapping" ]; then
    echo "[CHECK] stm_bridge.py harus sudah jalan manual di HP/Android."
    hp_ok=""
    for ip in "${HP_CANDIDATES[@]}"; do
        if timeout 1 bash -c "echo > /dev/tcp/${ip}/${HP_PORT}" 2>/dev/null; then
            echo "[OK] HP bridge terjangkau di ${ip}:${HP_PORT}"
            hp_ok=1
            break
        fi
    done
    if [ -z "$hp_ok" ]; then
        echo "[WARN] Tidak ada HP bridge yang terjangkau di ${HP_CANDIDATES[*]}:${HP_PORT}"
        echo "       robot1.py tetap dijalankan, dia akan terus coba reconnect sendiri."
    fi
else
    echo "[CHECK] Mode $MODE: pastikan kabel USB dari laptop ke STM32 sudah kepasang."
    echo "        Port dipilih otomatis (robot1_usb.py), nggak perlu diisi manual."
    if [ -n "$IS_PUSH" ]; then
        echo "        (advisory_only: robot TIDAK akan nyetir sendiri, dorong pakai tangan"
        echo "        -- ini cuma buat baca odometry/encoder + lihat rekomendasi di RViz.)"
    fi
    if [ -n "$IS_LIVE_SLAM" ]; then
        echo "        (live SLAM: peta tersimpan TIDAK dipakai, dibangun ulang dari nol tiap"
        echo "        run dan TIDAK disimpan -- tunggu beberapa detik biar slam_toolbox dapet"
        echo "        scan pertama sebelum coverage_planner_node mulai jalan.)"
    fi
fi

source /opt/ros/jazzy/setup.bash
if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
    source "$HOME/ros2_ws/install/setup.bash"
else
    echo "[ERROR] $HOME/ros2_ws/install/setup.bash tidak ada. Sudah colcon build?"
    exit 1
fi

# Each background job runs via `setsid` in its own process group, so cleanup
# can kill the WHOLE tree (ros2 launch + every node it spawns) with one
# `kill -- -$pid`. Killing only the tracked pid itself (plain `kill "$pid"`)
# only stops the `ros2 launch` wrapper -- its child node processes
# (coverage_planner_node, amcl, etc.) can survive as orphans and keep running
# a second, conflicting copy the next time this script starts.
pids=()
cleanup() {
    echo ""
    echo "[STOP] Mematikan semua proses (termasuk semua node turunannya)..."
    for pid in "${pids[@]}"; do
        kill -TERM -- "-$pid" 2>/dev/null || true
    done
    sleep 2
    for pid in "${pids[@]}"; do
        kill -KILL -- "-$pid" 2>/dev/null || true
    done
    wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

if [ "$MODE" = "mapping" ]; then
    echo "[START] robot1.py (via HP)"
    setsid python3 "$ROBOT1_PY" &
else
    echo "[START] robot1_usb.py (USB langsung ke STM32, port auto-detect -- lihat log-nya buat port yang kepilih)"
    setsid env STM32_USB_PORT="$STM32_USB_PORT" python3 "$ROBOT1_USB_PY" &
fi
pids+=("$!")
sleep 2

LIDAR_ARGS=(
    serial_port:="$SERIAL_PORT"
    laser_frame:="$LASER_FRAME"
    laser_x:="$LASER_X" laser_y:="$LASER_Y" laser_z:="$LASER_Z"
    laser_roll:="$LASER_ROLL" laser_pitch:="$LASER_PITCH" laser_yaw:="$LASER_YAW"
    laser_inverted:="$LASER_INVERTED"
    exclude_radius_m:="$EXCLUDE_RADIUS_M"
    exclude_angle_min_deg:="$EXCLUDE_ANGLE_MIN_DEG" exclude_angle_max_deg:="$EXCLUDE_ANGLE_MAX_DEG"
)

if [ "$MODE" = "mapping" ]; then
    echo "[START] mapping_launch.py (RPLidar + TF LiDAR + slam_toolbox)"
    setsid ros2 launch robotpel mapping_launch.py "${LIDAR_ARGS[@]}" &
    pids+=("$!")
    RVIZ_CFG="mapping.rviz"
else
    ADVISORY_ONLY="false"
    if [ -n "$IS_PUSH" ]; then
        ADVISORY_ONLY="true"
    fi
    if [ -n "$IS_LIVE_SLAM" ]; then
        echo "[START] coverage_live_slam_launch.py (RPLidar + TF LiDAR + slam_toolbox (LIVE, gak disimpan) + coverage_planner_node, advisory_only=$ADVISORY_ONLY)"
        setsid ros2 launch robotpel coverage_live_slam_launch.py advisory_only:="$ADVISORY_ONLY" "${LIDAR_ARGS[@]}" &
    else
        echo "[START] coverage_launch.py (RPLidar + TF LiDAR + AMCL + coverage_planner_node, advisory_only=$ADVISORY_ONLY)"
        setsid ros2 launch robotpel coverage_launch.py map:="$MAP_YAML" advisory_only:="$ADVISORY_ONLY" "${LIDAR_ARGS[@]}" &
    fi
    pids+=("$!")
    RVIZ_CFG="coverage.rviz"
fi

echo ""
echo "Semua jalan (mode: $MODE). Buka RViz terpisah (config sudah jadi):"
echo "  rviz2 -d \"/home/freedom/Documents/Robot magang/robotpel/rviz/${RVIZ_CFG}\""
echo ""
if { [ "$MODE" = "coverage" ] || [ "$MODE" = "push" ]; }; then
    echo "Di RViz, klik '2D Pose Estimate' sekali di posisi awal robot yang sebenarnya."
fi
if [ -n "$IS_LIVE_SLAM" ]; then
    echo "Mode live SLAM: gak perlu klik apa-apa buat lokalisasi, tunggu aja beberapa detik"
    echo "sampai slam_toolbox dapet scan pertama (lihat log 'Waiting for the map->base_footprint TF')."
fi
if [ -n "$IS_PUSH" ]; then
    echo "Mode push: dorong robotnya pakai tangan, perhatikan teks di atas robot di RViz"
    echo "(LURUS/BELOK KIRI/BELOK KANAN/dst) -- itu cuma rekomendasi, motor TIDAK jalan sendiri."
fi
echo "Ctrl+C di sini untuk berhenti."
wait
