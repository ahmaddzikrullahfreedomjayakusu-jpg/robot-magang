#!/usr/bin/env bash
# One-shot bringup: robot1.py + (RPLidar+TF+SLAM) untuk mapping, atau
# (RPLidar+TF+AMCL+Nav2+coverage_planner_node+cmd_vel_to_motor_bridge) untuk ngepel.
#
# Usage:
#   ./start.sh mapping     (default kalau tanpa argumen)
#   ./start.sh coverage
#
# TIDAK menyalakan RViz -- buka manual di terminal lain, config sudah jadi:
#   rviz2 -d ".../robotpel/rviz/mapping.rviz"    (mode mapping)
#   rviz2 -d ".../robotpel/rviz/coverage.rviz"   (mode coverage)
# TIDAK menyalakan stm_bridge.py -- itu jalan di HP/Android, nyalain manual di sana dulu.
set -eo pipefail
# NOTE: no "set -u" -- /opt/ros/jazzy/setup.bash itself references unset
# variables, so nounset mode breaks sourcing it.

MODE="${1:-mapping}"
if [ "$MODE" != "mapping" ] && [ "$MODE" != "coverage" ]; then
    echo "Usage: $0 [mapping|coverage]"
    exit 1
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
EXCLUDE_RADIUS_M="0.25"        # buang deteksi LiDAR lebih dekat dari ini (lingkaran diameter 50cm) -- laptop/kabel
EXCLUDE_ANGLE_MIN_DEG="0.0"    # opsional: sektor sudut tetap tambahan yang dibuang; min==max = nonaktif
EXCLUDE_ANGLE_MAX_DEG="0.0"
MAP_YAML="/home/freedom/Documents/Robot magang/robotpel/maps/room.yaml"  # dipakai kalau MODE=coverage
# ====================================================================

ROBOT1_PY="/home/freedom/Documents/Robot magang/robot1.py"
HP_PORT=8888
HP_CANDIDATES=("192.168.0.147" "192.168.0.148" "192.168.0.149")

if [ "$MODE" = "coverage" ] && [ ! -f "$MAP_YAML" ]; then
    echo "[ERROR] Map tidak ditemukan: $MAP_YAML"
    echo "        Jalankan './start.sh mapping' dulu dan simpan map-nya (lihat README bagian 4)."
    exit 1
fi

STALE_PATTERN="lib/robotpel/coverage_planner_node|lib/robotpel/cmd_vel_to_motor_bridge|lib/robotpel/scan_blind_spot_filter|opt/ros/jazzy/lib/nav2_|opt/ros/jazzy/lib/rplidar_ros|opt/ros/jazzy/lib/slam_toolbox|opt/ros/jazzy/lib/tf2_ros/static_transform_publisher|python3 .*robot1\.py"
stale_pids=$(pgrep -f -- "$STALE_PATTERN" || true)
if [ -n "$stale_pids" ]; then
    echo "[CLEANUP] Ada sisa proses dari sesi sebelumnya yang belum mati bersih, dimatikan dulu:"
    echo "$stale_pids" | xargs -r ps -o pid,cmd -p 2>/dev/null
    echo "$stale_pids" | xargs -r kill -TERM 2>/dev/null || true
    sleep 1
    echo "$stale_pids" | xargs -r kill -KILL 2>/dev/null || true
fi

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

echo "[START] robot1.py"
setsid python3 "$ROBOT1_PY" &
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
    echo "[START] coverage_launch.py (RPLidar + TF LiDAR + AMCL + Nav2 + coverage_planner_node + cmd_vel_to_motor_bridge)"
    setsid ros2 launch robotpel coverage_launch.py map:="$MAP_YAML" "${LIDAR_ARGS[@]}" &
    pids+=("$!")
    RVIZ_CFG="coverage.rviz"
fi

echo ""
echo "Semua jalan (mode: $MODE). Buka RViz terpisah (config sudah jadi):"
echo "  rviz2 -d \"/home/freedom/Documents/Robot magang/robotpel/rviz/${RVIZ_CFG}\""
echo ""
if [ "$MODE" = "coverage" ]; then
    echo "Di RViz, klik '2D Pose Estimate' sekali di posisi awal robot yang sebenarnya."
fi
echo "Ctrl+C di sini untuk berhenti."
wait
