#!/usr/bin/env bash
set -e

PEL_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$PEL_DIR/ros2_ws"
PKG_SRC="$PEL_DIR/mopping_nav"

echo "[PEL] Menyiapkan workspace ROS 2 lokal..."
mkdir -p "$WS_DIR/src"

if [ ! -e "$WS_DIR/src/mopping_nav" ]; then
  ln -s "$PKG_SRC" "$WS_DIR/src/mopping_nav"
fi

if [ -f "/opt/ros/jazzy/setup.bash" ]; then
  source /opt/ros/jazzy/setup.bash
elif [ -f "/opt/ros/humble/setup.bash" ]; then
  source /opt/ros/humble/setup.bash
elif [ -f "/opt/ros/iron/setup.bash" ]; then
  source /opt/ros/iron/setup.bash
else
  echo "[PEL] ERROR: ROS 2 belum ditemukan di /opt/ros."
  echo "[PEL] Install/source ROS 2 dulu, lalu jalankan ulang script ini."
  exit 1
fi

echo "[PEL] Build paket mopping_nav..."
cd "$WS_DIR"
colcon build --packages-select mopping_nav

source "$WS_DIR/install/setup.bash"

echo "[PEL] Menjalankan Autonomous Mopping Navigation..."
echo "[PEL] Pastikan topic /map, /scan, /odom, /tf sudah aktif."
echo "[PEL] Tekan Ctrl+C untuk berhenti."
ros2 launch mopping_nav mopping_navigation.launch.py
