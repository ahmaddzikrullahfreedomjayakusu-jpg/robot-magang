#!/bin/bash

source /opt/ros/jazzy/setup.bash

set -u

PID_LIDAR=""
PID_ROBOT=""
PID_MEMORY=""
PID_TF1=""
PID_TF2=""
PID_SLAM=""
LIDAR_PORT=""
TIMEOUT_TOPIC=20
TIMEOUT_TF=20

cleanup()
{
    echo ""
    echo "======================================"
    echo " STOP SMART ROBOT"
    echo "======================================"

    for PID in "$PID_SLAM" "$PID_MEMORY" "$PID_ROBOT" "$PID_TF2" "$PID_TF1" "$PID_LIDAR"
    do
        if [ -n "$PID" ] && kill -0 "$PID" 2>/dev/null
        then
            kill -- "-$PID" 2>/dev/null
            kill "$PID" 2>/dev/null
        fi
    done

    sleep 1

    for PID in "$PID_SLAM" "$PID_MEMORY" "$PID_ROBOT" "$PID_TF2" "$PID_TF1" "$PID_LIDAR"
    do
        if [ -n "$PID" ] && kill -0 "$PID" 2>/dev/null
        then
            kill -9 -- "-$PID" 2>/dev/null
            kill -9 "$PID" 2>/dev/null
        fi
    done

    wait 2>/dev/null

    echo "STOPPED"
}

fail()
{
    echo "ERROR: $1"
    cleanup
    exit 1
}

process_alive()
{
    local PID="$1"
    local NAME="$2"

    if [ -z "$PID" ] || ! kill -0 "$PID" 2>/dev/null
    then
        fail "$NAME GAGAL START"
    fi
}

wait_topic()
{
    local TOPIC="$1"
    local LABEL="$2"
    local PID="$3"
    local NAME="$4"
    local TIMEOUT="$5"
    local START
    local NOW

    START=$(date +%s)

    while true
    do
        if ros2 topic list 2>/dev/null | grep -qx "$TOPIC"
        then
            echo "    $LABEL READY"
            return 0
        fi

        process_alive "$PID" "$NAME"

        NOW=$(date +%s)

        if [ $((NOW - START)) -ge "$TIMEOUT" ]
        then
            fail "$NAME TIMEOUT MENUNGGU $TOPIC"
        fi

        sleep 0.3
    done
}

wait_bool_true()
{
    local TOPIC="$1"
    local LABEL="$2"
    local PID="$3"
    local NAME="$4"
    local TIMEOUT="$5"
    local START
    local NOW

    START=$(date +%s)

    while true
    do
        if timeout 2 ros2 topic echo --once "$TOPIC" 2>/tmp/ready_check.log | grep -q "data: true"
        then
            echo "    $LABEL READY"
            return 0
        fi

        process_alive "$PID" "$NAME"

        NOW=$(date +%s)

        if [ $((NOW - START)) -ge "$TIMEOUT" ]
        then
            fail "$NAME TIMEOUT MENUNGGU DATA $TOPIC"
        fi

        sleep 0.3
    done
}

wait_transform()
{
    local TARGET_FRAME="$1"
    local SOURCE_FRAME="$2"
    local LABEL="$3"
    local PID="$4"
    local START
    local NOW

    START=$(date +%s)

    while true
    do
        timeout 2 ros2 run tf2_ros tf2_echo "$TARGET_FRAME" "$SOURCE_FRAME" >/tmp/tf_check.log 2>&1

        if grep -q "Translation:" /tmp/tf_check.log
        then
            echo "    $LABEL READY"
            return 0
        fi

        process_alive "$PID" "TF ROBOT"

        NOW=$(date +%s)

        if [ $((NOW - START)) -ge "$TIMEOUT_TF" ]
        then
            fail "TF ROBOT TIMEOUT MENUNGGU $LABEL"
        fi

        sleep 0.3
    done
}

trap 'cleanup; exit 130' SIGINT
trap 'cleanup; exit 143' SIGTERM

echo "======================================"
echo "       SMART ROBOT START"
echo "======================================"

for PORT in /dev/ttyUSB* /dev/ttyACM*
do
    [ -e "$PORT" ] || continue

    INFO=$(udevadm info --query=property --name="$PORT" 2>/dev/null)

    if echo "$INFO" | grep -Eqi "RPLIDAR|CP210|Silicon.Labs"
    then
        LIDAR_PORT="$PORT"
        break
    fi
done

if [ -z "$LIDAR_PORT" ]
then
    for PORT in /dev/ttyUSB*
    do
        if [ -e "$PORT" ]
        then
            LIDAR_PORT="$PORT"
            break
        fi
    done
fi

if [ -z "$LIDAR_PORT" ]
then
    fail "RPLIDAR TIDAK DITEMUKAN"
fi

echo "LIDAR = $LIDAR_PORT"

echo "[1] RPLIDAR"
setsid ros2 launch rplidar_ros rplidar.launch.py \
    serial_port:="$LIDAR_PORT" \
    serial_baudrate:=115200 \
    > /tmp/lidar.log 2>&1 &
PID_LIDAR=$!

wait_topic "/scan" "/scan" "$PID_LIDAR" "RPLIDAR" "$TIMEOUT_TOPIC"

echo "[2] ROBOT + ENCODER"
setsid python3 "$HOME/robot1.py" > /tmp/robot.log 2>&1 &
PID_ROBOT=$!

wait_topic "/scan_front" "/scan_front" "$PID_ROBOT" "ROBOT + ENCODER" "$TIMEOUT_TOPIC"
wait_bool_true "/encoder_ready" "ENCODER DATA" "$PID_ROBOT" "ROBOT + ENCODER" "$TIMEOUT_TOPIC"
wait_topic "/odom" "/odom" "$PID_ROBOT" "ROBOT + ENCODER" "$TIMEOUT_TOPIC"

echo "[3] TF ROBOT"
setsid ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0 \
    --yaw 0 --pitch 0 --roll 0 \
    --frame-id base_footprint \
    --child-frame-id base_link \
    > /tmp/tf_base_footprint_base_link.log 2>&1 &
PID_TF1=$!

setsid ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0 \
    --yaw 3.14159265359 --pitch 0 --roll 0 \
    --frame-id base_link \
    --child-frame-id laser \
    > /tmp/tf_base_link_laser.log 2>&1 &
PID_TF2=$!

wait_transform "odom" "base_link" "odom -> base_link" "$PID_ROBOT"
wait_transform "base_footprint" "base_link" "base_footprint -> base_link" "$PID_TF1"
wait_transform "base_link" "laser" "base_link -> laser" "$PID_TF2"

echo "[4] SMART HARD MAP MEMORY V2"
setsid python3 "$HOME/memory1.py" > /tmp/memory.log 2>&1 &
PID_MEMORY=$!

wait_topic "/smart_memory_ready" "SMART MEMORY V2" "$PID_MEMORY" "SMART MEMORY V2" "$TIMEOUT_TOPIC"
wait_topic "/scan_slam" "/scan_slam" "$PID_MEMORY" "SMART MEMORY" "$TIMEOUT_TOPIC"

cat > /tmp/smart_slam.yaml <<EOF
slam_toolbox:
  ros__parameters:
    solver_plugin: solver_plugins::CeresSolver
    odom_frame: odom
    map_frame: map
    base_frame: base_footprint
    scan_topic: /scan_slam
    mode: mapping
    resolution: 0.05
    max_laser_range: 12.0
    minimum_time_interval: 0.10
    transform_publish_period: 0.02
    map_update_interval: 1.0
    minimum_travel_distance: 0.03
    minimum_travel_heading: 0.03
    transform_timeout: 0.5
    tf_buffer_duration: 30.0
    scan_buffer_size: 40
    scan_buffer_maximum_scan_distance: 12.0
    do_loop_closing: true
    loop_search_maximum_distance: 4.0
    loop_match_minimum_chain_size: 8
    loop_match_minimum_response_coarse: 0.30
    loop_match_minimum_response_fine: 0.40
    link_scan_maximum_distance: 2.0
    link_match_minimum_response_fine: 0.10
EOF

echo "[5] SLAM"
setsid ros2 launch slam_toolbox online_async_launch.py \
    use_sim_time:=false \
    slam_params_file:=/tmp/smart_slam.yaml \
    > /tmp/slam.log 2>&1 &
PID_SLAM=$!

wait_topic "/map" "/map" "$PID_SLAM" "SLAM" "$TIMEOUT_TOPIC"

echo ""
echo "======================================"
echo "         ROBOT READY"
echo "======================================"
echo ""
echo "LIDAR       = $LIDAR_PORT"
echo "/scan       = RAW"
echo "/scan_front = FRONT ROBOT"
echo "/scan_slam  = SMART MAP INPUT"
echo "/odom       = ENCODER"
echo "/encoder_ready = DATA ENCODER MASUK"
echo "/encoder_debug = DEBUG ENCODER"
echo "/odom_trail = JEJAK ENCODER ASLI"
echo "/odom_path  = ROUTE ODOM"
echo "/known_corridor = AREA LAMA YANG DI-HARD LOCK"
echo "/map_mode   = EXPLORE / KNOWN"
echo "/map_gate_reason = ALASAN GATE MAPPING"
echo "/robot_body = PANAH 25CM"
echo "/map        = SLAM"
echo "/tf         = ROBOT TF"
echo "/tf_static  = STATIC TF"
echo ""
echo "CTRL+C = STOP"
echo "======================================"
echo ""

wait