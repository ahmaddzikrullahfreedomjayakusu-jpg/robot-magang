#!/bin/bash

exec env -i \
    HOME="$HOME" \
    USER="$USER" \
    LOGNAME="$LOGNAME" \
    SHELL=/bin/bash \
    DISPLAY="${DISPLAY:-:0}" \
    XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}" \
    XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}" \
    XDG_SESSION_TYPE="${XDG_SESSION_TYPE:-x11}" \
    QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}" \
    PATH="/opt/ros/jazzy/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin" \
    AMENT_PREFIX_PATH="/opt/ros/jazzy" \
    PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages" \
    LD_LIBRARY_PATH="/opt/ros/jazzy/opt/rviz_ogre_vendor/lib:/opt/ros/jazzy/opt/gz_math_vendor/lib:/opt/ros/jazzy/opt/gz_utils_vendor/lib:/opt/ros/jazzy/opt/gz_cmake_vendor/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:/opt/ros/jazzy/lib" \
    /opt/ros/jazzy/bin/rviz2 -d "$HOME/smart_robot.rviz"
