#!/usr/bin/env bash
set -euo pipefail

# ROS setup scripts reference variables that may be unset; temporarily disable nounset.
set +u
source /opt/ros/jazzy/setup.bash
source /home/jcallano/ros2_ws/install/setup.bash
set -u

export PYTHONPATH=/home/jcallano/ros2_ws/.venv/lib/python3.12/site-packages:${PYTHONPATH:-}
export LD_LIBRARY_PATH=/home/jcallano/ros2_ws/libuvc_install/lib:${LD_LIBRARY_PATH:-}

export LIDAR_TYPE=A1
export MACHINE_TYPE=JetAuto
export DEPTH_CAMERA_TYPE=Astra

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
export ROS_LOCALHOST_ONLY=0
export FASTRTPS_DEFAULT_PROFILES_FILE=${HOME}/.ros/fastdds_unicast.xml

# ── Limpieza de procesos anteriores ──────────────────────────────────────────
echo "Cleaning up stale nodes..."
pkill -TERM -f "astra_camera_node|astra_color_node|rplidar_node|ros_robot_controller_node|joint_state_publisher|robot_state_publisher|static_transform_publisher|system_monitor" 2>/dev/null || true
sleep 2
# Force-kill los nodos que no responden a SIGTERM (exceptuando astra_camera_node por libuvc)
pkill -KILL -f "astra_color_node|system_monitor" 2>/dev/null || true
sleep 1

# ── Monitor de rendimiento (background) ──────────────────────────────────────
echo "Starting system monitor..."
ros2 run system_monitor system_monitor &
MONITOR_PID=$!

# Matar el monitor al salir (Ctrl+C o fin normal del bringup)
trap "echo 'Stopping system_monitor...'; kill ${MONITOR_PID} 2>/dev/null || true" EXIT INT TERM

# ── Bringup principal (foreground — bloquea hasta Ctrl+C) ────────────────────
echo "Starting unified bringup (no RViz by default)..."
ros2 launch jetauto_description robot_bringup.launch.py
