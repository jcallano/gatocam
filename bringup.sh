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

# Kill any stale nodes from previous runs before starting
echo "Cleaning up stale nodes..."
pkill -TERM -f "astra_camera_node|astra_color_node|rplidar_node|ros_robot_controller_node|joint_state_publisher|robot_state_publisher|static_transform_publisher" 2>/dev/null || true
sleep 2
# Force-kill anything that didn't respond to SIGTERM
pkill -KILL -f "astra_camera_node|astra_color_node" 2>/dev/null || true
sleep 1

echo "Starting unified bringup (no RViz by default)..."
ros2 launch jetauto_description robot_bringup.launch.py
