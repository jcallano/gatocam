#!/usr/bin/env bash
# bringup_slam3d.sh — Bringup completo del robot + SLAM 3D (rtabmap_ros)
#
# Fusiona: LIDAR 2D (/scan) + profundidad Astra (/camera/depth/image_raw)
#           + RGB (/camera/color/image_raw) + odometría (/odom_raw)
#           + nube de puntos (/camera/depth/points)
#
# PREREQUISITO — instalar paquetes (una sola vez, en tu terminal):
#   sudo apt-get install -y ros-jazzy-rtabmap-ros
#
# USO:
#   ./bringup_slam3d.sh          # construye nuevo mapa
#   ./bringup_slam3d.sh --loc    # solo localización (mapa previo en ~/.ros/rtabmap.db)
#
set -euo pipefail

WS=/home/jcallano/ros2_ws
LOCALIZATION=false
[[ "${1:-}" == "--loc" ]] && LOCALIZATION=true

# --- Verificar rtabmap_ros ---
set +u
source /opt/ros/jazzy/setup.bash
set -u

if ! ros2 pkg list 2>/dev/null | grep -q "rtabmap_ros"; then
    echo ""
    echo "  [ERROR] rtabmap_ros no está instalado."
    echo ""
    echo "  Instala con:"
    echo "    sudo apt-get install -y ros-jazzy-rtabmap-ros"
    echo ""
    exit 1
fi

# --- Entorno ---
set +u
source "${WS}/install/setup.bash"
set -u

export PYTHONPATH="${WS}/.venv/lib/python3.12/site-packages:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="${WS}/libuvc_install/lib:${LD_LIBRARY_PATH:-}"
export LIDAR_TYPE=A1
export MACHINE_TYPE=JetAuto
export DEPTH_CAMERA_TYPE=Astra

# --- Limpiar procesos previos ---
echo "Limpiando nodos anteriores..."
pkill -TERM -f "astra_camera_node|astra_color_node|rplidar_node|ros_robot_controller_node|joint_state_publisher|robot_state_publisher|static_transform_publisher|rtabmap" 2>/dev/null || true
sleep 2
pkill -KILL -f "astra_color_node" 2>/dev/null || true
sleep 1

# --- Bringup base ---
echo "Arrancando bringup base..."
ros2 launch jetauto_description robot_bringup.launch.py &
BASE_PID=$!

echo "Esperando 10 s para que los sensores y odom initialicen..."
sleep 10

# --- Lanzar rtabmap ---
echo "Arrancando rtabmap_ros (SLAM 3D)..."

RTAB_LOCALIZATION="${LOCALIZATION}"

ros2 launch rtabmap_ros rtabmap.launch.py \
    use_sim_time:=false \
    frame_id:=base_footprint \
    odom_topic:=/odom_raw \
    scan_topic:=/scan \
    depth_topic:=/camera/depth/image_raw \
    rgb_topic:=/camera/color/image_raw \
    camera_info_topic:=/camera/depth/camera_info \
    subscribe_scan:=true \
    subscribe_depth:=true \
    approx_sync:=true \
    localization:=${RTAB_LOCALIZATION} \
    Rtabmap/DetectionRate:=1 \
    Mem/STMSize:=30 \
    Kp/MaxFeatures:=400 &
RTAB_PID=$!

echo ""
if [ "${LOCALIZATION}" = "true" ]; then
    echo "  Modo LOCALIZACIÓN — usando mapa guardado en ~/.ros/rtabmap.db"
else
    echo "  Modo MAPEADO — construyendo nuevo mapa 3D."
fi
echo ""
echo "  Topics principales:"
echo "    /rtabmap/map          — OccupancyGrid 2D"
echo "    /rtabmap/mapData      — Mapa 3D completo"
echo "    /rtabmap/cloud_map    — Nube de puntos del entorno"
echo "    /rtabmap/odom         — Odometría corregida"
echo ""
echo "  Base de datos del mapa: ~/.ros/rtabmap.db"
echo "  Para exportar a PCD/PLY después:"
echo "    rtabmap-databaseViewer ~/.ros/rtabmap.db"
echo ""
echo "  Presiona Ctrl+C para detener."

trap "echo 'Deteniendo...'; kill ${RTAB_PID} ${BASE_PID} 2>/dev/null || true; sleep 2" INT TERM
wait ${BASE_PID}
