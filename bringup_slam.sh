#!/usr/bin/env bash
# bringup_slam.sh — Bringup completo del robot + SLAM 2D (slam_toolbox)
#
# PREREQUISITO — instalar paquetes (una sola vez, en tu terminal):
#   sudo apt-get install -y ros-jazzy-slam-toolbox
#
# USO:
#   ./bringup_slam.sh            # modo online (crea mapa en tiempo real)
#   ./bringup_slam.sh --save     # guarda el mapa al salir (requiere interacción)
#
# El mapa se guarda en ~/maps/  tras llamar al servicio save_map o con --save.
#
set -euo pipefail

WS=/home/jcallano/ros2_ws
MAP_DIR="${HOME}/maps"

# --- Verificar que slam_toolbox está instalado ---
set +u
source /opt/ros/jazzy/setup.bash
set -u

if ! ros2 pkg list 2>/dev/null | grep -q "slam_toolbox"; then
    echo ""
    echo "  [ERROR] slam_toolbox no está instalado."
    echo ""
    echo "  Instala con:"
    echo "    sudo apt-get install -y ros-jazzy-slam-toolbox"
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

mkdir -p "${MAP_DIR}"

# --- Limpiar procesos previos ---
echo "Limpiando nodos anteriores..."
pkill -TERM -f "astra_camera_node|astra_color_node|rplidar_node|ros_robot_controller_node|joint_state_publisher|robot_state_publisher|static_transform_publisher|slam_toolbox|async_slam_toolbox" 2>/dev/null || true
sleep 2
pkill -KILL -f "astra_color_node" 2>/dev/null || true
sleep 1

# --- Lanzar bringup base (sensores + odom) ---
echo "Arrancando bringup base..."
ros2 launch jetauto_description robot_bringup.launch.py &
BASE_PID=$!

echo "Esperando 8 s para que los sensores initialicen..."
sleep 8

# --- Lanzar slam_toolbox en modo online asíncrono ---
echo "Arrancando slam_toolbox (online async)..."
ros2 launch slam_toolbox online_async_launch.py \
    use_sim_time:=false \
    slam_params_file:="${WS}/src/jetauto_description/config/slam_toolbox_params.yaml" &
SLAM_PID=$!

echo ""
echo "  SLAM 2D activo. Mueve el robot con el gamepad para explorar."
echo "  Topics:"
echo "    /map               — OccupancyGrid"
echo "    /slam_toolbox/...  — servicios para guardar mapa"
echo ""
echo "  Para guardar el mapa manualmente:"
echo "    ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \"{name: {data: '${MAP_DIR}/mi_mapa'}}\""
echo ""
echo "  Presiona Ctrl+C para detener."

# --- Esperar y limpiar al salir ---
trap "echo 'Deteniendo...'; kill ${SLAM_PID} ${BASE_PID} 2>/dev/null || true; sleep 2" INT TERM
wait ${BASE_PID}
