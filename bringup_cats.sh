#!/usr/bin/env bash
# bringup_cats.sh — Detección de gatos (YOLOv8 NPU) + seguimiento autónomo
#
# Lanza:
#   1. Bringup base (sensores + odom)
#   2. Nodo YOLO RKNN (NPU Rockchip) → /yolo/detections
#   3. Nodo cat_follower → /cmd_vel (sigue el gato detectado)
#
# PREREQUISITO:
#   - Modelo yolov8n.rknn compilado y copiado a:
#     src/jetauto_description/scripts/yolov8n.rknn
#   - librknnrt.so en /usr/lib/
#   - rknn-toolkit-lite2 instalado en .venv/
#
# MODO SIN SEGUIMIENTO (solo detección, control manual):
#   ./bringup_cats.sh --detect-only
#
set -euo pipefail

WS=/home/jcallano/ros2_ws
FOLLOW=true
[[ "${1:-}" == "--detect-only" ]] && FOLLOW=false

# --- Verificar modelo RKNN ---
MODEL="${WS}/src/jetauto_description/scripts/yolov8n.rknn"
if [ ! -f "${MODEL}" ]; then
    echo ""
    echo "  [ERROR] Modelo RKNN no encontrado: ${MODEL}"
    echo ""
    echo "  Compila el modelo en un PC x86 con rknn-toolkit2:"
    echo "    from rknn.api import RKNN"
    echo "    rknn = RKNN()"
    echo "    rknn.load_onnx(model='yolov8n.onnx')"
    echo "    rknn.build(do_quantization=True, dataset='dataset.txt')"
    echo "    rknn.export_rknn('yolov8n.rknn')"
    echo "  Luego cópialo al robot:"
    echo "    scp yolov8n.rknn jcallano@robot:${MODEL}"
    echo ""
    exit 1
fi

# --- Entorno ---
set +u
source /opt/ros/jazzy/setup.bash
source "${WS}/install/setup.bash"
set -u

export PYTHONPATH="${WS}/.venv/lib/python3.12/site-packages:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="${WS}/libuvc_install/lib:${LD_LIBRARY_PATH:-}"
export LIDAR_TYPE=A1
export MACHINE_TYPE=JetAuto
export DEPTH_CAMERA_TYPE=Astra

# --- Limpiar procesos previos ---
echo "Limpiando nodos anteriores..."
pkill -TERM -f "astra_camera_node|astra_color_node|rplidar_node|ros_robot_controller_node|joint_state_publisher|robot_state_publisher|static_transform_publisher|jetauto_yolo_rknn|cat_follower" 2>/dev/null || true
sleep 2
pkill -KILL -f "astra_color_node" 2>/dev/null || true
sleep 1

# --- Bringup base ---
echo "Arrancando bringup base..."
ros2 launch jetauto_description robot_bringup.launch.py &
BASE_PID=$!

echo "Esperando 8 s para que los sensores y odom initialicen..."
sleep 8

# --- Lanzar YOLO NPU ---
echo "Arrancando YOLOv8 en la NPU..."
ros2 run jetauto_description jetauto_yolo_rknn.py &
YOLO_PID=$!

sleep 2

# --- Lanzar seguidor de gatos (o solo informar) ---
if [ "${FOLLOW}" = "true" ]; then
    echo "Arrancando cat_follower (modo seguimiento autónomo)..."
    ros2 run jetauto_description cat_follower_node.py &
    FOLLOW_PID=$!
    echo ""
    echo "  MODO: Detección + Seguimiento autónomo"
    echo "  El robot seguirá al gato más grande en pantalla."
    echo ""
    echo "  Para parar el seguimiento sin detener todo:"
    echo "    pkill -TERM -f cat_follower_node"
    echo ""
    echo "  Para retomar el control manual:"
    echo "    pkill -TERM -f cat_follower_node"
    echo "    python3 ${WS}/src/jetauto_description/scripts/jetauto_teleop_joy.py"
else
    echo ""
    echo "  MODO: Solo detección (control manual activo)"
    echo "  Detecciones en: /yolo/detections"
    echo ""
    echo "  Para activar el seguimiento en otra terminal:"
    echo "    ros2 run jetauto_description cat_follower_node.py"
fi

echo "  Topics:"
echo "    /yolo/detections   — vision_msgs/Detection2DArray"
echo "    /cmd_vel           — geometry_msgs/Twist (si seguimiento activo)"
echo ""
echo "  Para visualizar con RViz:"
echo "    rviz2 -d ${WS}/src/jetauto_description/rviz/sensors.rviz"
echo ""
echo "  Presiona Ctrl+C para detener todo."

if [ "${FOLLOW}" = "true" ]; then
    trap "echo 'Deteniendo...'; kill ${FOLLOW_PID} ${YOLO_PID} ${BASE_PID} 2>/dev/null || true; sleep 2" INT TERM
else
    trap "echo 'Deteniendo...'; kill ${YOLO_PID} ${BASE_PID} 2>/dev/null || true; sleep 2" INT TERM
fi

wait ${BASE_PID}
