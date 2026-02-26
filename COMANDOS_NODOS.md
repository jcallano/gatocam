# Comandos de Inicio de Nodos — JetAuto ROS 2

> Todos los comandos asumen que se ejecutan desde una terminal en el robot (Orange Pi 5).
> El **entorno base** debe estar cargado antes de lanzar cualquier nodo.

---

## Entorno Base (ejecutar siempre primero)

```bash
source /opt/ros/jazzy/setup.bash
source /home/jcallano/ros2_ws/install/setup.bash
export PYTHONPATH=/home/jcallano/ros2_ws/.venv/lib/python3.12/site-packages:${PYTHONPATH:-}
export LD_LIBRARY_PATH=/home/jcallano/ros2_ws/libuvc_install/lib:${LD_LIBRARY_PATH:-}
export LIDAR_TYPE=A1
export MACHINE_TYPE=JetAuto
export DEPTH_CAMERA_TYPE=Astra
```

---

## 0. Bringup Unificado (forma recomendada)

Lanza todos los nodos de una sola vez con limpieza automática de procesos anteriores.
Incluye: STM32 bridge, TF tree, LIDAR, cámara Astra (depth+IR+color), y odometría Mecanum.

```bash
cd /home/jcallano/ros2_ws
./bringup.sh
```

Con RViz incluido:

```bash
cd /home/jcallano/ros2_ws
./bringup.sh  # luego en otra terminal:
rviz2 -d /home/jcallano/ros2_ws/src/jetauto_description/rviz/sensors.rviz
```

O directamente desde el launch:

```bash
ros2 launch jetauto_description robot_bringup.launch.py use_rviz:=true
```

---

## 1. STM32 Bridge (ros_robot_controller)

Controla motores, IMU, LED, buzzer, servos y botones vía UART.

```bash
ros2 launch ros_robot_controller ros_robot_controller.launch.py \
    port:=/dev/ttyACM0 \
    baudrate:=1000000
```

**Verificar que arrancó:**
```bash
ros2 topic echo /ros_robot_controller/battery   # debe mostrar voltaje en mV
ros2 topic echo /ros_robot_controller/imu_raw   # debe mostrar datos del MPU6050
```

> Puerto: `/dev/ttyACM0` a 1 Mbps. El nodo aplica DTR/RTS reset automáticamente para despertar el IMU.

---

## 2. Robot Description (TF Tree)

Publica el árbol de transformadas del robot (URDF + TF estático cámara).

```bash
need_compile=True ros2 launch jetauto_description robot_description.launch.py \
    use_gui:=false \
    use_rviz:=false \
    use_sim_time:=false
```

**Verificar que arrancó:**
```bash
ros2 topic echo /robot_description --once   # debe mostrar el XML del URDF
ros2 run tf2_ros tf2_echo base_footprint lidar_frame   # debe mostrar transformada
```

> Requiere las variables de entorno `LIDAR_TYPE`, `MACHINE_TYPE` y `DEPTH_CAMERA_TYPE` (incluidas en el entorno base).

---

## 3. RPLidar A1

Publica el escaneo láser 2D.

```bash
ros2 launch rplidar_ros rplidar_a1_launch.py \
    serial_port:=/dev/ttyUSB0 \
    frame_id:=lidar_frame
```

**Verificar que arrancó:**
```bash
ros2 topic hz /scan   # debe mostrar ~7 Hz (normal para A1 por USB)
```

> Si lanza `SL_RESULT_OPERATION_TIMEOUT`, probablemente hay un proceso `rplidar_node` zombie:
> ```bash
> pkill -9 -f rplidar_node && sleep 1
> ```

---

## 4. Astra Camera — Depth + IR + PointCloud

Publica profundidad, infrarrojos y nube de puntos via OpenNI/libuvc.

```bash
ros2 launch astra_camera astra_pro.launch.xml \
    enable_color:=false \
    use_uvc_camera:=false \
    depth_width:=640 \
    depth_height:=480 \
    depth_fps:=15 \
    enable_ir:=true \
    ir_width:=640 \
    ir_height:=480 \
    ir_fps:=30 \
    enable_point_cloud:=true
```

**Verificar que arrancó:**
```bash
ros2 topic hz /camera/depth/image_raw   # ~12-30 Hz
ros2 topic hz /camera/ir/image_raw      # ~30 Hz
ros2 topic hz /camera/depth/points      # ~20 Hz
```

> **NUNCA usar `kill -9`** en `astra_camera_node` — desconecta el depth sensor del bus USB
> hasta hacer replug físico. Siempre usar `pkill -TERM -f astra_camera_node && sleep 2`.

---

## 5. Astra Color RGB (astra_color_node)

Publica la imagen en color via raw V4L2 MJPG + decode hardware Rockchip.

```bash
ros2 run jetauto_description astra_color_node.py
```

Con parámetros explícitos:

```bash
ros2 run jetauto_description astra_color_node.py --ros-args \
    -p video_device:=/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0 \
    -p width:=640 \
    -p height:=480 \
    -p fps:=30 \
    -p frame_id:=camera_color_optical_frame
```

**Verificar que arrancó:**
```bash
ros2 topic hz /camera/color/image_raw              # ~15-17 Hz con depth+IR activos
ros2 topic echo /camera/color/image_raw --no-arr | grep encoding   # debe decir: bgr8
```

> El nodo reporta en el log: `JPEG decoder backend: mppjpegdec` al arrancar correctamente.

---

## 6. Odometría Mecanum (odom_publisher)

Convierte `/cmd_vel` en comandos de motor y publica odometría integrada.

```bash
ros2 run controller odom_publisher --ros-args \
    --params-file /home/jcallano/ros2_ws/src/controller/config/calibrate_params.yaml
```

**Verificar que arrancó:**
```bash
ros2 topic hz /odom_raw   # debe publicar a ~50 Hz
```

Prueba de movimiento (adelante 0.05 m/s):

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -1
```

> Requiere que el nodo STM32 (`ros_robot_controller`) esté activo primero.

---

## 7. Teleoperación con Gamepad

Mapea el joystick al movimiento del robot y al paneo de cámara.

```bash
python3 /home/jcallano/ros2_ws/src/jetauto_description/scripts/jetauto_teleop_joy.py
```

**Controles principales:**

| Control | Acción |
|---------|--------|
| Stick Izquierdo | Avanzar / Retroceder / Strafe |
| Stick Derecho (X) | Rotar |
| D-Pad Izq/Der | Paneo cámara |
| L3 (click) | Centrar cámara |
| L1 | Buzzer |
| L2 (hold) | Modo precisión (25%) |
| R2 (hold) | Modo turbo (×2) |
| START + SELECT | Emergency Stop |

> Requiere que `ros_robot_controller` esté activo. El gamepad llega via `/ros_robot_controller/joy`.

---

## 8. RViz

Visualización de sensores.

```bash
rviz2 -d /home/jcallano/ros2_ws/src/jetauto_description/rviz/sensors.rviz
```

> Si no abre por problemas de display, verificar que `DISPLAY=:0` está exportado.

---

## 9. Inferencia AI — YOLOv8 en NPU

Detecta objetos (gatos/obstáculos) usando la NPU Rockchip.

```bash
ros2 run jetauto_description jetauto_yolo_rknn.py
```

**Requisitos previos:**
- Archivo `yolov8n.rknn` compilado en PC x86 y copiado al robot
- `librknnrt.so` en `/usr/lib/`
- `rknn-toolkit-lite2` instalado en el venv

**Verificar que arrancó:**
```bash
ros2 topic hz /detections   # debe publicar detecciones de vision_msgs/Detection2DArray
```

> Suscrito a `/camera/color/image_raw`. Necesita `astra_color_node` activo.

---

## 10. SLAM 2D — slam_toolbox

Construye un mapa de ocupación 2D a partir del LIDAR mientras mueves el robot.

**Prerequisito (una sola vez):**
```bash
sudo apt-get install -y ros-jazzy-slam-toolbox
```

**Lanzar:**
```bash
cd /home/jcallano/ros2_ws
./bringup_slam.sh
```

Incluye bringup base + odom + slam_toolbox online async.

**Guardar mapa:**
```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
    "{name: {data: '/home/jcallano/maps/mapa_sala'}}"
```

**Verificar:**
```bash
ros2 topic hz /map     # ~1 Hz
ros2 topic list | grep slam_toolbox
```

---

## 11. SLAM 3D — rtabmap_ros

SLAM con fusión de sensores: LIDAR + depth Astra + RGB + odom. Genera mapa 3D.

**Prerequisito (una sola vez):**
```bash
sudo apt-get install -y ros-jazzy-rtabmap-ros
```

**Lanzar (modo mapeado):**
```bash
cd /home/jcallano/ros2_ws
./bringup_slam3d.sh
```

**Lanzar (modo localización, mapa ya guardado):**
```bash
./bringup_slam3d.sh --loc
```

**Verificar:**
```bash
ros2 topic hz /rtabmap/map        # OccupancyGrid 2D
ros2 topic hz /rtabmap/cloud_map  # Nube de puntos 3D
```

Mapa persistente en: `~/.ros/rtabmap.db`

---

## 12. Detección de Gatos — YOLOv8 NPU

Detecta gatos (y otros objetos COCO) con YOLOv8n en la NPU Rockchip.

```bash
cd /home/jcallano/ros2_ws
./bringup_cats.sh --detect-only
```

**Solo el nodo YOLO (si bringup ya está corriendo):**
```bash
ros2 run jetauto_description jetauto_yolo_rknn.py
```

**Verificar:**
```bash
ros2 topic hz /yolo/detections
ros2 topic echo /yolo/detections
```

---

## 13. Seguimiento Autónomo de Gatos

Sigue al gato detectado manteniendo distancia y centrándolo en cámara.

**Bringup completo + detección + seguimiento:**
```bash
cd /home/jcallano/ros2_ws
./bringup_cats.sh
```

**Solo el seguidor (si YOLO ya está corriendo):**
```bash
ros2 run jetauto_description cat_follower_node.py
```

**Con parámetros ajustados:**
```bash
ros2 run jetauto_description cat_follower_node.py --ros-args \
    -p target_bbox_h:=200.0 \
    -p max_linear_vel:=0.12 \
    -p kp_yaw:=0.6 \
    -p kp_dist:=0.3
```

**Parámetros principales:**

| Parámetro | Defecto | Descripción |
|-----------|---------|-------------|
| `target_bbox_h` | 180 px | Altura de bbox objetivo (≈ distancia deseada) |
| `max_linear_vel` | 0.15 m/s | Velocidad máxima de avance |
| `max_angular_vel` | 0.6 rad/s | Velocidad angular máxima |
| `kp_yaw` | 0.8 | Ganancia proporcional de giro |
| `kp_dist` | 0.4 | Ganancia proporcional de distancia |
| `stop_timeout` | 1.5 s | Tiempo sin detección para parar |
| `min_bbox_h` | 30 px | Tamaño mínimo para considerar detección |

**Parar seguimiento sin detener todo:**
```bash
pkill -TERM -f cat_follower_node
```

---

## Diagnóstico Rápido

```bash
# Ver todos los nodos activos
ros2 node list

# Ver todos los topics y sus tipos
ros2 topic list -t

# Verificar tasas de todos los topics de cámara
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw
ros2 topic hz /camera/ir/image_raw
ros2 topic hz /scan

# Estado USB de la cámara Astra
lsusb | grep -E "2bc5|05e3"

# Procesos astra activos (deben ser 0 antes de un bringup limpio)
pgrep -c astra_camera_node

# Verificar autosuspend USB cámara (debe decir "on")
cat /sys/bus/usb/devices/5-1.2/power/control
```

---

## Limpieza de Procesos

```bash
# Limpieza segura (TERM primero, luego KILL solo a los que no usan libuvc)
pkill -TERM -f "astra_camera_node|astra_color_node|rplidar_node|ros_robot_controller_node|robot_state_publisher|joint_state_publisher" 2>/dev/null || true
sleep 2
pkill -KILL -f "astra_color_node|rplidar_node|ros_robot_controller_node" 2>/dev/null || true
```

> Si el depth sensor desaparece de `lsusb` tras un `kill -9` accidental: **replug físico** del USB de la Astra.
