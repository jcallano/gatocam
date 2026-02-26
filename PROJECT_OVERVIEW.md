# Proyecto: Robot móvil para monitoreo de gatos (ROS 2 Jazzy)

Este documento resume el estado actual del proyecto, hardware, software, entornos Python y nodos ROS 2. Se diseñó para que otros agentes puedan retomar el trabajo sin pérdida de contexto.

## Hardware
- SBC: Orange Pi 5 (AArch64), Armbian GNOME, kernel 6.1.115, boot en SSD NVMe.
- Chasis: 4 ruedas Mecanum (cinemática omnidireccional).
- Cámara: Orbbec Astra Pro Plus (RGB + depth + point cloud).
- LIDAR: RPLidar A1/A2 (escaneo 2D).
- Control motor/batería/IMU/servos: STM32 (Hiwonder JetAuto controller) por UART.
- Middleware ROS: FastDDS (requerido por el proyecto).

## Software
- OS base: Ubuntu 24.04 (Noble) sobre Armbian.
- ROS 2: Jazzy Jalisco.
- Workspace: `/home/jcallano/ros2_ws`

## Venvs y Python
- Venv principal: `/home/jcallano/ros2_ws/.venv`
- Dependencias instaladas en venv:
  - `pyserial` (necesario para `ros_robot_controller_sdk.py`)
- Nota: El ejecutable instalado por `ros_robot_controller` usa `#!/usr/bin/python3`, por lo que el venv **no se activa automáticamente**. Para que encuentre `pyserial`, hay que exportar `PYTHONPATH`.
para activar source /home/jcallano/ros2_ws/.venv/bin/activate

Comando recomendado para lanzar el controlador STM32 usando el venv:
```
export PYTHONPATH=/home/jcallano/ros2_ws/.venv/lib/python3.12/site-packages:$PYTHONPATH
source /opt/ros/jazzy/setup.bash
source /home/jcallano/ros2_ws/install/setup.bash
ros2 launch ros_robot_controller ros_robot_controller.launch.py port:=/dev/ttyUSB0 baudrate:=1000000
```

## Drivers y librerías críticas
- `libuvc` y `libusb-1.0` compiladas manualmente para ARM64.
- `libuvc` instalada en `/home/jcallano/ros2_ws/libuvc_install`.
- Variables usadas para compilar `astra_camera`:
  - `PKG_CONFIG_PATH=/home/jcallano/ros2_ws/libuvc_install/lib/pkgconfig`
  - `LD_LIBRARY_PATH=/home/jcallano/ros2_ws/libuvc_install/lib`

## Paquetes principales en el workspace
- `ros_robot_controller` (Python)
- `ros_robot_controller_msgs`
- `controller` (mecanum + odom)
- `kinematics`, `kinematics_msgs`
- `jetauto_description` (URDF/Xacro + meshes)
- `ros2_astra_camera` (astra_camera)
- `rplidar_ros`
- `vision_opencv`, `image_common`

## Documentación de protocolo STM32
- `jetauto_function_ids_documentation.md`
- `jetauto_stm32_architecture_report.md`
- SDK Python original:
  - `RobotControllerSourceCode/ROS2/src/home/ubuntu/ros2_ws/src/driver/ros_robot_controller/ros_robot_controller/ros_robot_controller_sdk.py`
- Protocolo:
  - Cabecera: `0xAA 0x55`
  - Campos: `[FunctionID][Length][Payload][CRC8]`
  - Baudrate: `1,000,000 bps`
  - CRC8: tabla específica (definida en el SDK).

## Nodos ROS 2 y sus atributos

### ros_robot_controller (STM32 bridge)
- Ejecutable: `ros_robot_controller`
- Launch: `ros_robot_controller.launch.py`
- Parámetros:
  - `port` (default `/dev/ttyUSB0`)
  - `baudrate` (default `1000000`)
  - `timeout` (default `5.0`)
  - `imu_frame` (default `imu_link`)
  - `reset_dtr_rts` (default `True`)
  - `reset_pulse_ms` (default `100`)
  - `reset_post_ms` (default `500`)
- Topics publicados:
  - `/ros_robot_controller/imu_raw` (`sensor_msgs/Imu`)
  - `/ros_robot_controller/battery` (`std_msgs/UInt16`)
  - `/ros_robot_controller/button` (`ros_robot_controller_msgs/ButtonState`)
  - `/ros_robot_controller/joy` (`sensor_msgs/Joy`)
  - `/ros_robot_controller/sbus` (`ros_robot_controller_msgs/Sbus`)
- Topics suscritos:
  - `/ros_robot_controller/set_motor` (`ros_robot_controller_msgs/MotorsState`)
  - `/ros_robot_controller/set_led` (`ros_robot_controller_msgs/LedState`)
  - `/ros_robot_controller/set_buzzer` (`ros_robot_controller_msgs/BuzzerState`)
  - `/ros_robot_controller/set_oled` (`ros_robot_controller_msgs/OLEDState`)
  - `/ros_robot_controller/bus_servo/set_position`
  - `/ros_robot_controller/bus_servo/set_state`
  - `/ros_robot_controller/pwm_servo/set_state`
- Servicios:
  - `~/init_finish` (`std_srvs/Trigger`)
  - `~/bus_servo/get_state`
  - `~/pwm_servo/get_state`

### controller/odom_publisher (mecanum + cmd_vel bridge)
- Ejecutable: `odom_publisher`
- Parámetros relevantes:
  - `wheelbase` (default `0.216`)
  - `track_width` (default `0.195`)
  - `wheel_diameter` (default `0.097`)
  - `linear_correction_factor` (default `1.00`)
  - `angular_correction_factor` (default `1.04`)
  - `base_frame_id` (default `base_footprint`)
  - `odom_frame_id` (default `odom`)
  - `imu_frame` (default `imu_link`)
  - `pub_odom_topic` (default `True`)
- Suscribe:
  - `/cmd_vel` (`geometry_msgs/Twist`)
  - `controller/cmd_vel` (`geometry_msgs/Twist`)
- Publica:
  - `ros_robot_controller/set_motor` (`ros_robot_controller_msgs/MotorsState`)
  - `odom_raw` (`nav_msgs/Odometry`)

### astra_camera (Orbbec depth + IR)
- Launch: `ros2 launch astra_camera astra_pro.launch.xml enable_color:=false use_uvc_camera:=false`
- Node visto: `/camera/camera`
- Topics publicados:
  - `/camera/depth/image_raw`
  - `/camera/depth/camera_info`
  - `/camera/depth/points`
  - `/camera/ir/image_raw`
  - `/camera/ir/camera_info`
- Requiere `LD_LIBRARY_PATH` apuntando a `libuvc_install/lib`
- **NUNCA usar `kill -9`** — desconecta el depth sensor del bus USB hasta replug físico

### astra_color_node (RGB color, raw V4L2)
- Ejecutable: `src/jetauto_description/scripts/astra_color_node.py`
- Paquete: `jetauto_description`
- Namespace: `camera`, Name: `color`
- Parámetros:
  - `video_device` (default: symlink by-id del Sonix UVC)
  - `width` (default `640`), `height` (default `480`), `fps` (default `30`)
  - `frame_id` (default `camera_color_optical_frame`)
- Topic publicado: `/camera/color/image_raw` (`sensor_msgs/Image`, encoding `bgr8`)
- **Arquitectura:** Raw V4L2 ioctls MJPG (AArch64 88-byte struct) + GStreamer mppjpegdec (VPU Rockchip)
- **Tasas esperadas:** ~15–17 fps con depth+IR simultáneos (limitado por pipeline GStreamer bajo carga)
- Retry automático al abrir (10 intentos × 1 s)
- Ver `ASTRA_CAMERA_TROUBLESHOOTING.md` para detalles de arquitectura y quirks

### rplidar_ros
- Driver de LIDAR 2D. Ajustes típicos en launch del paquete.
- Configuración validada para alinear el frame con el frente del robot:
  - `frame_id:=lidar_frame` (usa la rotación 180° definida en el URDF).
  - Puerto: `/dev/ttyUSB0` (RPLidar A1).
  - Launch recomendado:
    - `ros2 launch rplidar_ros rplidar_a1_launch.py serial_port:=/dev/ttyUSB0 frame_id:=lidar_frame`

## Descripción del robot (simulación)
- Paquete: `jetauto_description`
- URDF/Xacro base:
  - `src/jetauto_description/urdf/jetauto_car.urdf.xacro`
- Sensores:
  - `src/jetauto_description/urdf/lidar_a1.urdf.xacro`
  - `src/jetauto_description/urdf/depth_camera.urdf.xacro`
  - `src/jetauto_description/urdf/imu.urdf.xacro`

## Ejecuciones clave (comandos)

### STM32 bridge
```
export PYTHONPATH=/home/jcallano/ros2_ws/.venv/lib/python3.12/site-packages:$PYTHONPATH
source /opt/ros/jazzy/setup.bash
source /home/jcallano/ros2_ws/install/setup.bash
ros2 launch ros_robot_controller ros_robot_controller.launch.py port:=/dev/ttyUSB0 baudrate:=1000000
```

### Robot description (TF completo)
Requiere variables de entorno:
```
export LIDAR_TYPE=A1
export MACHINE_TYPE=JetAuto
export DEPTH_CAMERA_TYPE=Astra
```
Launch:
```
source /opt/ros/jazzy/setup.bash
source /home/jcallano/ros2_ws/install/setup.bash
need_compile=True ros2 launch jetauto_description robot_description.launch.py use_gui:=false use_rviz:=false use_sim_time:=false
```
Incluye un TF estático automático `depth_cam_link -> camera_link` para enlazar la cámara con el árbol TF del robot.

### RPLidar A1
```
source /opt/ros/jazzy/setup.bash
source /home/jcallano/ros2_ws/install/setup.bash
ros2 launch rplidar_ros rplidar_a1_launch.py serial_port:=/dev/ttyUSB0 frame_id:=lidar_frame
```

### Astra camera
```
source /opt/ros/jazzy/setup.bash
source /home/jcallano/ros2_ws/install/setup.bash
export LD_LIBRARY_PATH=/home/jcallano/ros2_ws/libuvc_install/lib:$LD_LIBRARY_PATH
ros2 launch astra_camera astra.launch.xml
```

### RViz (sensores)
Config creado: `src/jetauto_description/rviz/sensors.rviz`
```
source /opt/ros/jazzy/setup.bash
source /home/jcallano/ros2_ws/install/setup.bash
rviz2 -d /home/jcallano/ros2_ws/src/jetauto_description/rviz/sensors.rviz
```
Nota: si `/camera/depth/points` reporta error por TF, conectar el árbol del robot con el frame del driver de cámara:
```
source /opt/ros/jazzy/setup.bash
source /home/jcallano/ros2_ws/install/setup.bash
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 --frame-id depth_cam_link --child-frame-id camera_link
```

### Bringup unificado
Launch: `jetauto_description/launch/robot_bringup.launch.py`
- Por defecto `use_rviz:=false` para evitar problemas de memoria/GUI. Lanzar RViz manualmente si es necesario.
- **Script recomendado:** `bringup.sh` — mata nodos stale (SIGTERM → sleep 2 → SIGKILL solo para procesos seguros) antes de lanzar.

```bash
./bringup.sh
```

O directamente:
```bash
source /opt/ros/jazzy/setup.bash
source /home/jcallano/ros2_ws/install/setup.bash
ros2 launch jetauto_description robot_bringup.launch.py
```
Para abrir RViz desde el bringup:
```
ros2 launch jetauto_description robot_bringup.launch.py use_rviz:=true
```

## Contexto de sesión (para retomar tras reinicio)
- OS: Orange Pi 5, Armbian GNOME, Ubuntu 24.04, ROS 2 Jazzy.
- Workspace: `/home/jcallano/ros2_ws`.
- STM32 (robot controller): `/dev/ttyACM0`, 1 Mbps. IMU funciona.  
  Lanza con:
  ```
  export PYTHONPATH=/home/jcallano/ros2_ws/.venv/lib/python3.12/site-packages:$PYTHONPATH
  source /opt/ros/jazzy/setup.bash
  source /home/jcallano/ros2_ws/install/setup.bash
  ros2 launch ros_robot_controller ros_robot_controller.launch.py port:=/dev/ttyACM0 baudrate:=1000000
  ```
- RPLidar A1: `/dev/ttyUSB0`. Frame corregido con `frame_id:=lidar_frame`.
- Astra: launch `astra.launch.xml`, requiere `LD_LIBRARY_PATH` apuntando a `libuvc_install`.
- `robot_description` requiere envs:
  ```
  export LIDAR_TYPE=A1
  export MACHINE_TYPE=JetAuto
  export DEPTH_CAMERA_TYPE=Astra
  ```
- `robot_bringup.launch.py` integra robot_description + lidar + astra + RViz opcional.
- RViz: config `src/jetauto_description/rviz/sensors.rviz`.
- PointCloud2 depende de TF: se integró en `robot_description.launch.py` un TF estático `depth_cam_link -> camera_link`.
- Mapeo motores confirmado:
  - ID1=FL adelante, ID2=RL adelante, ID3=FR atrás, ID4=RR atrás (signos ya compensados en `controller/mecanum.py`).
- Script rápido de bringup:
  - `bringup.sh` (lanza `robot_bringup` con envs y paths correctos).
### Mecanum + odom
```
source /opt/ros/jazzy/setup.bash
source /home/jcallano/ros2_ws/install/setup.bash
ros2 run controller odom_publisher --ros-args --params-file /home/jcallano/ros2_ws/src/controller/config/calibrate_params.yaml
```

### Test /cmd_vel
```
source /opt/ros/jazzy/setup.bash
source /home/jcallano/ros2_ws/install/setup.bash
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -1
```

## Estado actual
- `ros_robot_controller` corre OK si se exporta `PYTHONPATH` del venv.
- `odom_publisher` convierte `/cmd_vel` a `MotorsState` correctamente.
- `astra_camera` y `rplidar_ros` están compilados e integrados en el workspace.
- `robot_description` publica TF completo cuando se definen `LIDAR_TYPE`, `MACHINE_TYPE`, `DEPTH_CAMERA_TYPE`.
- RViz tiene configuración lista para `/scan` y `/camera/depth/points`.
- **`astra_color_node.py` operativo:** MJPG 640×480 via raw V4L2 + GStreamer mppjpegdec (VPU Rockchip). Publica `/camera/color/image_raw` a ~15–17 fps simultáneamente con depth + IR. Validado estable 65 s sin desconexiones USB.
- **`bringup.sh` limpia nodos stale** antes de lanzar para evitar conflictos de device busy.

## Verificación activa (bringup completo validado)
Nodos activos:
- `/camera/camera` (astra_camera — depth + IR)
- `/camera/color` (astra_color_node — RGB MJPG)
- `/joint_state_publisher`
- `/odom_publisher`
- `/robot_state_publisher`
- `/rplidar_node`
- `/static_transform_publisher_*` (transform para `camera_depth_optical_frame`)

Topics verificados:
- `/scan`
- `/camera/depth/points`
- `/camera/depth/image_raw`
- `/camera/ir/image_raw`
- `/camera/color/image_raw` (`bgr8`, ~15–17 fps con depth+IR simultáneos)
- `/robot_description`
- `/tf`, `/tf_static`

## Estado de parámetros del chasis
- El chasis utilizado **es el mismo** del sourcecode original.
- Dimensiones base validadas desde el sourcecode:
  - `wheelbase = 0.216`
  - `track_width = 0.195`
  - `wheel_diameter = 0.097`

## Mapeo de motores y sentido (verificado)
Prueba individual con `ros2 topic pub /ros_robot_controller/set_motor`:
- Motor ID 1 -> **FL (front-left)**: `rps > 0` = adelante (CCW visto desde el robot).
- Motor ID 2 -> **RL (rear-left)**: `rps > 0` = adelante.
- Motor ID 3 -> **FR (front-right)**: `rps > 0` = **atrás**.
- Motor ID 4 -> **RR (rear-right)**: `rps > 0` = **atrás**.

Este patrón coincide con la inversión de signo que ya aplica `controller/mecanum.py` para los motores 3 y 4.

## Pendientes
- Integrar `odom_publisher` en `robot_bringup.launch.py` para arranque completamente unificado.
- Mejorar estabilidad de tasa del color (~15–17 fps): investigar tuning del pipeline GStreamer mppjpegdec o ajustar `max-buffers` de appsrc.
- RPLidar: ocasional `SL_RESULT_OPERATION_TIMEOUT` al arrancar — investigar si es por boot contention.
