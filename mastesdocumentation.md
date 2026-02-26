# JetAuto Robot — Documentación Maestra

**Plataforma:** Hiwonder JetAuto · Orange Pi 5 (RK3588S) · ROS 2 Jazzy Jalisco
**Propósito:** Robot móvil de monitoreo de gatos en interiores (~100 m², Omán)
**Fecha última actualización:** 2026-02-26

---

## Índice

1. [Hardware del Sistema](#1-hardware-del-sistema)
2. [Configuración de Software Base](#2-configuración-de-software-base)
3. [Workspace ROS 2 — Estructura y Compilación](#3-workspace-ros-2--estructura-y-compilación)
4. [Protocolo STM32 — Comunicación Serial](#4-protocolo-stm32--comunicación-serial)
5. [Nodos ROS 2 — Referencia Completa](#5-nodos-ros-2--referencia-completa)
6. [Topics y Mensajes](#6-topics-y-mensajes)
7. [Cámara Astra Pro Plus — Arquitectura y Solución de Problemas](#7-cámara-astra-pro-plus--arquitectura-y-solución-de-problemas)
8. [Teleoperación con Gamepad](#8-teleoperación-con-gamepad)
9. [Inferencia AI — YOLOv8 en NPU Rockchip](#9-inferencia-ai--yolov8-en-npu-rockchip)
10. [Bringup del Sistema](#10-bringup-del-sistema)
11. [Comandos de Referencia Rápida](#11-comandos-de-referencia-rápida)
12. [Estado Actual y Pendientes](#12-estado-actual-y-pendientes)

---

## 1. Hardware del Sistema

### 1.1 SBC Principal

| Campo | Valor |
|-------|-------|
| Modelo | Orange Pi 5 (SoC RK3588S) |
| OS | Ubuntu 24.04 (Noble) sobre Armbian GNOME |
| Kernel | 6.1.115-vendor-rk35xx |
| Almacenamiento | Boot en SSD NVMe |
| Arquitectura | AArch64 (ARM 64-bit) |
| CPU LITTLE (cores 0-3) | Cortex-A55 @ 1.8 GHz |
| CPU Big (cores 4-7) | Cortex-A76 @ 2.4 GHz |
| GPU | Mali-G610 @ 1.0 GHz |
| NPU | RKNPU @ hasta 1.0 GHz · `/dev/dri/renderD129` |

### 1.2 Chasis y Mecánica

| Campo | Valor |
|-------|-------|
| Modelo | Hiwonder JetAuto |
| Tipo | 4WD Mecanum (cinemática omnidireccional) |
| Wheelbase | 0.216 m |
| Track width | 0.195 m |
| Diámetro rueda | 0.097 m |

**Mapeo de motores (verificado por hardware):**

| Motor ID | Posición | Sentido `rps > 0` |
|----------|----------|-------------------|
| 1 | FL (Front-Left) | Adelante (CCW visto desde el robot) |
| 2 | RL (Rear-Left) | Adelante |
| 3 | FR (Front-Right) | **Atrás** (signo invertido en mecanum.py) |
| 4 | RR (Rear-Right) | **Atrás** (signo invertido en mecanum.py) |

### 1.3 Periféricos y Puertos

| Dispositivo | Puerto | Notas |
|-------------|--------|-------|
| STM32 (JetAuto controller) | `/dev/ttyACM0` | 1 Mbps, DTR/RTS reset requerido |
| RPLidar A1 | `/dev/ttyUSB0` | Bus USB 003 (ehci, 480M) |
| Astra RGB UVC | `/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0` | Bus 005, `2bc5:050f` |
| Astra Depth | (userspace libuvc) | Bus 005, `2bc5:060f`, Driver=[none] es normal |

**Mapa de buses USB (Orange Pi 5):**

| Bus | Controlador | Velocidad | Uso |
|-----|-------------|-----------|-----|
| 002 | xhci / usbhost3_0 | 5000M (USB 3.0) | Libre |
| 003 | ehci / fc800000.usb | 480M (USB 2.0) | Hub JetAuto (STM32 + RPLidar) |
| 005 | ehci / fc880000.usb | 480M (USB 2.0) | Astra Pro Plus |
| 007 | xhci / usbdrd3_0 | 480M | Logitech Unifying (Gamepad) |

### 1.4 Arquitectura USB Interna de la Cámara Astra

```
USB Bus 005 (ehci-platform, 480 Mbps)
└── 5-1: Genesys Logic GL850G (05e3:0608)  ← hub interno USB 2.0
    ├── 5-1.1: Depth sensor   (2bc5:060f)  ← libuvc (astra_camera)
    └── 5-1.2: UVC RGB camera (2bc5:050f)  ← kernel uvcvideo
```

> **CRÍTICO:** El hub GL850G es **USB 2.0 únicamente** (máx ~40 MB/s útiles).
> YUYV 640×480@30fps = 18.4 MB/s + Depth ~18 MB/s + IR ~18 MB/s ≈ 54 MB/s → saturación.
> **Solución implementada:** MJPG para color (~1-3 MB/s), elimina la saturación.

---

## 2. Configuración de Software Base

### 2.1 Device Tree Overlays

Archivo: `/boot/armbianEnv.txt`
```
overlays=dmc-oc-3500mhz opp-oc-24ghz
```

| Overlay | Estado | Motivo |
|---------|--------|--------|
| `dmc-oc-3500mhz` | **ACTIVO** | DDR a 3500 MHz. Evita micro-congelamientos en streams de cámara. Sin errores. |
| `opp-oc-24ghz` | **ACTIVO** | Habilita 2.4 GHz en cores A76. Genera `pvtpll voltsel=-1` no fatales. Necesario para rendimiento. |
| `panthor-gpu` | **DESACTIVADO** | Causaba inestabilidad. Driver Mali DDK activo en su lugar. |
| `khadas-edge2-cam3` | **ELIMINADO** | Era overlay de otra placa. Causaba errores `rkcif MIPI sensor failed`. |

### 2.2 Errores de Kernel Conocidos (No Fatales)

| Error | Causa | Acción |
|-------|-------|--------|
| `pvtpll voltsel=-1` | `opp-oc-24ghz` sin entradas PVTPLL | Aceptado, ignorar |
| `Unsupported VOP aclk dvfs` | Limitación del driver | Ignorar, cosmético |
| `dw9714 i2c write failed -6` | Driver cámara MIPI buscando hardware inexistente | Ignorar |

### 2.3 Performance Governor (Persistente)

Servicio: `/etc/systemd/system/performance-governor.service` (enabled, active)

| Componente | Governor | Frecuencia |
|------------|----------|------------|
| CPU policy0 (cores 0-3, LITTLE) | performance | 1.8 GHz |
| CPU policy4 (cores 4-5, Big) | performance | 2.4 GHz |
| CPU policy6 (cores 6-7, Big) | performance | 2.4 GHz |
| GPU Mali | performance | 1.0 GHz |
| NPU RKNPU | rknpu_ondemand | hasta 1.0 GHz |

> La NPU usa `rknpu_ondemand` (no `performance`) — escala automáticamente bajo carga, comportamiento correcto para inferencia.

### 2.4 Regla udev — Autosuspend USB

Archivo: `/etc/udev/rules.d/55-orbbec-power.rules`
Deshabilita autosuspend en la Astra Pro Plus para evitar EPROTO en STREAMON.

Verificar estado:
```bash
cat /sys/bus/usb/devices/5-1.2/power/control  # debe decir "on"
```

---

## 3. Workspace ROS 2 — Estructura y Compilación

### 3.1 Ubicación y Entorno

```
/home/jcallano/ros2_ws/
├── src/                    # paquetes fuente
├── install/                # instalación colcon
├── build/                  # artefactos de compilación
├── .venv/                  # Python venv con pyserial, rknn-toolkit-lite2, etc.
├── libuvc_install/         # libuvc compilada manualmente para AArch64
├── bringup.sh              # script de arranque completo
└── mastesdocumentation.md  # este archivo
```

### 3.2 Paquetes del Workspace

| Paquete | Tipo | Rol |
|---------|------|-----|
| `ros_robot_controller` | Python | Bridge STM32 via UART (motores, IMU, LED, servos, buzzer) |
| `ros_robot_controller_msgs` | CMake (msgs) | Definiciones de mensajes y servicios STM32 |
| `controller` | Python | Cinemática Mecanum + odometría (`/cmd_vel` → `MotorsState`) |
| `kinematics` | Python | Solver IK/FK para brazo 5-DOF |
| `kinematics_msgs` | CMake (msgs) | Mensajes para control de brazo |
| `jetauto_description` | Python (datos) | URDF/Xacro, meshes, RViz config, scripts Python |
| `ros2_astra_camera` | C++ | Driver Astra Pro Plus (depth + IR + pointcloud via OpenNI/libuvc) |
| `rplidar_ros` | C++ | Driver RPLidar A1 |
| `vision_opencv` | C++ | cv_bridge (compilado desde fuente — ver nota ABI) |
| `image_common` | C++ | image_transport (compilado desde fuente — ver nota ABI) |

> **⚠️ Nota ABI:** `vision_opencv` e `image_common` se compilan desde fuente con ABI diferente a los paquetes apt del sistema. El paquete `ros-jazzy-v4l2-camera` del repositorio apt es incompatible → usar `astra_color_node.py` en su lugar.

### 3.3 Variables de Entorno Requeridas

```bash
# ROS y workspace
source /opt/ros/jazzy/setup.bash
source /home/jcallano/ros2_ws/install/setup.bash

# Python venv (para pyserial, rknn-toolkit-lite2, etc.)
export PYTHONPATH=/home/jcallano/ros2_ws/.venv/lib/python3.12/site-packages:${PYTHONPATH:-}

# libuvc (para astra_camera)
export LD_LIBRARY_PATH=/home/jcallano/ros2_ws/libuvc_install/lib:${LD_LIBRARY_PATH:-}

# Variables requeridas por URDF/Xacro del jetauto_description
export LIDAR_TYPE=A1
export MACHINE_TYPE=JetAuto
export DEPTH_CAMERA_TYPE=Astra
```

### 3.4 Compilación

```bash
cd /home/jcallano/ros2_ws
source /opt/ros/jazzy/setup.bash

# Compilación completa con symlinks (recomendado)
colcon build --symlink-install

# Compilación de un paquete específico
colcon build --symlink-install --packages-select jetauto_description

source install/setup.bash
```

> **Nota symlink-install:** Crea la cadena `install → build → src`. Si previamente existía una copia en lugar de symlink, hay que eliminarla primero:
> `rm install/jetauto_description/lib/jetauto_description/astra_color_node.py`

### 3.5 Archivos Clave

| Archivo | Descripción |
|---------|-------------|
| `src/ros_robot_controller/ros_robot_controller/ros_robot_controller_node.py` | Nodo bridge STM32 |
| `src/ros_robot_controller/ros_robot_controller/ros_robot_controller_sdk.py` | SDK UART STM32 |
| `src/controller/controller/odom_publisher_node.py` | Odometría Mecanum |
| `src/controller/controller/mecanum.py` | Cinemática Mecanum |
| `src/controller/config/calibrate_params.yaml` | Parámetros físicos chasis |
| `src/kinematics/kinematics/search_kinematics_solutions_node.py` | IK brazo 5-DOF |
| `src/jetauto_description/urdf/jetauto.xacro` | URDF principal |
| `src/jetauto_description/launch/robot_bringup.launch.py` | Launch unificado |
| `src/jetauto_description/launch/robot_description.launch.py` | TF tree + TF estático cámara |
| `src/jetauto_description/scripts/astra_color_node.py` | Nodo RGB cámara (raw V4L2 + mppjpegdec) |
| `src/jetauto_description/scripts/jetauto_teleop_joy.py` | Teleoperación gamepad |
| `src/jetauto_description/rviz/sensors.rviz` | Config RViz sensores |
| `bringup.sh` | Script arranque con limpieza de procesos stale |

---

## 4. Protocolo STM32 — Comunicación Serial

### 4.1 Configuración

| Parámetro | Valor |
|-----------|-------|
| Puerto | `/dev/ttyACM0` |
| Baudrate | **1,000,000 bps (1 Mbps)** |
| Reset al abrir | DTR/RTS: HIGH 100ms → LOW → esperar 500ms |

> **IMU Wake-Up:** El STM32 tiene un fallo I2C al arrancar. Si DTR/RTS no se pulsan al abrir el puerto, el MPU6050 no inicia correctamente. El driver `ros_robot_controller_node.py` hace esto automáticamente.

### 4.2 Estructura del Paquete

```
0xAA  0x55  [Function ID]  [Length]  [Data Payload...]  [CRC8]
```

| Campo | Bytes | Descripción |
|-------|-------|-------------|
| Header | 2 | Siempre `0xAA 0x55` |
| Function ID | 1 | Comando/tipo de dato |
| Length | 1 | Longitud del payload en bytes |
| Data Payload | N | Parámetros del comando |
| CRC8 | 1 | Checksum sobre FuncID + Length + Data |

> CRC8 usa tabla Latch personalizada precalculada (`crc8_table` en el SDK), operaciones XOR byte a byte → `uint8`.

### 4.3 Function IDs

| ID | Nombre | Dirección | Descripción |
|----|--------|-----------|-------------|
| `0x00` | SYS / Batería | STM32 → PC | Voltaje batería en mV (uint16 LE) · Payload: `[0x04, Bat_L, Bat_H]` |
| `0x01` | LED | PC → STM32 | Parpadeo LED · Payload: `[ID, OnTime_L, OnTime_H, OffTime_L, OffTime_H, Repeat_L, Repeat_H]` (7 bytes) |
| `0x02` | Buzzer | PC → STM32 | Tono+duración · Payload: `[Freq_L, Freq_H, OnTime_L, OnTime_H, OffTime_L, OffTime_H, Repeat_L, Repeat_H]` (8 bytes) |
| `0x03` | Motor DC | PC → STM32 | Velocidad ruedas · Payload: `[0x01, Cantidad, (ID, Vel_float32)...]` · ID enviado como `(ID real - 1)` |
| `0x04` | PWM Servo | PC → STM32 | **⚠️ DESACTIVADO en firmware JetAuto** — ignorado por STM32 |
| `0x05` | Bus Servo | Bidireccional | Brazo 5-DOF y paneo cámara · Ver sección 4.4 |
| `0x06` | Botones | STM32 → PC | Pulsadores KEY1/KEY2 · Payload: `[Key_ID, Evento]` |
| `0x07` | IMU | STM32 → PC | MPU6050 · Payload: 6 × float32 = 24 bytes `[ax, ay, az, gx, gy, gz]` |
| `0x08` | Gamepad | STM32 → PC | Mando inalámbrico · Payload: 7 bytes `[Btn_L, Btn_H, Hat, LX, LY, RX, RY]` |
| `0x09` | SBUS | STM32 → PC | RC aeromodelismo · Payload: 36 bytes |
| `0x0A+` | — | — | No mapeado |

### 4.4 Bus Servo (Brazo + Cámara PTZ)

Sub-comandos destacados del Function ID `0x05`:

| Sub-cmd | Acción |
|---------|--------|
| `0x01` | Set Position · Payload: `[0x01, Duration_L, Duration_H, Cantidad, (Servo_ID, Pos_L, Pos_H)...]` |
| `0x03` | Stop (detener giro) |
| `0x0B` / `0x0C` | Activar / Desactivar Torque |
| `0x10` | Set ID |
| `0x12`, `0x22` | Leer IDs / Offset |

**Servo de paneo cámara Astra:**

| Campo | Valor |
|-------|-------|
| Servo ID | `1` |
| Extremo derecho | `200` |
| Centro | `500` |
| Extremo izquierdo | `800` |

---

## 5. Nodos ROS 2 — Referencia Completa

### 5.1 ros_robot_controller (STM32 Bridge)

- **Ejecutable:** `ros_robot_controller`
- **Launch:** `ros_robot_controller.launch.py`
- **Puerto:** `/dev/ttyACM0` · Baudrate: `1000000`

| Parámetro | Default | Descripción |
|-----------|---------|-------------|
| `port` | `/dev/ttyUSB0` | **Usar `/dev/ttyACM0` en JetAuto** |
| `baudrate` | `1000000` | — |
| `timeout` | `5.0` | Timeout UART (s) |
| `imu_frame` | `imu_link` | Frame del IMU en TF |
| `reset_dtr_rts` | `True` | Pulse DTR/RTS al abrir (necesario para IMU) |
| `reset_pulse_ms` | `100` | Duración del pulso DTR/RTS (ms) |
| `reset_post_ms` | `500` | Espera tras el pulso (ms) |

### 5.2 controller/odom_publisher (Mecanum + Odometría)

- **Ejecutable:** `odom_publisher`
- **Config:** `src/controller/config/calibrate_params.yaml`

| Parámetro | Valor validado |
|-----------|---------------|
| `wheelbase` | `0.216` m |
| `track_width` | `0.195` m |
| `wheel_diameter` | `0.097` m |
| `linear_correction_factor` | `1.00` |
| `angular_correction_factor` | `1.04` |

### 5.3 astra_camera (Depth + IR + PointCloud)

- **Launch:** `astra_pro.launch.xml`
- **Namespace:** `/camera` · **Node:** `/camera/camera`
- **Requiere:** `LD_LIBRARY_PATH` apuntando a `libuvc_install/lib`
- **Parámetros en bringup:**

| Parámetro | Valor |
|-----------|-------|
| `enable_color` | `false` (lo maneja `astra_color_node.py`) |
| `use_uvc_camera` | `false` |
| `depth_width/height/fps` | `640/480/15` |
| `enable_ir` | `true` · `640/480/30` |
| `enable_point_cloud` | `true` |

> **⚠️ NUNCA usar `kill -9` en `astra_camera_node`** — puede desconectar el depth sensor (`2bc5:060f`) del bus USB hasta replug físico. Siempre usar SIGTERM + esperar.

### 5.4 astra_color_node (RGB Color — Raw V4L2)

- **Ejecutable:** `src/jetauto_description/scripts/astra_color_node.py`
- **Namespace:** `camera` · **Name:** `color`
- **Topic publicado:** `/camera/color/image_raw` (`sensor_msgs/Image`, encoding `bgr8`)

| Parámetro | Default |
|-----------|---------|
| `video_device` | symlink by-id del Sonix UVC |
| `width` | `640` |
| `height` | `480` |
| `fps` | `30` |
| `frame_id` | `camera_color_optical_frame` |

**Arquitectura:**
1. **Captura:** Raw V4L2 ioctls (struct v4l2_buffer = 88 bytes AArch64), formato MJPG, mmap con 4 buffers
2. **Decode:** GStreamer pipeline → `mppjpegdec` (VPU Rockchip MPP)
3. **Fallback automático decode:** `mppjpegdec` → `v4l2jpegdec` → `jpegdec` (SW) → `cv2.imdecode`
4. **Retry apertura:** 10 intentos × 1 s (tolera device busy durante negociación USB)

**Tasas esperadas (64x480, depth+IR simultáneos, test 65 s):**
- Standalone (solo color): ~30 fps
- Simultáneo con depth+IR: ~15–17 fps avg (rango 8–20 fps), sin crashes

**Constantes V4L2 AArch64 (struct v4l2_buffer = 88 bytes):**

| ioctl | Valor AArch64 | Valor x86 (incorrecto) |
|-------|---------------|------------------------|
| `VIDIOC_QUERYBUF` | `0xc0585609` | `0xc0445609` |
| `VIDIOC_QBUF` | `0xc058560f` | `0xc044560f` |
| `VIDIOC_DQBUF` | `0xc0585611` | `0xc0445611` |
| `VIDIOC_S_FMT` | `0xc0d05605` | — |
| `VIDIOC_REQBUFS` | `0xc0145608` | — |
| `VIDIOC_STREAMON` | `0x40045612` | — |
| `VIDIOC_STREAMOFF` | `0x40045613` | — |

> **⚠️ NUNCA usar `cv2.VideoCapture(dev, cv2.CAP_V4L2)`** — usa constantes x86 incorrectas → ioctls corruptos → USB reset de la cámara en AArch64.

### 5.5 rplidar_node

- **Launch:** `rplidar_a1_launch.py`
- **Puerto:** `/dev/ttyUSB0` · **Frame:** `lidar_frame`
- **Topic:** `/scan` (`sensor_msgs/LaserScan`) · ~7 Hz (normal para A1 por USB)

### 5.6 robot_state_publisher + joint_state_publisher

- Requieren variables: `LIDAR_TYPE=A1`, `MACHINE_TYPE=JetAuto`, `DEPTH_CAMERA_TYPE=Astra`, `need_compile=True`
- Incluyen TF estático `depth_cam_link → camera_link` (integrado en `robot_description.launch.py`)

### 5.7 jetauto_teleop_joy (Teleoperación Gamepad)

- **Script:** `src/jetauto_description/scripts/jetauto_teleop_joy.py`
- **Requiere:** `ros_robot_controller` activo
- Ver sección 8 para mapeo completo de controles

---

## 6. Topics y Mensajes

### 6.1 Topics Principales

| Topic | Tipo | Dirección | Nodo |
|-------|------|-----------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | Entrada | `odom_publisher` |
| `/odom_raw` | `nav_msgs/Odometry` | Salida | `odom_publisher` |
| `/ros_robot_controller/set_motor` | `MotorsState` | `odom_publisher` → STM32 | — |
| `/ros_robot_controller/imu_raw` | `sensor_msgs/Imu` | STM32 → salida | — |
| `/ros_robot_controller/battery` | `std_msgs/UInt16` | STM32 → salida | mV (ej. 11500 = 11.5V) |
| `/ros_robot_controller/button` | `ButtonState` | STM32 → salida | id: 1/2, state: 1=pulso, 2=sostenido |
| `/ros_robot_controller/joy` | `sensor_msgs/Joy` | STM32 → salida | Gamepad desempaquetado |
| `/ros_robot_controller/set_led` | `LedState` | entrada → STM32 | id, on_time, off_time, repeat |
| `/ros_robot_controller/set_buzzer` | `BuzzerState` | entrada → STM32 | freq(Hz), on_time(s), off_time(s), repeat |
| `/ros_robot_controller/set_oled` | `OLEDState` | entrada → STM32 | index(línea), text(String) |
| `/ros_robot_controller/bus_servo/set_position` | `ServosPosition` | entrada → STM32 | duration(s), position[{id, position}] |
| `/scan` | `sensor_msgs/LaserScan` | RPLidar → salida | frame: `lidar_frame` |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | astra_camera → salida | ~12–30 fps |
| `/camera/depth/points` | `sensor_msgs/PointCloud2` | astra_camera → salida | ~20 fps, jitter alto |
| `/camera/ir/image_raw` | `sensor_msgs/Image` | astra_camera → salida | ~30 fps estable |
| `/camera/color/image_raw` | `sensor_msgs/Image` | astra_color_node → salida | bgr8, ~15–17 fps con depth+IR |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | robot_description → salida | árbol TF completo |

### 6.2 Servicios STM32

| Servicio | Tipo | Descripción |
|----------|------|-------------|
| `~/init_finish` | `std_srvs/Trigger` | Confirma que STM32 inicializó correctamente |
| `~/bus_servo/get_state` | — | Estado de servo en bus |
| `~/pwm_servo/get_state` | — | Estado de servo PWM |

---

## 7. Cámara Astra Pro Plus — Arquitectura y Solución de Problemas

### 7.1 Problema 1: cv2.VideoCapture Causa USB Reset en AArch64

**Síntoma:** `cap.grab()` retorna `False`, seguido de desconexión/reconexión USB en dmesg.

**Causa:** El backend V4L2 de OpenCV usa `struct v4l2_buffer` de 68 bytes (compilado para x86). En AArch64 es 88 bytes porque `struct timeval` pasa de 8B a 16B. Los ioctls incorrectos llegan al firmware Orbbec → USB reset.

**Solución:** Usar `astra_color_node.py` con raw V4L2 ioctls y las constantes correctas para AArch64 (ver tabla en sección 5.4).

### 7.2 Problema 2: VIDIOC_STREAMON Falla con EPROTO (errno 71)

**Síntoma:** `VIDIOC_STREAMON failed: [Errno 71] Protocol error`

**Causa A — Procesos stale:**
```bash
pgrep -a -f astra_camera_node   # verificar
pkill -TERM -f astra_camera_node && sleep 2
```

**Causa B — Firmware UVC en estado inválido:** Resetear SOLO la cámara UVC (no el hub):
```python
import fcntl, os, glob

USBDEVFS_RESET = 0x5514

def find_usb_device(vendor, product):
    for d in glob.glob('/sys/bus/usb/devices/*/idVendor'):
        base = os.path.dirname(d)
        try:
            v = open(d).read().strip()
            p = open(base + '/idProduct').read().strip()
            if v == vendor and p == product:
                busnum = int(open(base + '/busnum').read().strip())
                devnum = int(open(base + '/devnum').read().strip())
                return f'/dev/bus/usb/{busnum:03d}/{devnum:03d}'
        except OSError:
            pass
    return None

path = find_usb_device('2bc5', '050f')  # SOLO el UVC RGB
if path:
    fd = os.open(path, os.O_WRONLY)
    fcntl.ioctl(fd, USBDEVFS_RESET)
    os.close(fd)
```

> **⚠️ NUNCA resetear el hub GL850G (`05e3:0608`)** — puede dejar el depth sensor (`2bc5:060f`) en estado de enumeración fallida que requiere replug físico.

**Causa C — Autosuspend:** Verificar `/sys/bus/usb/devices/5-1.2/power/control` == `"on"`. La regla `/etc/udev/rules.d/55-orbbec-power.rules` debería mantenerlo activo.

### 7.3 Problema 3: Saturación USB con YUYV + Depth + IR

**Causa:** Hub GL850G máx ~40 MB/s útiles. YUYV(18.4) + Depth(~18) + IR(~18) ≈ 54 MB/s.

**Solución implementada:** MJPG para color reduce de 18.4 MB/s a ~1–3 MB/s.

| Formato | MB/s | Viable con depth+IR |
|---------|------|:---:|
| YUYV 640×480@30fps | 18.4 | No |
| YUYV 640×480@10fps | 6.1 | Marginal |
| YUYV 320×240@30fps | 4.6 | Sí |
| MJPG 640×480@30fps | ~1–3 | **Sí** |

### 7.4 Problema 4: v4l2_camera Package — Símbolo No Encontrado

**Síntoma:** `undefined symbol: _ZN15image_transport23create_camera_publisherE...`

**Causa:** Incompatibilidad ABI entre `image_common` compilado desde fuente y el paquete apt del sistema.

**Solución:** Usar `astra_color_node.py` que solo depende de `rclpy`, `cv_bridge`, `numpy` y `cv2`.

### 7.5 Problema 5: kill -9 Desconecta el Depth Sensor del Bus USB

**Síntoma:** Tras `kill -9 astra_camera_node`, el sensor `2bc5:060f` desaparece de `lsusb`. Bringup posterior falla con ENODEV.

**Causa:** `astra_camera_node` mantiene un handle libuvc sobre el USB. SIGKILL no permite cerrar limpiamente → el firmware del GL850G no puede recuperar el endpoint.

**Prevención:** **NUNCA usar `kill -9` en `astra_camera_node`.**
```bash
pkill -TERM -f astra_camera_node
sleep 2
# Solo si SIGTERM no funcionó:
pkill -KILL -f astra_camera_node
```

**Recuperación:** Solo replug físico del cable USB de la Astra.

### 7.6 Problema 6: GStreamer PyGObject — pull_sample No Disponible

**Síntoma:** `AttributeError: 'GstAppSink' object has no attribute 'pull_sample'` o `try_pull_sample`.

**Causa:** `pull_sample()` y `try_pull_sample()` requieren importar el typelib `GstApp` explícitamente. No siempre disponible en todas las instalaciones de PyGObject.

**Solución:** Usar `sink.emit('pull-sample')` — es un action signal de GstAppSink siempre disponible sin GstApp:
```python
def on_new_sample(sink, _sq=sq, _Gst=Gst):
    sample = sink.emit('pull-sample')  # ← action signal, sin GstApp
    try:
        _sq.put_nowait(sample)
    except Exception:
        pass
    return _Gst.FlowReturn.OK

appsink.connect('new-sample', on_new_sample)
```

### 7.7 Pipeline GStreamer (Implementación Final)

```python
pipeline_str = (
    f"appsrc name=src format=time is-live=true block=false do-timestamp=true "
    f"max-buffers=2 leaky-type=downstream ! "
    f"image/jpeg,width={width},height={height} ! "
    f"jpegparse ! mppjpegdec ! "
    f"videoconvert ! video/x-raw,format=BGR ! "
    f"appsink name=sink emit-signals=true max-buffers=1 drop=true sync=false"
)
```

> `max-buffers=2 leaky-type=downstream` en appsrc es crítico para evitar acumulación de backlog que produce gaps de hasta 1.8 s.

### 7.8 Reporte de Estabilidad (Medición Real)

**Test A — Solo color, YUYV, 640×480 (30 s):**

| Stream | FPS | Std dev |
|--------|-----|---------|
| Color YUYV (standalone) | 30.0 | 2 ms |

**Test B — Simultáneo MJPG + mppjpegdec, 640×480 (65 s):**

| Sensor | Topic | FPS avg | Rango | Max gap |
|--------|-------|---------|-------|---------|
| IR | `/camera/ir/image_raw` | **30.0** | 30–30 | — |
| Depth | `/camera/depth/image_raw` | **~23** | 12–30 | — |
| Color MJPG | `/camera/color/image_raw` | **~15–17** | 8–20 | 0.748 s |

**Sin errores en test completo:** Ninguna desconexión USB, ningún crash de nodo.

---

## 8. Teleoperación con Gamepad

### 8.1 Mapeo de Botones `sensor_msgs/Joy`

**Ejes (`msg.axes`):**

| Control Físico | Índice | Valores |
|----------------|--------|---------|
| Joystick Izq — X | `0` | +1.0 = Izquierda, -1.0 = Derecha |
| Joystick Izq — Y | `1` | +1.0 = Arriba, -1.0 = Abajo |
| Joystick Der — X | `2` | +1.0 = Izquierda, -1.0 = Derecha |
| Joystick Der — Y | `3` | +1.0 = Arriba, -1.0 = Abajo |
| R2 (gatillo der) | `4` | 0.0 libre → 1.0 fondo |
| L2 (gatillo izq) | `5` | 0.0 libre → 1.0 fondo |
| D-Pad — X | `6` | +1.0 = Izquierda, -1.0 = Derecha |
| D-Pad — Y | `7` | +1.0 = Arriba, -1.0 = Abajo |

**Botones (`msg.buttons`):**

| Botón Físico | Índice |
|--------------|--------|
| A (Cruz) | `0` |
| B (Círculo) | `1` |
| X (Cuadrado) | `3` |
| Y (Triángulo) | `4` |
| L1 (Bumper Izq) | `6` |
| R1 (Bumper Der) | `7` |
| Select | `10` |
| Start | `11` |
| L3 (click stick izq) | `13` |
| R3 (click stick der) | `14` |

### 8.2 Acciones de Teleoperación (jetauto_teleop_joy.py)

| Control | Acción |
|---------|--------|
| Stick Izquierdo | Avanzar/Retroceder + Strafe lateral Mecanum |
| Stick Derecho (Eje X) | Rotar sobre su propio eje |
| D-Pad Izq/Der | Girar cámara Astra (paneo servo ID1) |
| L3 (click) | Auto-centrado rápido cámara (→ posición 500) |
| L1 | Tocar buzzer corto |
| L2 (hold) | Modo Precisión — velocidad al 25% |
| R2 (hold) | Modo Turbo — velocidad ×2 |
| START + SELECT | **Emergency Stop** — detiene las 4 ruedas |

---

## 9. Inferencia AI — YOLOv8 en NPU Rockchip

### 9.1 Preparación del Modelo (PC x86 Remoto)

El modelo debe compilarse a formato `.rknn` en un PC x86 con `rknn-toolkit2` (no disponible en ARM).

```python
# Paso 1: Exportar a ONNX (opset 12 obligatorio para RKNN)
from ultralytics import YOLO
model = YOLO("yolov8n.pt")
model.export(format="onnx", opset=12)

# Paso 2: Compilar a RKNN (en PC x86)
from rknn.api import RKNN
rknn = RKNN(verbose=True)
rknn.config(mean_values=[[0, 0, 0]], std_values=[[255, 255, 255]], target_platform='rk3588')
rknn.load_onnx(model='yolov8n.onnx')
rknn.build(do_quantization=False)
rknn.export_rknn('yolov8n.rknn')
```

> **Versiones estrictas:** `torch==2.4.0`, `torchvision==0.19.0`, `onnx==1.16.1` (ONNX 1.20+ rompe rknn-toolkit2).

### 9.2 Ejecución en NPU (Orange Pi 5)

**Dependencias en `.venv`:**
```bash
pip install "numpy<2" rknn-toolkit-lite2 opencv-python cv-bridge
```
> `numpy >= 2.0` genera SegFault en la API Rockchip Lite. Pinear a `< 2` (recomendado `1.26.4`).

**Requisito de runtime:**
```
/usr/lib/librknnrt.so   ← descargable de rknpu2/runtime/Linux/librknn_api/aarch64/
```

**Nodo de inferencia:**
- Suscrito a `/camera/color/image_raw`
- Publica `vision_msgs/Detection2DArray`
- Ejecutar: `ros2 run jetauto_description jetauto_yolo_rknn.py`

---

## 10. Bringup del Sistema

### 10.1 Script Recomendado: bringup.sh

El script `bringup.sh` en la raíz del workspace:
1. Fuente los entornos ROS y workspace
2. Exporta variables de entorno necesarias
3. **Mata nodos stale** antes de arrancar (evita conflictos de device busy):
   - `pkill -TERM` a todos los nodos del bringup → espera 2 s
   - `pkill -KILL` solo a `astra_camera_node` y `astra_color_node` (seguro de matar)
4. Lanza `robot_bringup.launch.py`

```bash
cd /home/jcallano/ros2_ws
./bringup.sh
```

### 10.2 Launch Unificado — robot_bringup.launch.py

Lanza en orden (paralelo con MJPG — no hay race condition de ancho de banda):

1. `robot_description` (TF tree + TF estático `depth_cam_link → camera_link`)
2. `rplidar_a1_launch.py` (RPLidar A1)
3. `ros_robot_controller.launch.py` (bridge STM32)
4. `astra_pro.launch.xml` (depth + IR, `enable_color:=false`, `use_uvc_camera:=false`)
5. `astra_color_node.py` (RGB MJPG + mppjpegdec, namespace `camera`)
6. `rviz2` (solo si `use_rviz:=true`)

```bash
# Sin RViz (por defecto):
ros2 launch jetauto_description robot_bringup.launch.py

# Con RViz:
ros2 launch jetauto_description robot_bringup.launch.py use_rviz:=true
```

### 10.3 Verificación Post-Bringup

```bash
# Listar nodos activos
ros2 node list

# Verificar tasas de topics clave
ros2 topic hz /camera/color/image_raw   # ~15-17 fps con depth+IR
ros2 topic hz /camera/depth/image_raw   # ~12-30 fps
ros2 topic hz /camera/ir/image_raw      # ~30 fps
ros2 topic hz /scan                     # ~7 Hz (normal A1)

# Verificar codificación de imagen color
ros2 topic echo /camera/color/image_raw --no-arr | grep encoding   # bgr8

# Verificar TF completo
ros2 run tf2_ros tf2_echo base_footprint camera_color_optical_frame
```

### 10.4 Flujo de Arranque Manual (Componente por Componente)

Para debug o arranque selectivo:

```bash
# 1. Exportar entorno (siempre primero)
source /opt/ros/jazzy/setup.bash
source /home/jcallano/ros2_ws/install/setup.bash
export PYTHONPATH=/home/jcallano/ros2_ws/.venv/lib/python3.12/site-packages:${PYTHONPATH:-}
export LD_LIBRARY_PATH=/home/jcallano/ros2_ws/libuvc_install/lib:${LD_LIBRARY_PATH:-}

# 2. STM32
ros2 launch ros_robot_controller ros_robot_controller.launch.py \
    port:=/dev/ttyACM0 baudrate:=1000000

# 3. Robot description (TF)
export LIDAR_TYPE=A1 MACHINE_TYPE=JetAuto DEPTH_CAMERA_TYPE=Astra
need_compile=True ros2 launch jetauto_description robot_description.launch.py \
    use_gui:=false use_rviz:=false use_sim_time:=false

# 4. RPLidar A1
ros2 launch rplidar_ros rplidar_a1_launch.py \
    serial_port:=/dev/ttyUSB0 frame_id:=lidar_frame

# 5. Astra depth + IR
ros2 launch astra_camera astra_pro.launch.xml \
    enable_color:=false use_uvc_camera:=false \
    depth_width:=640 depth_height:=480 depth_fps:=15 \
    enable_ir:=true ir_width:=640 ir_height:=480 ir_fps:=30 \
    enable_point_cloud:=true

# 6. Astra color RGB (iniciar DESPUÉS o simultáneo — MJPG soporta ambos)
ros2 run jetauto_description astra_color_node.py

# 7. Odometría Mecanum
ros2 run controller odom_publisher --ros-args \
    --params-file /home/jcallano/ros2_ws/src/controller/config/calibrate_params.yaml

# 8. Teleoperación (opcional)
python3 /home/jcallano/ros2_ws/src/jetauto_description/scripts/jetauto_teleop_joy.py

# 9. RViz (opcional)
rviz2 -d /home/jcallano/ros2_ws/src/jetauto_description/rviz/sensors.rviz
```

---

## 11. Comandos de Referencia Rápida

### Diagnóstico

```bash
# Estado USB de la cámara
lsusb | grep -E "2bc5|05e3"

# Verificar device V4L2
ls /dev/v4l/by-id/ | grep Sonix

# Procesos astra stale (deben ser 0 antes del bringup)
pgrep -c astra_camera_node

# Autosuspend cámara
cat /sys/bus/usb/devices/5-1.2/power/control   # debe ser "on"

# Test streaming V4L2 raw (3 frames)
v4l2-ctl -d /dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0 \
    --stream-mmap --stream-count=3
```

### Actuadores STM32 (Comandos Rápidos)

```bash
# Buzzer (2000 Hz, 2 pitidos)
ros2 topic pub --once /ros_robot_controller/set_buzzer \
    ros_robot_controller_msgs/msg/BuzzerState \
    "{freq: 2000, on_time: 0.1, off_time: 0.1, repeat: 2}"

# Servo cámara — centro
ros2 topic pub --once /ros_robot_controller/bus_servo/set_position \
    ros_robot_controller_msgs/msg/ServosPosition \
    "{duration: 1.0, position: [{id: 1, position: 500}]}"

# Movimiento de prueba (adelante 0.05 m/s, 1 vez)
ros2 topic pub /cmd_vel geometry_msgs/Twist \
    "{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -1
```

### Limpieza de Procesos

```bash
# Limpieza segura (SIGTERM a todo, SIGKILL solo a nodos sin libuvc)
pkill -TERM -f "astra_camera_node|astra_color_node|rplidar_node|ros_robot_controller_node" 2>/dev/null || true
sleep 2
pkill -KILL -f "astra_color_node|rplidar_node|ros_robot_controller_node" 2>/dev/null || true

# ⚠️ Si ya se usó kill -9 y el depth sensor desapareció: replug físico obligatorio
```

claude cli

claude --dangerously-skip-permissions

---

## 12. Estado Actual y Pendientes

### 12.1 Estado Validado (2026-02-26)

| Sistema | Estado | Notas |
|---------|--------|-------|
| STM32 bridge | **OK** | `/dev/ttyACM0`, 1 Mbps, IMU funciona |
| Odometría Mecanum | **OK** | `/cmd_vel` → `MotorsState` correcto |
| RPLidar A1 | **OK** | `/scan` ~7 Hz, frame `lidar_frame` alineado |
| Astra depth + IR | **OK** | Depth 12–30 Hz, IR 30 Hz, PointCloud2 ~20 fps |
| Astra color RGB | **OK** | ~15–17 fps, MJPG + mppjpegdec, estable 65 s |
| TF tree completo | **OK** | `robot_description` publica con envs correctos |
| Bringup unificado | **OK** | `bringup.sh` + `robot_bringup.launch.py` |
| RViz config | **OK** | `sensors.rviz` con color, depth, scan, TF |
| Performance governor | **OK** | Persistente vía systemd |
| NPU RKNPU | **OK** | `/dev/dri/renderD129`, hasta 1.0 GHz |

### 12.2 Pendientes

| Prioridad | Tarea | Notas |
|-----------|-------|-------|
| Alta | Integrar `odom_publisher` en `robot_bringup.launch.py` | Actualmente se lanza manualmente |
| Media | Mejorar tasa color RGB | ~15–17 fps actual; investigar tuning GStreamer mppjpegdec |
| Media | Validar SLAM 3D | Usar depth + RGB + Lidar + odometría + IMU |
| Media | RPLidar SL_RESULT_OPERATION_TIMEOUT al arrancar | Posible boot contention; añadir retry en launch |
| Baja | Detección de gatos con NPU | YOLOv8n.rknn listo, falta integración completa en bringup |
| Baja | Mapa de recursos bajo carga completa | Confirmar margen CPU/RAM para NPU + SLAM simultáneos |

---

*Este documento consolida: `PROJECT_OVERVIEW.md`, `ASTRA_CAMERA_TROUBLESHOOTING.md`, `CHAT_CONTEXT.md`, `jetauto_stm32_architecture_report.md`, `jetauto_function_ids_documentation.md` y la memoria del proyecto.*
