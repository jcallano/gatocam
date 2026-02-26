# Contexto resumido de este chat

## Objetivo
Integrar y depurar sensores (Astra + RPLidar), STM32 por serial, cinemática Mecanum y RViz en ROS 2 Jazzy sobre Orange Pi 5.

## Estado logrado
- **STM32 (robot controller)** funciona en `/dev/ttyACM0` a 1 Mbps.
- **Astra** corre con `ros2_astra_camera` y publica `/camera/depth/points`, `/camera/ir/image_raw`.
- **RPLidar A1** corre en `/dev/ttyUSB0`.
  - **Estabilidad (`/scan`)**: Frecuencia muy estable (7.12 Hz, std dev 7ms), latencia pipeline (delay) ~141 ms. (Nota: 7Hz es normal para A1 por USB).
- **TF** completo desde `robot_description` cuando se exportan:
  - `LIDAR_TYPE=A1`, `MACHINE_TYPE=JetAuto`, `DEPTH_CAMERA_TYPE=Astra`.
- **LIDAR** alineado usando `frame_id:=lidar_frame`.
- **PointCloud2** requiere conexión de frame entre `depth_cam_link` y `camera_link` (ya integrado en `robot_description.launch.py`).
- **RViz** tiene config lista: `src/jetauto_description/rviz/sensors.rviz`.
- **Bringup unificado**: `jetauto_description/launch/robot_bringup.launch.py` (por defecto `use_rviz:=false`).

## Cambios clave en código
- `ros_robot_controller/ros_robot_controller_node.py`: parámetros `port`, `baudrate`, `timeout`, DTR/RTS reset, log de configuración.
- `ros_robot_controller/launch/ros_robot_controller.launch.py`: acepta parámetros de serial.
- `controller/odom_publisher_node.py`: parámetros `wheelbase`, `track_width`, `wheel_diameter`, `machine_type` con fallback.
- `controller/config/calibrate_params.yaml`: incluye wheelbase/track_width/wheel_diameter.
- `jetauto_description/launch/robot_description.launch.py`: añade `static_transform_publisher` para `depth_cam_link -> camera_link`.
- `jetauto_description/launch/robot_bringup.launch.py`: unifica robot_description + lidar + astra + rviz opcional.

## Nodos Creados Localmente
### 1. `jetauto_teleop_joy.py` (Mando a Actuadores)
Conecta las entradas crudas de `/ros_robot_controller/joy` con los motores Mecanum, el Paneo de Cámara y el Buzzer, incluyendo funciones de "Embrague" de velocidad y apagado de emergencia.
* **Ubicación:** `/home/jcallano/ros2_ws/src/jetauto_description/scripts/jetauto_teleop_joy.py`
* **Dependencias:** `ros_robot_controller` debe estar vivo.

**Mapeo de Teleoperación:**
- **🕹️ Stick Izquierdo:** Avanzar / Retroceder y Strafing Mecanum lateral puro.
- **🕹️ Stick Derecho (Eje X):** Rotar sobre su propio eje.
- **🎮 DPAD (Izquierda/Derecha):** Girar la cámara Astra (Pan).
- **🕹️ L3 (Click Stick Izq):** Auto-centrado rápido de la cámara.
- **🖲️ L1 (Bumper Izq):** Tocar la bocina corta (Buzzer interactivo).
- **🚥 L2 / R2 (Gatillos):**
  - **L2:** Modo Precisión (Reduce toda la velocidad al 25% mientras se presiona).
  - **R2:** Modo Turbo (Multiplica toda la velocidad x2 mientras se presiona).
- **🛑 START + SELECT:** Emergency Stop (Detiene la tracción de las 4 ruedas automáticamente).

## Scripts de Utilidad

## Mapeo de motores (verificado)
- ID1 = FL adelante (CCW)
- ID2 = RL adelante
- ID3 = FR atrás

## Mapeo de botones Gamepad (Estilo Xbox/PS)
El tópico `/ros_robot_controller/joy` expone exactamente los siguientes índices (ya validados por hardware):
- **Ejes de movimiento**: Joystick Izquierdo (X=`axes[0]`, Y=`axes[1]`), Joystick Derecho (X=`axes[2]`, Y=`axes[3]`).
- **Gatillos y DPad**: `L2`=`axes[5]`, `R2`=`axes[4]`, DPad (X=`axes[6]`, Y=`axes[7]`).
- **Botones Principales**: `A`=`buttons[0]`, `B`=`buttons[1]`, `X`=`buttons[3]`, `Y`=`buttons[4]`.
- **Botones Secundarios**: `L1`=`buttons[6]`, `R1`=`buttons[7]`, `Select`=`buttons[10]`, `Start`=`buttons[11]`, `L3`=`buttons[13]`, `R3`=`buttons[14]`.

## Servos (Actuadores Auxiliares)
- La única articulación libre del robot es la base de la cámara Astra (Paneo).
- Controlado por: `/ros_robot_controller/bus_servo/set_position`
- **ID de Servo:** `1`
- **Rango:** `200` (Derecha) -> `500` (Centro) -> `800` (Izquierda)

## Tópicos Auxiliares STM32
Además de la base móvil y la cámara, la placa controladora expone los siguientes actuadores/sensores:

- **Batería (`std_msgs/UInt16`)**: `/ros_robot_controller/battery` (En milivoltios, ej: 11500 = 11.5V)
- **Botones de Placa (`ros_robot_controller_msgs/ButtonState`)**: `/ros_robot_controller/button` 
  - `id`: 1 (KEY1) o 2 (KEY2).
  - `state`: 1=Pulso, 2=Sostenido.
- **Buzzer (`ros_robot_controller_msgs/BuzzerState`)**: `/ros_robot_controller/set_buzzer`
  - Requiere: `freq` (Hz), `on_time` (s), `off_time` (s), `repeat` (contador).
- **LED RGB (`ros_robot_controller_msgs/LedState`)**: `/ros_robot_controller/set_led`
  - Requiere: `id` (usualmente 1), `on_time`, `off_time`, `repeat`.
- **Pantalla OLED (`ros_robot_controller_msgs/OLEDState`)**: `/ros_robot_controller/set_oled`
  - Requiere: `index` (línea), `text` (String a mostrar).

## Comandos clave
### STM32
```
export PYTHONPATH=/home/jcallano/ros2_ws/.venv/lib/python3.12/site-packages:$PYTHONPATH
source /opt/ros/jazzy/setup.bash
source /home/jcallano/ros2_ws/install/setup.bash
ros2 launch ros_robot_controller ros_robot_controller.launch.py port:=/dev/ttyACM0 baudrate:=1000000
```

### Robot description (TF completo)
```
export LIDAR_TYPE=A1
export MACHINE_TYPE=JetAuto
export DEPTH_CAMERA_TYPE=Astra
need_compile=True ros2 launch jetauto_description robot_description.launch.py use_gui:=false use_rviz:=false use_sim_time:=false
```

### RPLidar A1
```
ros2 launch rplidar_ros rplidar_a1_launch.py serial_port:=/dev/ttyUSB0 frame_id:=lidar_frame
```

### Astra
```
export LD_LIBRARY_PATH=/home/jcallano/ros2_ws/libuvc_install/lib:$LD_LIBRARY_PATH
ros2 launch astra_camera astra.launch.xml
```

### Bringup unificado (sin RViz)
```
ros2 launch jetauto_description robot_bringup.launch.py
```

### RViz (manual)
```
rviz2 -d /home/jcallano/ros2_ws/src/jetauto_description/rviz/sensors.rviz
```

### Movimiento de Servo Cámara (Paneo)
```
# Va al centro en 1 segundo
ros2 topic pub --once /ros_robot_controller/bus_servo/set_position ros_robot_controller_msgs/msg/ServosPosition "{duration: 1.0, position: [{id: 1, position: 500}]}"
```

### Pitido del Buzzer
```
ros2 topic pub --once /ros_robot_controller/set_buzzer ros_robot_controller_msgs/msg/BuzzerState "{freq: 2000, on_time: 0.1, off_time: 0.1, repeat: 2}"
```

### Correr Teleoperación Gamepad Autorizada
Asegura que `ros_robot_controller` esté vivo primero.
```bash
python3 /home/jcallano/ros2_ws/src/jetauto_description/scripts/jetauto_teleop_joy.py
```

## Notas
- RViz a veces no abre desde launch por temas de GUI/memoria. Abrir manualmente.
- `bringup.sh` en la raíz del workspace ejecuta el bringup unificado con envs correctos.
- **Troubleshooting RPLidar**: Si lanza `Error, operation time out. SL_RESULT_OPERATION_TIMEOUT!`, revisar si quedó un proceso `rplidar_node` en background o zombie bloqueando el bus (`killall -9 rplidar_node`).

---

## Sesion 2026-02-23 21:59:49 +04
### Estado actual
- **TF**: intermitencia de `PointCloud2` se reduce al fijar `Fixed Frame` a `camera_depth_optical_frame` (evita cuello de botella TF).
- **RGB**: topic `/camera/color/image_raw` ahora tiene publisher (1) y subscriber (RViz). En RViz, el display `RGB Image` puede mostrar "no image" si el `Transport Hint` no esta en `raw`.

### Cambios en RViz
- `src/jetauto_description/rviz/sensors.rviz`:
  - `Fixed Frame` global: `camera_depth_optical_frame`.
  - `PointCloud2` `Depth` = 10, `Reliability` = Best Effort.
  - Nuevo display `RGB Image` habilitado en `/camera/color/image_raw`.

### Cambios en bringup
- `src/jetauto_description/launch/robot_bringup.launch.py`:
  - `astra_launch` por defecto ahora es `astra_pro.launch.xml`.
  - Parametros forzados para Astra:
    - `enable_color=true`, `color_width=640`, `color_height=480`, `color_fps=30`
    - `depth_width=640`, `depth_height=480`, `depth_fps=15`
    - `enable_ir=true`, `ir_width=640`, `ir_height=480`, `ir_fps=30`
    - `enable_point_cloud=true`
    - `use_uvc_camera=true`, `uvc_product_id=0x050f`

### Build
- Ejecutado: `colcon build --packages-select jetauto_description`

### Problemas pendientes
- RViz: `RGB Image` a veces muestra "no image". Verificar:
  - Display `RGB Image` -> `Topic` = `/camera/color/image_raw`
  - `Transport Hint` = `raw`
- Pendiente: definir e implementar **SLAM 3D robusto** usando todos los sensores (Astra depth/RGB, Lidar, odometria, IMU), y **validar consumo de recursos** para confirmar si queda margen para **deteccion de gatos con NPU**.



# 🛠️ Configuración de Hardware - Orange Pi 5 (RK3588S)
**Proyecto:** Robot JetAuto / SLAM 3D & Cat Tracking
**Ubicación:** Omán | **Entorno:** Sala 100m²

## 1. Device Tree Overlays - Estado Final

Archivo: `/boot/armbianEnv.txt`
```
overlays=dmc-oc-3500mhz opp-oc-24ghz
```

| Overlay | Estado | Motivo |
| :--- | :--- | :--- |
| `dmc-oc-3500mhz` | **ACTIVO** | DDR a 3500 MHz. Evita micro-congelamientos en streams de cámara. Sin errores de kernel. |
| `opp-oc-24ghz` | **ACTIVO** | Habilita 2.4 GHz en cores A76. Genera errores `pvtpll voltsel=-1` no fatales. Sin él los Big cores capan. |
| `panthor-gpu` | **DESACTIVADO** | Causaba inestabilidad con Antigravity. Driver Mali DDK activo en su lugar. |
| `khadas-edge2-cam3` | **ELIMINADO** | Era overlay de otra placa (Khadas Edge2). Causaba errores `rkcif MIPI sensor failed` cosméticos. |

## 2. Errores de Kernel conocidos y su estado

| Error | Componente | Causa | Acción |
| :--- | :--- | :--- | :--- |
| `pvtpll voltsel=-1` clk_id=0,2,3,5,6 | CPU, GPU, NPU | `opp-oc-24ghz` sin entradas PVTPLL para pvtm-volt-sel=2/3 | **Aceptado** - no fatal, governor performance lo mitiga |
| `Unsupported VOP aclk dvfs` | Salida de video | Limitación del driver | **Ignorar** - cosmético |
| `dw9714 i2c write failed -6` | Driver cámara MIPI | Driver buscando hardware inexistente | **Ignorar** - no afecta Astra USB |
| `rkcif MIPI sensor failed` | MIPI cam | `khadas-edge2-cam3` eliminado → desaparecerá en próximo reinicio | **Resuelto** |

## 3. Optimización del Sistema - Performance Governor

Governor `performance` aplicado y persistente vía systemd:

```bash
# Servicio instalado en: /etc/systemd/system/performance-governor.service
# Estado: enabled, active
```

| Componente | Governor | Frecuencia fija |
| :--- | :--- | :--- |
| CPU policy0 (LITTLE, cores 0-3) | performance | 1.8 GHz |
| CPU policy4 (Big, cores 4-5) | performance | 2.4 GHz |
| CPU policy6 (Big, cores 6-7) | performance | 2.4 GHz |
| GPU Mali (`fb000000.gpu`) | performance | 1.0 GHz |
| NPU RKNPU (`fdab0000.npu`) | rknpu_ondemand | hasta 1.0 GHz |

> **Nota:** La NPU usa `rknpu_ondemand` (no `performance`) porque escala automáticamente bajo carga - comportamiento correcto para inferencia.

## 4. Hardware USB - Cámara Astra Pro Plus

- **Hub interno:** Genesys Logic GL850G (`05e3:0608`) - USB 2.0 únicamente (hardware de la cámara no soporta USB 3.0)
- **RGB UVC:** `2bc5:050f` | Driver: `uvcvideo` | Bus 005 (480M)
- **Depth Sensor:** `2bc5:060f` | Driver: `[none]` (normal - manejado por `libuvc` en userspace)
- **NPU:** `/dev/dri/renderD129` | Driver: `rknpu 0.9.8` (built-in kernel) | **OPERATIVA**
- USB 2.0 a 480 Mbps es suficiente para RGB 640x480@30fps + Depth@15fps

## 5. Mapa de puertos USB del Orange Pi 5 (RK3588S)

| Bus | Controlador | Velocidad | Uso actual |
| :--- | :--- | :--- | :--- |
| Bus 002 | xhci / usbhost3_0 | 5000M (USB 3.0) | Libre |
| Bus 005 | ehci / fc880000.usb | 480M (USB 2.0) | Astra Pro Plus |
| Bus 007 | xhci / usbdrd3_0 | 480M | Logitech Unifying |
| Bus 003 | ehci / fc800000.usb | 480M (USB 2.0) | Hub JetAuto (STM32 + RPLidar) |

## 6. Pipeline de Visión por Computadora (YOLOv8 + RK3588 NPU)

Se ha implementado una arquitectura de detección de objetos (gatos/obstáculos) que corre nativamente en la NPU del Rockchip (Orange Pi 5) para no saturar la CPU, usando el formato `.rknn`.

### A. Preparación del Modelo (Realizado en PC x86 Ubuntu remoto)
Para compilar un modelo ONNX a RKNN, es necesario un entorno x86 con la versión completa del Toolkit de Rockchip.

**Dependencias y proceso:**
1. Instalar dependencias estrictas de PyTorch y Rockchip (Python 3.12 compatible con PIP/Wheel):
   - `pip install torch==2.4.0 torchvision==0.19.0 onnx==1.16.1`
   - Ojo: Versiones más nuevas de ONNX (1.20+) rompen el compilador `rknn-toolkit2` por deprecación del atributo `mapping`.
2. Exportar modelo PyTorch estandarizado a ONNX:
   ```python
   from ultralytics import YOLO
   model = YOLO("yolov8n.pt")
   model.export(format="onnx", opset=12) # Opset 12 obligatorio para RKNN
   ```
3. Instalar `rknn-toolkit2` en el PC Remoto y correr el script local `build_rknn.py`:
   ```python
   # build_rknn.py
   from rknn.api import RKNN
   rknn = RKNN(verbose=True)
   rknn.config(mean_values=[[0, 0, 0]], std_values=[[255, 255, 255]], target_platform='rk3588')
   rknn.load_onnx(model='yolov8n.onnx')
   rknn.build(do_quantization=False) # RK3588 soporta i8 o fb16 nativo
   rknn.export_rknn('yolov8n.rknn')
   ```

### B. Ejecución en NPU Local (JetAuto / Orange Pi 5)
Una vez transferido el archivo final `yolov8n.rknn` al robot, el despliegue requiere las librerías "Lite" y los drivers nativos de Rockchip.

1. **Drivers de NPU (Hardware Layer):**
   El sistema base Ubuntu debe poseer el archivo `librknnrt.so` en `/usr/lib/`. 
   Se puede descargar de la repo oficial de Rockchip (`rknpu2/runtime/Linux/librknn_api/aarch64/librknnrt.so`).

2. **Dependencias del Workspace (Python 3.12 en `.venv`):**
   - El nodo de inferencia necesita: `pip install "numpy<2" rknn-toolkit-lite2 opencv-python cv-bridge`
   - Note: NumPy 2.0+ genera SegFault en la API Rockchip Lite de C++. Obligatorio pinear a `1.26.4` o menor.

3. **Nodo de ROS 2 (`jetauto_yolo_rknn`)**:
   - Suscrito a `/camera/color/image_raw`.
   - Utiliza `cv_bridge` para parsear imagen.
   - Envía el Tensor al acelerador NPU (`self.rknn_lite.inference(inputs)`).
   - Traduce los índices YOLO a BoundingBoxes usando NMS y los publica como `vision_msgs/Detection2DArray` a la red ROS.

**Ejecución de la inferencia AI:**
```bash
ros2 run jetauto_description jetauto_yolo_rknn.py
```
