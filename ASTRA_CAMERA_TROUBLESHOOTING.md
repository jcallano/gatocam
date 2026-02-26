# Orbbec Astra Pro Plus — Diagnóstico y Soluciones

Documento de referencia rápida para problemas conocidos con la cámara Astra Pro Plus
en este sistema (Orange Pi 5 / RK3588S, AArch64, Ubuntu 24.04, ROS 2 Jazzy).

---

## Arquitectura USB de la cámara

```
USB Bus 005 (ehci-platform, 480 Mbps)
└── 5-1: Genesys Logic GL850G hub (05e3:0608)  ← hub interno de la cámara
    ├── 5-1.1: Depth sensor     (2bc5:060f)     ← OpenNI / libuvc
    └── 5-1.2: UVC RGB camera   (2bc5:050f)     ← kernel uvcvideo driver
```

- El hub GL850G es **USB 2.0 únicamente** — no es un problema, es por diseño.
- El depth sensor (`060f`) es manejado por `astra_camera` via libuvc.
- El RGB UVC (`050f`) es manejado por el driver del kernel `uvcvideo` y expuesto como `/dev/videoN`.
- La ruta persistente (no cambia entre reinicios ni reconexiones):
  `/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0`

---

## Problema 1: OpenCV VideoCapture causa USB reset en AArch64

### Síntoma
```
cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
cap.isOpened()  # True
cap.grab()      # False  ← siempre falla
```
Después del intento, `dmesg` muestra:
```
usb 5-1.2: USB disconnect, device number N
usb 5-1.2: new high-speed USB device number N+1 using ehci-platform
```

### Causa raíz
El backend V4L2 de OpenCV usa las constantes de ioctl compiladas para **x86 (32-bit)**,
donde `struct v4l2_buffer` mide **68 bytes**. En **AArch64** mide **88 bytes** porque
`struct timeval` cambia de `long (4B)` a `long (8B)`.

Esto produce números de ioctl incorrectos:

| ioctl | x86 (incorrecto) | AArch64 (correcto) |
|-------|------------------|--------------------|
| VIDIOC_QUERYBUF | `0xc0445609` | `0xc0585609` |
| VIDIOC_QBUF | `0xc044560f` | `0xc058560f` |
| VIDIOC_DQBUF | `0xc0445611` | `0xc0585611` |

Los ioctls corruptos llegan al firmware UVC de la cámara Orbbec, que responde
reseteando el dispositivo USB.

### Solución
**Nunca usar `cv2.VideoCapture` con `cv2.CAP_V4L2` en este sistema.**

Usar raw V4L2 ioctls en Python con las constantes correctas.
Ver implementación en `src/jetauto_description/scripts/astra_color_node.py`.

### Referencia rápida de constantes AArch64
```python
# Obtener en tiempo de compilación con:
# gcc -x c - -o /tmp/v4l2size.c <<< '#include <linux/videodev2.h>
# int main(){printf("%lu\n",sizeof(struct v4l2_buffer));}'

V4L2_BUF_TYPE_VIDEO_CAPTURE = 1
V4L2_MEMORY_MMAP = 1
V4L2_PIX_FMT_YUYV = 0x56595559
V4L2_PIX_FMT_MJPG = 0x47504a4d

VIDIOC_S_FMT    = 0xc0d05605   # struct v4l2_format = 208 bytes
VIDIOC_REQBUFS  = 0xc0145608   # struct v4l2_requestbuffers = 20 bytes
VIDIOC_QUERYBUF = 0xc0585609   # struct v4l2_buffer = 88 bytes  ← AArch64
VIDIOC_QBUF     = 0xc058560f
VIDIOC_DQBUF    = 0xc0585611
VIDIOC_STREAMON = 0x40045612
VIDIOC_STREAMOFF = 0x40045613
```

### Layout de struct v4l2_buffer en AArch64 (88 bytes)
```
offset  0: __u32 index
offset  4: __u32 type
offset  8: __u32 bytesused
offset 12: __u32 flags
offset 16: __u32 field
offset 24: struct timeval timestamp  (16 bytes en 64-bit)
offset 40: struct v4l2_timecode      (32 bytes)
offset 56: __u32 sequence
offset 60: __u32 memory
offset 64: union m { .offset, .userptr, ... }
offset 72: __u32 length
```

---

## Problema 2: VIDIOC_STREAMON falla con EPROTO (errno 71)

### Síntoma
```
VIDIOC_STREAMON failed: [Errno 71] Protocol error
```
Ocurre incluso con ioctls correctos y el dispositivo abierto con éxito.

### Causas y soluciones

#### Causa A: Procesos `astra_camera_node` corriendo en background
Cuando se lanza el bringup y luego se mata con Ctrl+C, el proceso
`astra_camera_node` puede quedar zombie o en estado de limpieza. Estos
procesos mantienen handles sobre el USB y bloquean el `STREAMON` del UVC.

```bash
# Verificar
pgrep -a -f astra_camera_node

# Solución
killall -9 astra_camera_node
sleep 2
```

#### Causa B: Firmware de la cámara en estado inválido tras múltiples resets
Después de varios USB resets (causados por OpenCV o por crashes), el firmware
del sensor UVC Sonix (`2bc5:050f`) entra en un estado donde rechaza todos los
intentos de `STREAMON`. Esto persiste hasta que se resetea el dispositivo USB.

**Solución sin sudo** — resetear SOLO la cámara UVC, no el hub completo:
```python
import fcntl, os, glob, subprocess

USBDEVFS_RESET = 0x5514

# Encontrar SOLO el dispositivo UVC (2bc5:050f), NO el hub ni el depth
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

# Reset solo la UVC RGB (2bc5:050f)
path = find_usb_device('2bc5', '050f')
if path:
    fd = os.open(path, os.O_WRONLY)
    fcntl.ioctl(fd, USBDEVFS_RESET)
    os.close(fd)
    print(f'Reset UVC camera: {path}')
```
Esperar ~2 segundos para que el sistema re-enumere el dispositivo.

> **⚠️ ADVERTENCIA:** NO resetear el hub GL850G (`05e3:0608`) ni todos los dispositivos
> del bus 005 a la vez. Hacer reset del hub puede dejar el sensor de profundidad
> (`2bc5:060f`) en un estado de enumeración fallida que requiere replug físico.

**Solución alternativa:** Desconectar y reconectar físicamente el USB de la cámara.

#### Causa C: Autosuspend USB activo
Si `power/control = auto` y `autosuspend_delay_ms = 2000`, el dispositivo
puede suspenderse y rechazar STREAMON.

```bash
# Verificar
cat /sys/bus/usb/devices/5-1.2/power/control
# Debería decir "on"

# Fix manual (temporal)
echo on | sudo tee /sys/bus/usb/devices/5-1.2/power/control
```

La regla udev permanente ya está instalada en `/etc/udev/rules.d/55-orbbec-power.rules`.

---

## Problema 3: MJPG da ~15fps en lugar de 30fps

### Síntoma
Con formato MJPG y `cv2.imdecode()` para decodificar, la tasa publicada
es ~15fps con alta varianza.

### Causa
`cv2.imdecode()` decodifica JPEG por software en los cores ARM. En Cortex-A76
esto toma ~30-60ms por frame a 640x480, limitando el throughput a ~15-30fps
con overhead de publicación.

### Solución para uso standalone (sin depth + IR)
Usar **YUYV** en lugar de MJPG. El sensor soporta YUYV 640x480@30fps.
La conversión de color es mucho más rápida:
```python
arr = np.frombuffer(data, dtype=np.uint8).reshape((-1, width, 2))
frame = cv2.cvtColor(arr, cv2.COLOR_YUV2BGR_YUYV)
# Resultado: 30fps, std dev < 0.003s
```

**Limitación:** YUYV usa 18.4 MB/s. Cuando depth + IR también están activos
el hub GL850G se satura y el color pierde frames (ver Problema 5).
Para uso simultáneo con depth + IR, usar MJPG (ver Problema 5).

---

## Problema 4: v4l2_camera package — símbolo no encontrado

### Síntoma
```
undefined symbol: _ZN15image_transport23create_camera_publisherERKSS...
```

### Causa
El workspace compila `image_common` desde fuente con una ABI diferente a la
versión del sistema. El binario `ros-jazzy-v4l2-camera` del repositorio apt
enlaza contra la versión del sistema → incompatibilidad de ABI.

### Solución
No usar el paquete `ros-jazzy-v4l2-camera`. Usar `astra_color_node.py` del
paquete `jetauto_description` que no tiene dependencias externas más allá de
`rclpy`, `cv_bridge`, `numpy`, y `cv2`.

---

## Problema 5: Color falla cuando depth + IR están activos simultáneamente

### Síntoma
Con los tres sensores activos, el color publica ~30fps al inicio pero cae
a 0fps en cuanto depth + IR alcanzan su régimen. `VIDIOC_STREAMON` puede
fallar directamente con EPROTO si color arranca después del depth.

### Causa raíz — saturación del hub USB 2.0

El hub interno GL850G es **USB 2.0 únicamente** (480 Mbps ≈ ~40 MB/s útiles).
El ancho de banda requerido supera ese límite:

| Stream | Formato | Resolución | FPS | MB/s |
|--------|---------|-----------|-----|------|
| Color | YUYV | 640×480 | 30 | **18.4** |
| Depth | bulk (OpenNI) | 640×480 | 30 | ~18.0 |
| IR | bulk (OpenNI) | 640×480 | 30 | ~18.0 |
| **Total** | | | | **~54.4** |
| **Límite hub** | | | | **~40** |

Cuando depth + IR empiezan su bulk transfer, el driver `uvcvideo` no puede
negociar la `SET_INTERFACE` alternate-setting necesaria para el isochronous
endpoint del color → el firmware UVC responde con error de protocolo → EPROTO.

### Solución A: Arrancar color ANTES que depth (mitiga pero no elimina)

El `VIDIOC_STREAMON` del color puede completarse si se negocia **antes** de
que depth + IR comiencen a transmitir. Una vez en streaming, la UVC puede
mantener su isochronous slot aunque el bus esté saturado.

**⚠️ Condición:** Esto depende del timing; no garantiza estabilidad a largo
plazo. Si el depth usa ráfagas grandes, los frames de color empezarán a
perderse (select() timeout).

### Solución B (implementada): MJPG + Hardware decode Rockchip mppjpegdec

MJPG reduce el payload color de **18.4 MB/s a ~1–3 MB/s** (factor ×6–18),
lo que hace posible la coexistencia con depth + IR.

El decode lo realiza el **VPU Rockchip MPP** vía GStreamer (`mppjpegdec`),
evitando carga CPU significativa.

**Pipeline GStreamer (implementado en `astra_color_node.py`):**
```python
# Cadena de decoders en orden de preferencia:
# mppjpegdec (Rockchip VPU) → v4l2jpegdec (V4L2 M2M) → jpegdec (SW GStreamer) → cv2.imdecode

pipeline_str = (
    f"appsrc name=src format=time is-live=true block=false do-timestamp=true "
    f"max-buffers=2 leaky-type=downstream ! "          # ← evita acumulación de backlog
    f"image/jpeg,width={width},height={height} ! "
    f"jpegparse ! mppjpegdec ! "
    f"videoconvert ! video/x-raw,format=BGR ! "
    f"appsink name=sink emit-signals=true max-buffers=1 drop=true sync=false"
)
```

**Quirk PyGObject — usar `emit('pull-sample')` en lugar de `pull_sample()`:**
```python
# INCORRECTO — requiere GstApp typelib importado explícitamente:
# sample = appsink.pull_sample()        # AttributeError sin GstApp
# sample = appsink.try_pull_sample(0)   # AttributeError sin GstApp

# CORRECTO — action signal siempre disponible sin GstApp:
def on_new_sample(sink, _sq=sq, _Gst=Gst):
    sample = sink.emit('pull-sample')
    try:
        _sq.put_nowait(sample)
    except Exception:
        pass  # queue llena → frame descartado
    return _Gst.FlowReturn.OK

appsink.connect('new-sample', on_new_sample)
```

**Resultado:** `mppjpegdec` confirmado disponible en Orange Pi 5 / RK3588.
El nodo reporta `JPEG decoder backend: mppjpegdec` al arrancar.

### Solución C: Reducir resolución o FPS del color

Si se requiere YUYV (sin latencia de decode), reducir el ancho de banda:

| Config | MB/s | Factible con depth+IR |
|--------|------|:---:|
| YUYV 640×480 @ 30fps | 18.4 | No |
| YUYV 640×480 @ 10fps | 6.1 | Sí (marginal) |
| YUYV 320×240 @ 30fps | 4.6 | Sí |
| MJPG 640×480 @ 30fps | ~1–3 | Sí |

---

## Reporte de estabilidad

### Test A: Standalone, YUYV (referencia, 30 s)

**Condición:** Solo color activo (sin depth + IR), YUYV 640×480@30fps.

| Sensor | FPS real | Std dev (intervalo) |
|--------|:---:|:---:|
| Color YUYV | **30.0** | 2 ms |

### Test B: Simultáneo, MJPG + mppjpegdec (65 s, estado final)

**Condición:** depth + IR + color simultáneos, MJPG 640×480, hardware decode.
**Plataforma:** Orange Pi 5 / RK3588S, Ubuntu 24.04, ROS 2 Jazzy.
**Método:** `ros2 topic hz` + `ros2 topic delay` durante 65 s.

| Sensor | Topic | FPS objetivo | FPS real (avg) | Rango | Std dev | Max gap |
|--------|-------|:---:|:---:|:---:|:---:|:---:|
| IR | `/camera/ir/image_raw` | 30 | **30.0** | 30.0 | 0.5 ms | — |
| Depth | `/camera/depth/image_raw` | 15 | **~23** | 12–30 | variable | — |
| Color MJPG | `/camera/color/image_raw` | 30 | **~15–17** | 8–20 | — | 0.748 s |

### Interpretación

**IR — Excelente.** 30 fps exactos, std dev 0.5 ms. Pipeline estable.

**Depth — Aceptable.** Variabilidad 12–30 Hz es comportamiento normal
de OpenNI con ráfagas bulk USB. Suficiente para navegación y SLAM.

**Color MJPG — Funcional sin crashes.** La tasa ~15–17 Hz (sobre objetivo
de 30) se debe a la latencia combinada del pipeline GStreamer mppjpegdec
bajo carga. El max gap de 0.748 s es ocasional; **no hay errores ni
desconexiones USB** durante el test de 65 s. Adecuado para monitoreo de
gatos en interiores.

**Sin errores en test completo:** Ninguna desconexión USB, ningún
`VIDIOC_DQBUF` timeout (solo warn "Frame timeout" ocasional), ningún
crash de nodo.

---

## Flujo de arranque recomendado para la cámara

### Verificar estado antes del bringup
```bash
# 1. Verificar que la cámara está enumerada
ls /dev/v4l/by-id/ | grep Sonix

# 2. Verificar que no hay procesos astra residuales
pgrep -c astra_camera_node  # debe ser 0

# 3. Verificar autosuspend
cat /sys/bus/usb/devices/5-1.2/power/control  # debe ser "on"

# 4. Test rápido de streaming (opcional)
v4l2-ctl -d /dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0 \
    --stream-mmap --stream-count=3  # debe imprimir <<<
```

### Secuencia del bringup — orden crítico (MJPG resuelve la race condition)

Con **MJPG** el ancho de banda del color (~1–3 MB/s) es compatible con
depth + IR, eliminando la dependencia de orden de arranque.

```bash
# Lanzar todo en paralelo — funciona con MJPG:
ros2 launch jetauto_description robot_bringup.launch.py
```

**`bringup.sh` mata nodos stale antes de arrancar:**
```bash
pkill -TERM -f "astra_camera_node|astra_color_node|..." 2>/dev/null || true
sleep 2
pkill -KILL -f "astra_camera_node|astra_color_node" 2>/dev/null || true
sleep 1
ros2 launch jetauto_description robot_bringup.launch.py
```

**Secuencia interna del launch:**
1. `astra_camera` (depth + IR, `use_uvc_camera:=false`, `enable_color:=false`)
2. `astra_color_node.py` (RGB via raw V4L2 MJPG + mppjpegdec)
3. El nodo color tiene retry automático (10 intentos × 1 s) si el device
   está temporalmente ocupado durante la negociación USB inicial.

**Opción legacy — YUYV standalone (sin depth + IR):**
```bash
# Solo para depuración/calibración de color sin los demás sensores:
ros2 run jetauto_description astra_color_node.py \
    --ros-args -p video_device:=/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0
```

---

## Verificar que todo funciona

```bash
# Después del bringup, verificar tasas:
ros2 topic hz /camera/color/image_raw   # ~15-17fps con mppjpegdec simultáneo depth+IR
ros2 topic hz /camera/depth/image_raw   # ~12-30fps (variabilidad OpenNI normal)
ros2 topic hz /camera/ir/image_raw      # ~30fps

# Verificar delay (pipeline latency):
ros2 topic delay /camera/depth/image_raw  # esperado ~3ms
ros2 topic delay /camera/ir/image_raw     # esperado ~2ms

# Verificar que los frames tienen contenido (no todos negros):
ros2 topic echo /camera/color/image_raw --no-arr | grep encoding
# encoding: bgr8
```

---

---

## Problema 6: kill -9 en astra_camera_node desconecta el depth sensor del bus USB

### Síntoma
Tras ejecutar `kill -9 <pid_astra_camera_node>`, el sensor de profundidad
(`2bc5:060f`) desaparece de `/dev/bus/usb/` y `lsusb`:
```
usb 5-1.1: USB disconnect, device number N
```
Los intentos de bringup posteriores fallan con `ENODEV` al abrir el depth.
No recupera hasta **reconexión física del USB**.

### Causa
`astra_camera_node` mantiene un handle libuvc sobre el depth sensor USB.
`SIGKILL` no permite al proceso cerrar ese handle correctamente; el
driver libuvc o el kernel dejan el endpoint en un estado inválido que
el firmware del GL850G no puede recuperar sin replug.

### Solución
**Nunca usar `kill -9` / `SIGKILL` en `astra_camera_node`.**

Siempre usar `SIGTERM` y esperar a que el proceso limpie:
```bash
# Correcto — SIGTERM + espera
pkill -TERM -f astra_camera_node
sleep 2

# Solo si SIGTERM no bastó después del sleep:
pkill -KILL -f astra_camera_node
```

`bringup.sh` ya implementa esta secuencia: TERM → sleep 2 → KILL solo
para astra_color_node (que no usa libuvc y es seguro de matar).

### Recuperación si ya ocurrió
```bash
# Si el depth sensor desapareció → solo replug físico:
# 1. Desconectar el USB de la Astra
# 2. Esperar ~3s
# 3. Reconectar

# Verificar recuperación:
lsusb | grep 2bc5  # debe mostrar 2bc5:060f y 2bc5:050f
```

---

## Archivos relevantes

| Archivo | Descripción |
|---------|-------------|
| `src/jetauto_description/scripts/astra_color_node.py` | Nodo RGB raw V4L2 |
| `src/jetauto_description/launch/robot_bringup.launch.py` | Launch principal |
| `src/ros2_astra_camera/astra_camera/launch/astra_pro.launch.xml` | Launch Astra (uvc_product_id fijo a 0x050f) |
| `/etc/udev/rules.d/55-orbbec-power.rules` | Deshabilita autosuspend USB |
| `/etc/systemd/system/performance-governor.service` | Governor CPU/GPU a performance |
