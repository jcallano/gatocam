#!/usr/bin/env python3
"""
system_monitor_node — Métricas de rendimiento del Orange Pi 5 (RK3588S).

Publica:
  /system_monitor/diagnostics  (diagnostic_msgs/DiagnosticArray) @ 1 Hz

Subsistemas monitorizados:
  - CPU    : uso % por cluster (A55/A76), frecuencia, temperatura
  - Memoria: RAM usada/disponible, swap
  - GPU/NPU: carga %, temperatura
  - Red    : TX/RX MB/s en end1
  - Tópicos: Hz real de /scan, /camera/color/image_raw,
             /camera/depth/image_raw, /odom_raw
"""

import os
import re
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Header

try:
    import psutil
    _PSUTIL = True
except ImportError:
    _PSUTIL = False


# ── helpers /sys ──────────────────────────────────────────────────────────────

def _read_sys(path: str):
    """Lee un archivo /sys o /proc; retorna el string crudo o None si falla."""
    try:
        with open(path) as f:
            return f.read().strip()
    except Exception:
        return None


def _read_temp_c(path: str) -> float:
    """Convierte milikelvin a °C desde un archivo thermal_zone/temp."""
    raw = _read_sys(path)
    return float(raw) / 1000.0 if raw else -1.0


def _read_freq_mhz(policy: int) -> float:
    path = f'/sys/devices/system/cpu/cpufreq/policy{policy}/scaling_cur_freq'
    raw = _read_sys(path)
    return float(raw) / 1000.0 if raw else -1.0  # kHz → MHz


# ── nodo ─────────────────────────────────────────────────────────────────────

class SystemMonitorNode(Node):

    # Umbrales WARN / ERROR para cada métrica
    _THRESHOLDS = {
        'cpu_load':    (80.0,  95.0),   # %
        'cpu_temp':    (80.0,  90.0),   # °C
        'gpu_load':    (85.0,  95.0),
        'gpu_temp':    (80.0,  90.0),
        'ram_avail':   (300.0, 150.0),  # MB  (low=malo)
        'swap_used':   (100.0, 500.0),  # MB
        'hz_scan':     (5.0,   1.0),    # Hz  (low=malo)
        'hz_image':    (10.0,  5.0),
        'hz_odom':     (20.0,  5.0),
    }

    def __init__(self):
        super().__init__('system_monitor')

        # Publisher
        self._pub = self.create_publisher(
            DiagnosticArray, '/system_monitor/diagnostics', 10)

        # Subscripciones para medir Hz de tópicos clave
        # QoS best-effort para no bloquear si el sensor no publica
        be_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._hz_windows: dict[str, list[float]] = {
            '/scan': [],
            '/camera/color/image_raw': [],
            '/camera/depth/image_raw': [],
            '/odom_raw': [],
        }
        self.create_subscription(
            LaserScan, '/scan',
            lambda _: self._stamp('/scan'), be_qos)
        self.create_subscription(
            Image, '/camera/color/image_raw',
            lambda _: self._stamp('/camera/color/image_raw'), be_qos)
        self.create_subscription(
            Image, '/camera/depth/image_raw',
            lambda _: self._stamp('/camera/depth/image_raw'), be_qos)
        self.create_subscription(
            Odometry, '/odom_raw',
            lambda _: self._stamp('/odom_raw'), be_qos)

        # Red: contador previo para delta
        self._net_prev = None
        self._net_time_prev: float = 0.0

        # Mapa thermal_zone_type → path al archivo temp
        self._thermal = self._discover_thermal_zones()
        self.get_logger().info(
            f'Zonas térmicas detectadas: {list(self._thermal.keys())}')

        # Inicializar psutil (primera llamada descartada)
        if _PSUTIL:
            psutil.cpu_percent(percpu=True, interval=None)

        self.create_timer(1.0, self._publish)
        self.get_logger().info('system_monitor iniciado → /system_monitor/diagnostics')

    # ── Hz tracking ──────────────────────────────────────────────────────────

    def _stamp(self, topic: str) -> None:
        now = time.monotonic()
        win = self._hz_windows[topic]
        win.append(now)
        cutoff = now - 10.0
        # Purgar entradas antiguas (mínimo)
        if len(win) > 2 and win[0] < cutoff:
            self._hz_windows[topic] = [t for t in win if t >= cutoff]

    def _hz(self, topic: str) -> float:
        win = self._hz_windows[topic]
        now = time.monotonic()
        recent = [t for t in win if t >= now - 5.0]
        if len(recent) < 2:
            return 0.0
        return (len(recent) - 1) / (recent[-1] - recent[0])

    # ── /sys helpers ─────────────────────────────────────────────────────────

    def _discover_thermal_zones(self) -> dict[str, str]:
        zones: dict[str, str] = {}
        base = '/sys/class/thermal'
        try:
            for entry in sorted(os.listdir(base)):
                if not entry.startswith('thermal_zone'):
                    continue
                type_path = os.path.join(base, entry, 'type')
                temp_path = os.path.join(base, entry, 'temp')
                zone_type = _read_sys(type_path)
                if zone_type:
                    zones[zone_type] = temp_path
        except Exception:
            pass
        return zones

    def _temp(self, *keywords) -> float:
        """Temperatura °C de la primera zona cuyo tipo contenga algún keyword."""
        for kw in keywords:
            for zone_type, path in self._thermal.items():
                if kw in zone_type.lower():
                    return _read_temp_c(path)
        return -1.0

    def _npu_load(self) -> float:
        # Opción 1: debugfs — fuente fiable, requiere root o CAP_SYS_ADMIN
        # Formato: "NPU load:  Core0: 37%, Core1:  0%, Core2:  0%,"
        raw = _read_sys('/sys/kernel/debug/rknpu/load')
        if raw:
            vals = re.findall(r'(\d+)%', raw)
            if vals:
                return sum(float(v) for v in vals) / len(vals)
        # Opción 2: devfreq — NOTA: en RK3588S el governor RKNPU reporta
        # "100@freq_hz" cuando el firmware está cargado aunque la NPU esté
        # inactiva. El valor solo es útil cuando debugfs no está disponible.
        for path in (
            '/sys/class/devfreq/fdab0000.npu/load',
            '/sys/class/devfreq/npu/load',
        ):
            raw = _read_sys(path)
            if raw:
                try:
                    return float(raw.split('@')[0])
                except ValueError:
                    pass
        return -1.0

    def _gpu_load(self) -> float:
        for path in (
            '/sys/class/devfreq/fb000000.gpu/load',
            '/sys/devices/platform/fb000000.gpu/devfreq/fb000000.gpu/load',
        ):
            raw = _read_sys(path)
            if raw:
                try:
                    return float(raw.split('@')[0])
                except ValueError:
                    pass
        return -1.0

    def _net_delta(self) -> tuple[float, float]:
        """Retorna (tx_mbps, rx_mbps) en la interfaz end1/eth0."""
        if not _PSUTIL:
            return -1.0, -1.0
        now = time.monotonic()
        try:
            counters = psutil.net_io_counters(pernic=True)
            iface = counters.get('end1') or counters.get('eth0')
            if iface is None:
                return -1.0, -1.0
            if self._net_prev is None:
                self._net_prev = iface
                self._net_time_prev = now
                return 0.0, 0.0
            dt = max(now - self._net_time_prev, 1e-3)
            tx = max(0.0, (iface.bytes_sent - self._net_prev.bytes_sent) / dt / 1e6)
            rx = max(0.0, (iface.bytes_recv - self._net_prev.bytes_recv) / dt / 1e6)
            self._net_prev = iface
            self._net_time_prev = now
            return tx, rx
        except Exception:
            return -1.0, -1.0

    # ── utilidades DiagnosticStatus ───────────────────────────────────────────

    @staticmethod
    def _kv(key: str, value: float, fmt: str = '.1f', unit: str = '') -> KeyValue:
        return KeyValue(key=key, value=f'{value:{fmt}}{unit}')

    def _level(self, value: float, warn: float, error: float,
               low: bool = False) -> int:
        """Nivel OK/WARN/ERROR según umbrales. Si value<0 → WARN (dato no disponible)."""
        if value < 0:
            return DiagnosticStatus.WARN
        if low:
            if value <= error:
                return DiagnosticStatus.ERROR
            if value <= warn:
                return DiagnosticStatus.WARN
        else:
            if value >= error:
                return DiagnosticStatus.ERROR
            if value >= warn:
                return DiagnosticStatus.WARN
        return DiagnosticStatus.OK

    # ── publicación principal ─────────────────────────────────────────────────

    def _publish(self) -> None:
        msg = DiagnosticArray()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [
            self._status_cpu(),
            self._status_memory(),
            self._status_gpu_npu(),
            self._status_network(),
            self._status_topics(),
        ]
        self._pub.publish(msg)

    # ── subsistema CPU ────────────────────────────────────────────────────────

    def _status_cpu(self) -> DiagnosticStatus:
        s = DiagnosticStatus()
        s.name = 'CPU'
        s.hardware_id = 'RK3588S'

        if _PSUTIL:
            per = psutil.cpu_percent(percpu=True, interval=None)  # 8 valores
            a55 = per[:4]   # policy0: cores 0-3 (Cortex-A55)
            a76 = per[4:]   # policy4+6: cores 4-7 (Cortex-A76)
            avg55 = sum(a55) / 4
            avg76 = sum(a76) / 4
        else:
            a55 = a76 = []
            avg55 = avg76 = -1.0

        freq55 = _read_freq_mhz(0)
        freq76 = _read_freq_mhz(4)
        temp55   = self._temp('littlecore')
        temp76_0 = self._temp('bigcore0')
        temp76_1 = self._temp('bigcore1')
        temp76   = max(temp76_0, temp76_1)  # peor caso A76

        s.level = max(
            self._level(avg76, *self._THRESHOLDS['cpu_load']),
            self._level(temp76, *self._THRESHOLDS['cpu_temp']),
        )
        s.message = (
            f'A55={avg55:.0f}% @{freq55:.0f}MHz {temp55:.1f}°C | '
            f'A76={avg76:.0f}% @{freq76:.0f}MHz {temp76:.1f}°C'
        )
        s.values = [
            self._kv('A55 uso medio',  avg55,   unit='%'),
            self._kv('A76 uso medio',  avg76,   unit='%'),
            self._kv('A55 frecuencia', freq55,  unit=' MHz'),
            self._kv('A76 frecuencia', freq76,  unit=' MHz'),
            self._kv('A55 temp',       temp55,  unit='°C'),
            self._kv('A76 bigcore0',   temp76_0, unit='°C'),
            self._kv('A76 bigcore1',   temp76_1, unit='°C'),
        ]
        for i, v in enumerate(a55):
            s.values.append(self._kv(f'  A55 core{i}', v, unit='%'))
        for i, v in enumerate(a76):
            s.values.append(self._kv(f'  A76 core{i}', v, unit='%'))
        return s

    # ── subsistema Memoria ────────────────────────────────────────────────────

    def _status_memory(self) -> DiagnosticStatus:
        s = DiagnosticStatus()
        s.name = 'Memoria'
        s.hardware_id = 'RK3588S'

        if _PSUTIL:
            vm = psutil.virtual_memory()
            sw = psutil.swap_memory()
            used_mb  = vm.used      / 1e6
            avail_mb = vm.available / 1e6
            total_mb = vm.total     / 1e6
            swap_mb  = sw.used      / 1e6
            pct      = vm.percent
        else:
            used_mb = avail_mb = total_mb = swap_mb = pct = -1.0

        s.level = max(
            self._level(avail_mb, *self._THRESHOLDS['ram_avail'], low=True),
            self._level(swap_mb,  *self._THRESHOLDS['swap_used']),
        )
        s.message = (
            f'RAM {used_mb:.0f}/{total_mb:.0f} MB ({pct:.0f}%) | '
            f'Swap {swap_mb:.0f} MB'
        )
        s.values = [
            self._kv('RAM usada',       used_mb,  unit=' MB'),
            self._kv('RAM disponible',  avail_mb, unit=' MB'),
            self._kv('RAM total',       total_mb, unit=' MB'),
            self._kv('RAM uso',         pct,      unit='%'),
            self._kv('Swap usada',      swap_mb,  unit=' MB'),
        ]
        return s

    # ── subsistema GPU/NPU ────────────────────────────────────────────────────

    def _status_gpu_npu(self) -> DiagnosticStatus:
        s = DiagnosticStatus()
        s.name = 'GPU_NPU'
        s.hardware_id = 'RK3588S Mali-G610 / RKNPU2'

        gpu = self._gpu_load()
        npu = self._npu_load()
        t_gpu = self._temp('gpu', 'mali')
        t_npu = self._temp('npu')

        s.level = max(
            self._level(gpu, *self._THRESHOLDS['gpu_load']),
            self._level(t_gpu, *self._THRESHOLDS['gpu_temp']),
        )
        s.message = (
            f'GPU={gpu:.0f}% {t_gpu:.1f}°C | '
            f'NPU={npu:.0f}% {t_npu:.1f}°C'
        )
        s.values = [
            self._kv('GPU carga',  gpu,   unit='%'),
            self._kv('GPU temp',   t_gpu, unit='°C'),
            self._kv('NPU carga',  npu,   unit='%'),
            self._kv('NPU temp',   t_npu, unit='°C'),
        ]
        return s

    # ── subsistema Red ────────────────────────────────────────────────────────

    def _status_network(self) -> DiagnosticStatus:
        s = DiagnosticStatus()
        s.name = 'Red'
        s.hardware_id = 'end1'

        tx, rx = self._net_delta()

        s.level = DiagnosticStatus.OK
        s.message = f'TX={tx:.2f} MB/s  RX={rx:.2f} MB/s'
        s.values = [
            self._kv('TX', tx, fmt='.3f', unit=' MB/s'),
            self._kv('RX', rx, fmt='.3f', unit=' MB/s'),
        ]
        return s

    # ── subsistema Tópicos ROS ────────────────────────────────────────────────

    def _status_topics(self) -> DiagnosticStatus:
        s = DiagnosticStatus()
        s.name = 'Topicos_ROS'
        s.hardware_id = 'DDS/domain42'

        hz_scan  = self._hz('/scan')
        hz_color = self._hz('/camera/color/image_raw')
        hz_depth = self._hz('/camera/depth/image_raw')
        hz_odom  = self._hz('/odom_raw')

        s.level = max(
            self._level(hz_scan,  *self._THRESHOLDS['hz_scan'],  low=True),
            self._level(hz_color, *self._THRESHOLDS['hz_image'], low=True),
            self._level(hz_depth, *self._THRESHOLDS['hz_image'], low=True),
            self._level(hz_odom,  *self._THRESHOLDS['hz_odom'],  low=True),
        )
        s.message = (
            f'scan={hz_scan:.1f}Hz  color={hz_color:.1f}Hz  '
            f'depth={hz_depth:.1f}Hz  odom={hz_odom:.1f}Hz'
        )
        s.values = [
            self._kv('scan Hz',  hz_scan),
            self._kv('color Hz', hz_color),
            self._kv('depth Hz', hz_depth),
            self._kv('odom Hz',  hz_odom),
        ]
        return s


# ── entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = SystemMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
