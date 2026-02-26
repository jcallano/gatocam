# Configuración de Red ROS 2 — JetAuto ↔ Máquinas Externas

## Contexto del problema

FastDDS (el middleware DDS por defecto de ROS 2 Jazzy) usa **multicast UDP** para el descubrimiento de participantes (Participant Discovery Protocol, PDP). En redes domésticas o con VMs el multicast suele estar bloqueado o no llegar, lo que hace que las máquinas externas no vean ningún topic del robot aunque haya conectividad IP completa.

**Síntoma típico:**
```bash
# En la máquina remota, con ROS 2 instalado y misma red:
ros2 topic list
# Solo muestra: /parameter_events  /rosout
# No ve /scan, /odom_raw, /camera/*, etc.
```

**Causa:** el router/switch filtra paquetes UDP multicast (grupo 239.255.0.1). El ping funciona, pero FastDDS no puede hacer el handshake inicial.

---

## Solución implementada: Unicast Peer Discovery

En lugar de esperar que los paquetes multicast lleguen, se configura FastDDS para enviar probes de descubrimiento directamente a las IPs/puertos conocidos del robot usando **unicast**.

### Fórmula de puertos FastDDS

Para `ROS_DOMAIN_ID=42`, los puertos unicast de cada nodo siguen la fórmula:

```
puerto_participante_N = 7400 + 250 × domainID + 10 + 2×N
```

Con `domainID=42`:
```
base = 7400 + 250×42 + 10 = 7400 + 10500 + 10 = 17910
participante 0 → 17910
participante 1 → 17912
participante 2 → 17914
...
participante N → 17910 + 2×N
```

El bringup completo del JetAuto levanta ~10 nodos → puertos 17910–17930.

---

## Configuración del robot (ya aplicada)

**Archivo:** `~/.ros/fastdds_unicast.xml`

Actualizado automáticamente por `bringup.sh`. Para añadir una nueva máquina, agregar su IP en `initialPeersList` (ver sección "Añadir nueva máquina").

**Variables de entorno (en `~/.bashrc` y en `bringup.sh`):**
```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/.ros/fastdds_unicast.xml
```

---

## Configuración en una nueva máquina — Paso a paso

### Prerequisitos

- ROS 2 Jazzy instalado
- Misma red que el robot (IP en rango `192.168.100.x`)
- La VM en **modo bridge** (no NAT) — ver nota al final

### 0. Aumentar buffers UDP del sistema operativo

Los mensajes grandes de ROS 2 (PointCloud2 ~1.2 MB, Image ~900 KB) se fragmentan en cientos de paquetes UDP. Con el buffer por defecto (~208 KB) los fragmentos se descartan y los displays de RViz2 no muestran nada.

**Síntoma:** `/scan` (2 KB) se ve en RViz2 pero PointCloud2 e imágenes NO.

```bash
# Aplicar inmediatamente
sudo sysctl -w net.core.rmem_max=26214400
sudo sysctl -w net.core.rmem_default=26214400
sudo sysctl -w net.core.wmem_max=26214400
sudo sysctl -w net.core.wmem_default=26214400

# Hacer persistente en /etc/sysctl.conf
sudo tee -a /etc/sysctl.conf << 'EOF'

# ROS2 FastDDS — buffers UDP para mensajes grandes (PointCloud2, Image)
net.core.rmem_max=26214400
net.core.rmem_default=26214400
net.core.wmem_max=26214400
net.core.wmem_default=26214400
EOF
```

> **Hacer lo mismo en el robot** (lado emisor) para que los sockets de envío tengan buffer suficiente.

### 1. Crear el archivo FastDDS

```bash
mkdir -p ~/.ros

cat > ~/.ros/fastdds_unicast.xml << 'EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">

    <transport_descriptors>
        <transport_descriptor>
            <transport_id>UDPv4_large</transport_id>
            <type>UDPv4</type>
            <sendBufferSize>16777216</sendBufferSize>
            <receiveBufferSize>16777216</receiveBufferSize>
            <!-- 1400 = bajo MTU Ethernet (1500) → sin fragmentación IP → sin pérdidas -->
            <maxMessageSize>1400</maxMessageSize>
        </transport_descriptor>
    </transport_descriptors>

    <participant profile_name="default_profile" is_default_profile="true">
        <rtps>
            <userTransports>
                <transport_id>UDPv4_large</transport_id>
            </userTransports>
            <useBuiltinTransports>false</useBuiltinTransports>
            <builtin>
                <initialPeersList>
                    <!-- Nodos del robot JetAuto (192.168.100.197, domain 42) -->
                    <locator><udpv4><address>192.168.100.197</address><port>17910</port></udpv4></locator>
                    <locator><udpv4><address>192.168.100.197</address><port>17912</port></udpv4></locator>
                    <locator><udpv4><address>192.168.100.197</address><port>17914</port></udpv4></locator>
                    <locator><udpv4><address>192.168.100.197</address><port>17916</port></udpv4></locator>
                    <locator><udpv4><address>192.168.100.197</address><port>17918</port></udpv4></locator>
                    <locator><udpv4><address>192.168.100.197</address><port>17920</port></udpv4></locator>
                    <locator><udpv4><address>192.168.100.197</address><port>17922</port></udpv4></locator>
                    <locator><udpv4><address>192.168.100.197</address><port>17924</port></udpv4></locator>
                    <locator><udpv4><address>192.168.100.197</address><port>17926</port></udpv4></locator>
                    <locator><udpv4><address>192.168.100.197</address><port>17928</port></udpv4></locator>
                    <locator><udpv4><address>192.168.100.197</address><port>17930</port></udpv4></locator>
                    <!-- Esta misma máquina (reemplazar con tu IP) -->
                    <locator><udpv4><address>TU_IP_AQUI</address><port>0</port></udpv4></locator>
                </initialPeersList>
            </builtin>
        </rtps>
    </participant>

</profiles>
EOF
```

Sustituir `TU_IP_AQUI` con la IP de la nueva máquina:
```bash
MY_IP=$(ip route get 192.168.100.197 | grep -oP 'src \K\S+')
sed -i "s/TU_IP_AQUI/$MY_IP/" ~/.ros/fastdds_unicast.xml
echo "IP configurada: $MY_IP"
```

### 2. Añadir variables al `.bashrc`

```bash
cat >> ~/.bashrc << 'EOF'

# ROS 2 — JetAuto robot
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/.ros/fastdds_unicast.xml

# Meshes y descripción del robot (resuelve package://jetauto_description/...)
export AMENT_PREFIX_PATH=/mnt/ros2_ws/install/jetauto_description:${AMENT_PREFIX_PATH:-}
EOF

source ~/.bashrc
```

> **Nota:** `AMENT_PREFIX_PATH` apunta al workspace Samba (solo `share/`, no ejecuta binarios AArch64).
> Necesario para que RViz2 encuentre los ficheros STL del RobotModel.

### 3. Verificar conectividad

```bash
# En un terminal nuevo (para que .bashrc esté cargado):
source /opt/ros/jazzy/setup.bash

ros2 daemon stop && ros2 daemon start
sleep 15   # FastDDS necesita ~10-15s para el primer descubrimiento

ros2 node list    # debe mostrar los nodos del robot
ros2 topic list   # debe mostrar /scan, /odom_raw, /camera/*, etc.
```

Si aparecen los topics, la configuración es correcta.

### 4. Registrar la nueva máquina en el robot

Para que el robot también descubra la nueva máquina (necesario para servicios y acciones bidireccionales, o para publicar `/cmd_vel`):

**En el robot**, editar `~/.ros/fastdds_unicast.xml` y añadir la IP de la nueva máquina en `initialPeersList`:

```xml
<!-- Añadir antes de </initialPeersList> -->
<locator><udpv4><address>IP_NUEVA_MAQUINA</address><port>0</port></udpv4></locator>
```

Reiniciar el bringup del robot para que los nuevos nodos usen el perfil actualizado:
```bash
cd /home/jcallano/ros2_ws && ./bringup.sh
```

---

## Montar el workspace por Samba (opcional pero recomendado)

El robot expone el workspace completo via Samba. Montarlo da acceso a configs de RViz, launch files, modelos, etc.

### Instalar cliente Samba
```bash
sudo apt-get install -y cifs-utils
```

### Guardar credenciales
```bash
mkdir -p ~/.samba
cat > ~/.samba/robot_creds << 'EOF'
username=jcallano
password=ano7410
domain=WORKGROUP
EOF
chmod 600 ~/.samba/robot_creds
```

### Montar el share
```bash
sudo mkdir -p /mnt/ros2_ws
sudo mount -t cifs //192.168.100.197/ros2_ws /mnt/ros2_ws \
    -o credentials=$HOME/.samba/robot_creds,uid=$(id -u),gid=$(id -g),file_mode=0664,dir_mode=0775
```

### Montaje permanente (`/etc/fstab`)
```
//192.168.100.197/ros2_ws  /mnt/ros2_ws  cifs  credentials=/home/TUUSUARIO/.samba/robot_creds,uid=TU_UID,gid=TU_GID,file_mode=0664,dir_mode=0775,_netdev,auto  0  0
```

Obtener `uid` y `gid`:
```bash
id -u && id -g
```

---

## Lanzar RViz2 con la config del robot

```bash
source /opt/ros/jazzy/setup.bash
rviz2 -d /mnt/ros2_ws/src/jetauto_description/rviz/sensors.rviz
```

**Fixed Frame:** `base_link` — frame raíz del robot. Se verá:
- RobotModel (chasis + ruedas Mecanum)
- LaserScan (amarillo) — RPLidar A1 en `/scan`
- PointCloud2 — nube de puntos en `/camera/depth/points`
- RGB Image — `/camera/color/image_raw`
- TF axes de todos los frames

---

## Referencia rápida

### Datos de red del robot

| Parámetro | Valor |
|-----------|-------|
| IP (DHCP) | `192.168.100.197` |
| Interfaz | `end1` |
| `ROS_DOMAIN_ID` | `42` |
| Puertos FastDDS | `17910–17930` (uno por nodo) |
| Samba share | `//192.168.100.197/ros2_ws` |
| Samba user | `jcallano` |

### Fórmula de puertos para otros Domain IDs

```python
# Puerto del participante N en domainID D:
puerto = 7400 + 250 * D + 10 + 2 * N

# Ejemplo domain=0:  puerto_0 = 7410, puerto_1 = 7412, ...
# Ejemplo domain=42: puerto_0 = 17910, puerto_1 = 17912, ...
```

### Diagnóstico rápido

```bash
# ¿Llega UDP al robot?
ping 192.168.100.197

# ¿Ve topics?
ros2 topic list

# ¿Qué frames TF hay?
ros2 topic echo /tf --once 2>/dev/null | grep child_frame | sort -u

# ¿A qué Hz publica el LIDAR?
ros2 topic hz /scan

# Forzar redescubrimiento
ros2 daemon stop && ros2 daemon start && sleep 15 && ros2 node list
```

---

## Nota: VM en modo Bridge (obligatorio)

La VM **debe** estar en modo **Bridge** (no NAT) para tener IP propia en la red local y poder comunicarse con el robot.

- **VirtualBox:** Configuración de red → Adaptador → Conectado a: **Adaptador puente**
- **VMware:** VM Settings → Network Adapter → **Bridged** (Autodetect)
- **WSL2 (Windows 11):** Configuración → Red → Modo **Bridge** (o usar `wsl --update` y activar mirror networking)

Con NAT, la VM comparte la IP del host y el multicast/unicast DDS no llega al robot.

---

## Máquinas registradas

| Hostname | IP | Notas |
|----------|----|-------|
| orangepi5 (robot) | 192.168.100.197 | Servidor, bringup principal |
| jcallano-vmdev (VM Linux) | 192.168.100.228 | Cliente RViz2, desarrollo |
