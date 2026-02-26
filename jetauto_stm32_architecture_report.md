# Reporte de Arquitectura: Controlador Hiwonder JetAuto (ROS2 & STM32)

Este reporte detalla los hallazgos en la carpeta `D:\robotica\Source Code` para facilitar la migración del ecosistema de Jetson Nano a una Orange Pi 5.

## 1. Estructura del Proyecto Encontrada

La carpeta contiene dos grandes bloques de código:
*   **Firmware STM32**: Ubicado en `D:\robotica\Source Code\STM32\RosRobotControllerMini`. Es un proyecto estándar en C (basado en STM32Cube / HAL) para el microcontrolador de la placa inferior. El binario precompilado es `RRCMini_20240702.hex`.
*   **Workspace ROS2**: Ubicado en `D:\robotica\Source Code\ROS2\src\home\ubuntu\ros2_ws`. Contiene todos los paquetes para control, cinemática (mecanum), navegación, y SLAM.

## 2. El Protocolo de Comunicación (El Secreto del COM7)

A diferencia de la suposición inicial de que usaba `rosserial` o comandos simples `0x55 0x55`, el análisis profundo del archivo `ros_robot_controller_sdk.py` del driver revela un **protocolo serial propietario y empaquetado** diseñado por Hiwonder.

### Configuración del Puerto Serial
*   **Dispositivo**: El sistema ROS2 busca montar el puerto virtualizado típicamente bajo `/dev/rrc` (tu `COM7` en Windows).
*   **Baudrate (Muy importante)**: El SDK se conecta a **1,000,000 bps (1 Mbps)**. *(Esa es la razón por la que recibías "basura" o paquetes cortados cuando probamos a 115200 y 9600 baudios).*

### Estructura del Paquete
Todo mensaje que va desde la PC/SBC hacia el STM32 (escritura) o viceversa (lectura) tiene este formato estricto:

| Byte 0 | Byte 1 | Byte 2 | Byte 3 | Bytes 4 a N | Último Byte |
| :---: | :---: | :---: | :---: | :---: | :---: |
| `0xAA` | `0x55` | `Function ID` | `Length` | `Data Payload` | `CRC8 Checksum` |

1.  **Cabecera**: Siempre es `0xAA 0x55`. (A diferencia del `0x55 0x55` que se emite sólo en el bus directo de los servos).
2.  **Function ID (Comando)**: Define la acción. Los IDs son:
    *   `0x00`: Funciones del sistema y batería.
    *   `0x01`: LED.
    *   `0x02`: Buzzer (Zumbador).
    *   `0x03`: DC Motors (Las 4 ruedas mecanum).
    *   `0x04`: PWM Servos (Tradicionales de 3 pines, del 1 al 4).
    *   `0x05`: Bus Servos (El brazo robótico inteligente / PTZ).
    *   `0x06`: Estado de Botones pulsadores de la placa.
    *   `0x07`: Datos del IMU (MPU6050 - Acelerómetro y Giroscopio).
    *   `0x08`: Control Gamepad inalambrico conectado a la placa.
    *   `0x09`: SBUS (Control remoto analógico tipo drone).
3.  **Length**: Longitud total del Payload en bytes.
4.  **Payload (Data)**: Los parámetros del comando. 
5.  **Checksum**: Una suma de verificación de tipo CRC8 aplicada sobre el *Function ID*, *Length* y *Data*.

## 3. Ejemplo Práctico de Migración a Orange Pi 5

Dado que ya tenemos el SDK original en Python (`ros_robot_controller_sdk.py`), la migración a la Orange Pi 5 es extremadamente sencilla:

1.  **No necesitas reescribir la rueda**: Puedes simplemente copiar la carpeta `ros_robot_controller` a la Orange Pi 5.
2.  **El archivo `Board`**: El archivo SDK expone una clase `Board(device="/dev/ttyUSB0", baudrate=1000000)`. Instanciar este objeto en Python levanta hilos automáticos que parsean el tráfico del COM y exponen métodos limpios.

### Ejemplo de código simple en Python para la Orange Pi 5 (sin necesidad de ROS):

```python
import time
# Asumiendo que has copiado ros_robot_controller_sdk.py en el mismo directorio
from ros_robot_controller_sdk import Board 

# Inicializar conexión (cambiar COM7 por el puerto de la Orange Pi ej. /dev/ttyUSB0)
robot = Board(device="COM7", baudrate=1000000)
robot.enable_reception()

# 0. DESPERTAR EL SISTEMA IMU (CRÍTICO: SOFT-RESET DTR/RTS)
# El STM32 tiene un bug I2C al arrancar. Requiere un reinicio en caliente
# justo al abrir el puerto serial para estabilizar el sensor MPU6050.
# En Linux con PySerial esto a veces es automático. En Windows o scripts custom:
robot.port.dtr = True
robot.port.rts = True
time.sleep(0.1)
robot.port.dtr = False
robot.port.rts = False
time.sleep(0.5)

# Luego, enviar un comando de rotación nula al chasis para activar el loop interno.
robot.set_motor_speed([[1, 0], [2, 0], [3, 0], [4, 0]])

# 1. Hacer sonar el buzzer (Frecuencia 3000Hz, On 0.05s, Off 0.01s, 1 vez)
robot.set_buzzer(3000, 0.05, 0.01, 1)
time.sleep(1)

# 2. Leer Servos Seriales Inteligentes (El hardware JetAuto trae PWM desactivado)
# Los ID suelen empezar en 1 (Base del Brazo) o 254 para broadcast.
robot.bus_servo_read_position(1) 

# 3. Mover los motores del chasis (ID del 1 al 4, velocidad de -1.0 a 1.0)
# robot.set_motor_speed([[MotorID, Velocidad], ...])
robot.set_motor_speed([[1, 0.5], [2, 0.5], [3, 0.5], [4, 0.5]]) # Hacia adelante
time.sleep(2)
robot.set_motor_speed([[1, 0], [2, 0], [3, 0], [4, 0]]) # Parar

# 4. Leer IMU en un hilo continuo (requiere paso 0 previo)
while True:
    imu_data = robot.get_imu()
    if imu_data:
        # ax, ay, az, gx, gy, gz
        print(f"X: {imu_data[0]:.2f}, Y: {imu_data[1]:.2f}, Z: {imu_data[2]:.2f}")
    time.sleep(0.1)
```

## Conclusión
La placa controladora se encarga del PID de bajo nivel de los motores, leer y decodificar el IMU, y gobernar los servos en una capa aislada y programada en C desde fábrica. 

Para lograr que el JetAuto funcione en tu Orange Pi 5, solo tienes que replicar este flujo. Tienes dos caminos:
1.  **Camino Directo de Python (Standalone)**: Copias el archivo `ros_robot_controller_sdk.py` y programas todas las lógicas (cámaras, Lidar, evasión) interactuando directamente con los métodos `Board.set_motor_speed` o `Board.bus_servo_set_position`.
2.  **Camino de ROS2**: Instalas ROS2 en la Orange Pi (por ejemplo, Ubuntu 22.04 + Humble), copias todo el directorio `ros2_ws` completo que extrajimos, compilas con `colcon build`, y levantas el driver `ros2 launch controller controller.launch.py`. Ese paquete carga este SDK internamente y expone los tópicos `/cmd_vel` estándar automáticamente.
