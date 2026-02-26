# Documentación del Protocolo Hiwonder JetAuto: Function IDs

El controlador principal del Hiwonder JetAuto (STM32) se comunica a través de UART a **1,000,000 bps (1 Mbps)**.

La estructura general de todo paquete enviado o recibido es:
`0xAA 0x55 [Function ID] [Length] [Data Payload] [CRC8 Checksum]`

A continuación, se detallan todos los **Function IDs** disponibles y su propósito, basados en el código fuente oficial del SDK (archivo `ros_robot_controller_sdk.py`).

---

## `0x00`: PACKET_FUNC_SYS (Sistema y Batería)
Utilizado para obtener el estado del sistema, principalmente la lectura del voltaje de la batería.
* **Flujo**: STM32 -> PC
* **Data Payload**: `[0x04, Bateria_L, Bateria_H]`
  * `0x04`: Sub-comando de lectura de batería.
  * El voltaje en milivoltios (mV) se envía como un entero de 16-bits (Little-Endian).

## `0x01`: PACKET_FUNC_LED (Control LED)
Controla el encendido y parpadeo de los LEDs de la placa.
* **Flujo**: PC -> STM32
* **Data Payload (7 bytes)**: `[LED_ID, OnTime_L, OnTime_H, OffTime_L, OffTime_H, Repeat_L, Repeat_H]`
  * `LED_ID`: Identificador del LED (uint8). Usualmente `1`.
  * `OnTime`: Tiempo de encendido en milisegundos (uint16).
  * `OffTime`: Tiempo de apagado en milisegundos (uint16).
  * `Repeat`: Cantidad de veces que se repite el ciclo (uint16).

## `0x02`: PACKET_FUNC_BUZZER (Zumbador)
Controla el tono y duración del buzzer integrado.
* **Flujo**: PC -> STM32
* **Data Payload (8 bytes)**: `[Freq_L, Freq_H, OnTime_L, OnTime_H, OffTime_L, OffTime_H, Repeat_L, Repeat_H]`
  * `Freq`: Frecuencia del sonido en Hertz (uint16).
  * `OnTime`: Duración del sonido en milisegundos (uint16).
  * `OffTime`: Duración de silencio en milisegundos (uint16).
  * `Repeat`: Cantidad de repeticiones (uint16).

## `0x03`: PACKET_FUNC_MOTOR (Motores DC / Ruedas)
Controla la velocidad y sentido de giro de los motores (las ruedas mecanum del chasis).
* **Flujo**: PC -> STM32
* **Data Payload**: `[0x01, Cantidad De Motores, (Motor_ID, Velocidad)...]`
  * `0x01`: Comando de establecimiento de velocidad.
  * `Cantidad`: Cuántos motores se enviarán en este paquete.
  * Por cada motor:
    * `Motor_ID`: (uint8) Índice del motor enviado como `(ID Real - 1)`.
    * `Velocidad`: (float32 de 4 bytes) Rango normalizado, ej. `-1.0` a `1.0`.

## `0x04`: PACKET_FUNC_PWM_SERVO (Servos PWM Estándar)
*⚠️ **IMPORTANTE:** En el firmware estándar del JetAuto analizado, la rutina para registrar y recibir comandos PWM **fue removida/comentada** del `packet_handle.c`. Por consecuente, el STM32 ignorará cualquier envío que empiece con este Function ID. Deben utilizarse Servos Seriales de Bus en su lugar.*

## `0x05`: PACKET_FUNC_BUS_SERVO (Servos de Bus Serial / Brazo Robótico)
Controla la cadena daisy-chain de servos inteligentes (usualmente el brazo 6DOF, cámara PTZ, etc).
* **Flujo**: Bidireccional
* **Set Position Payload**: `[0x01, Duration_L, Duration_H, Cantidad de Servos, (Servo_ID, Posicion_L, Posicion_H)...]` (Misma estructura que los PWM servos).
* **Mapeo de Hardware JetAuto (Cámara Astra)**:
  * **Servo ID**: `1` (Control de Paneo / Izquierda-Derecha).
  * **Rango útil**: `200` (Extremo Derecha) a `800` (Extremo Izquierda).
  * **Centro absoluto**: `500`.
* **Otros sub-comandos Notables**:
  * `0x03`: Stop (Detener giro).
  * `0x0B` / `0x0C`: Activar/Desactivar Torque.
  * `0x10`: Set ID.
  * `0x20` / `0x24`: Configurar/Guardar Offset.
  * `0x30` / `0x34` / `0x38`: Configurar límites de Ángulo, Voltaje y Temperatura.
  * `0x12`, `0x22`, `0x05`, `0x07`, `0x09`...: Leer IDs, Offset, Posición, Voltaje y Temperatura del servomotor en el bus. *(El STM32 retorna estos resultados codificando un indicador de "success/fail" en el payload que recibe la PC).*

## `0x06`: PACKET_FUNC_KEY (Botones Físicos)
Envía notificaciones a la PC/SBC cuando se presionan los pulsadores (KEY1, KEY2) de la placa controladora principal.
* **Flujo**: STM32 -> PC
* **Data Payload**: `[Key_ID, Evento]`
  * `Key_ID`: `1` (KEY1) o `2` (KEY2).
  * `Evento` (Bitmask):
    * `0x01` (1): Seleccionado (Pressed)
    * `0x02` (2): Presión larga
    * `0x20` (32): Clic simple
    * `0x40` (64): Doble clic

## `0x07`: PACKET_FUNC_IMU (Sensor Inercial MPU6050)
Devuelve de manera continua o bajo demanda los datos crudos del IMU y el giroscopio integrados en la base (esenciales para SLAM en ROS).
* **Flujo**: STM32 -> PC
* **Data Payload (24 bytes)**: `[ax, ay, az, gx, gy, gz]`
  * Los 6 valores corresponden a mediciones en ejes X,Y,Z para la Aceleración y Giroscopio.
  * Cada valor se envía como un `float32` estándar (4 bytes cada uno en formato Little Endian). Total 24 bytes de datos.
* ⚠️ **Requisito de Despertado (Wake-Up)**: El firmware del STM32 tiene un fallo crítico al encenderse en el cual el bus I2C interno se bloquea impidiendo que el IMU MPU6050 inicie correctamente si la alimentación eléctrica no es perfectamente estable desde el milisegundo cero o si no detecta la pantalla OLED oficial. **Para forzar al STM32 a que empiece a reportar datos del IMU**, el dispositivo anfitrión (PC, Jetson, u Orange Pi) **DEBE realizar un Reinicio por Hardware (Soft-Reset) a la placa justo en el instante de abrir la conexión serial**, levantando los pines `DTR` y `RTS` a nivel alto (True) por 100ms, y luego volviéndolos a nivel bajo (False), permitiendo otros 500ms para que la STM32 termine el nuevo ciclo de arranque con la energía ya estabilizada. El driver original de ROS en Linux realiza este "coletazo" de DTR de manera automática e invisible al abrir el puerto `/dev/ttyUSB0` mediante la librería `pyserial`. En código custom (o en Windows), este proceso de purga debe codificarse manualmente.

## `0x08`: PACKET_FUNC_GAMEPAD (Mando de Juegos USB / Inalámbrico)
Envía datos del Gamepad si se encuentra enchufado un receptor (dongle) de gamepad directamente al USB de la controladora STM32 u operando.
* **Flujo**: STM32 -> PC
* **Data Payload (7 bytes)**: `[Botones_L, Botones_H, Hat, LX, LY, RX, RY]`
* En **ROS 2**, el driver `ros_robot_controller` desempaqueta estos bytes y los publica estructurados en el tópico `sensor_msgs/Joy` (`/ros_robot_controller/joy`).

### Mapeo Oficial de Hardware a `sensor_msgs/Joy` (Estilo Xbox/PS):

**Array de Botones (`msg.buttons`)**:
| Botón Físico | Índice ROS (`buttons[]`) |
| :--- | :--- |
| **A** (Cruz) | `0` |
| **B** (Círculo) | `1` |
| **X** (Cuadrado) | `3` |
| **Y** (Triángulo) | `4` |
| **L1** (Bumper Izq) | `6` |
| **R1** (Bumper Der) | `7` |
| **Select** | `10` |
| **Start** | `11` |
| **L3** (Joystick Izq Click)| `13` |
| **R3** (Joystick Der Click)| `14` |

**Array de Ejes (`msg.axes`)**:
| Control Físico | Índice ROS (`axes[]`) | Dirección Estándar |
| :--- | :--- | :--- |
| **Joystick Izquierdo - X** | `0` | `+1.0` (Izquierda) / `-1.0` (Derecha) |
| **Joystick Izquierdo - Y** | `1` | `+1.0` (Arriba) / `-1.0` (Abajo) |
| **Joystick Derecho - X** | `2` | `+1.0` (Izquierda) / `-1.0` (Derecha) |
| **Joystick Derecho - Y** | `3` | `+1.0` (Arriba) / `-1.0` (Abajo) |
| **R2** (Gatillo Analógico Der) | `4` | `0.0` (Libre) -> `1.0` (Presionado a fondo) |
| **L2** (Gatillo Analógico Izq) | `5` | `0.0` (Libre) -> `1.0` (Presionado a fondo) |
| **D-Pad (Cruceta) - X** | `6` | `+1.0` (Izquierda) / `-1.0` (Derecha) |
| **D-Pad (Cruceta) - Y** | `7` | `+1.0` (Arriba) / `-1.0` (Abajo) |

## `0x09`: PACKET_FUNC_SBUS (Radio control de aeromodelismo)
Devuelve valores provenientes de un receptor de Radio Control (estilo FrSky / FlySky) a través del bus S.BUS por si se quiere manejar el robot RC desde una emisora de drones.
* **Flujo**: STM32 -> PC
* **Data Payload (36 bytes)**: `[16x Canales, CH17, CH18, Sig_Loss, Fail_Safe]`
  * Los primeros 16 canales enviados como enteros con signo de 16-bits (Little-Endian).
  * Banderas booleanas (1 byte, 0 o 1) para si hay canal 17 o 18.
  * Banderas (1 byte) alertando si hay Pérdida de Señal (Signal Loss) o si está en modo a prueba de fallos (Fail Safe).

## `0x0A`: PACKET_FUNC_NONE
Cualquier función no mapeada por encima de 9.

---
**Nota sobre Checksums:**
La función que genera y valida el CRC8 en Python utiliza una tabla Latch personalizada precalculada (*crc8_table*) y realiza operaciones XOR byte por byte hasta generar un `uint8` (`& 0x00FF`).
