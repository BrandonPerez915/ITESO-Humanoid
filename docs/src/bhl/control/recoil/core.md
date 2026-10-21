# core.py (Recoil Motor Protocol)

## Descripción General

El módulo `core.py` implementa el protocolo de comunicación y control a nivel de registro para los actuadores inteligentes **Recoil**. Define los tipos de mensajes basados en CANopen/SDO/PDO, los códigos de error, los modos de operación y las rutinas de lectura y escritura de parámetros en la memoria no volátil de los motores.

---

## Estructura de Mensajes y Modos

### 1. Funciones CAN (`Function`)
Define la partición de identificadores de 11 bits del estándar CAN:
- `NMT` (Network Management) = `0b0000`
- `TRANSMIT_PDO_1` / `RECEIVE_PDO_1` = `0b0011` / `0b0100` (mensajes rápidos cíclicos de posición, velocidad y torque)
- `TRANSMIT_SDO` / `RECEIVE_SDO` = `0b1011` / `0b1100` (configuración de parámetros bajo demanda)
- `HEARTBEAT` = `0b1110` (monitoreo de presencia)

---

### 2. Modos de Operación (`Mode`)
- **Modos Seguros:**
  - `DISABLED` (`0x00`): Puentes H desactivados, motor en rueda libre.
  - `IDLE` (`0x01`): Modulación detenida.
  - `DAMPING` (`0x02`): Freno dinámico resistivo/regenerativo.
- **Modo de Calibración:**
  - `CALIBRATION` (`0x05`): Rutina de búsqueda de índice de encoder y alineación de fases magnéticas.
- **Modos de Lazo Cerrado (Closed-Loop):**
  - `CURRENT` (`0x10`): Control directo de corriente de cuadratura ($I_q$).
  - `TORQUE` (`0x11`): Control de torque en $N\cdot m$.
  - `VELOCITY` (`0x12`): Control de velocidad angular con rampa.
  - `POSITION` (`0x13`): Control de posición angular con perfil de interpolación.

---

### 3. Diagnóstico de Errores (`ErrorCode`)
Máscara de bits para detección de fallos:
- `ESTOP`: Parada de emergencia activada.
- `CALIBRATION_ERROR`: Fallo en la calibración del sensor magnético.
- `POWERSTAGE_ERROR`: Sobrecorriente o fallo en los MOSFETs del puente.
- `WATCHDOG_TIMEOUT`: Pérdida de paquetes CAN por más del tiempo límite.
- `OVER_VOLTAGE` / `UNDER_VOLTAGE`: Voltaje de alimentación fuera de rango seguro.
- `OVER_TEMPERATURE`: Sobrecalentamiento en el bobinado o en el driver.

---

### 4. Clase `Motor`
Encapsula las operaciones sobre un actuador individual:
- `set_mode(mode: Mode)`: Cambia el modo de funcionamiento del motor.
- `feed()`: Envía comando de refresco del watchdog para evitar paradas por timeout.
- `read_param(index)` / `write_param(index, value)`: Lee y escribe registros mediante SDO.
- `step(target_position, target_velocity, feedforward_torque, kp, kd)`: Envía el comando de control rápido por PDO y recibe la telemetría en un solo ciclo de bus.
