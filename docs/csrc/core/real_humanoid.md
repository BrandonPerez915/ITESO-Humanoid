# real_humanoid.cpp

## Descripción General

El archivo `real_humanoid.cpp` (y su cabecera asociada `real_humanoid.h`) implementa la clase central `RealHumanoid`, encargada del lazo de control determinista a 1000 Hz en C++ sobre hardware real. Administra la sincronización de hilos de tiempo real (`pthread`), el paso de mensajes por buses CAN, la adquisición de datos inerciales de la IMU, la comunicación UDP con el proceso de inferencia de la red neuronal y la máquina de estados de seguridad.

---

## Componentes y Arquitectura

### 1. Inicialización y Carga de Calibración
- Carga el archivo de calibración `calibration.yaml` mediante `YAML::LoadFile`:
  - Extrae `position_offsets`: Desplazamientos angulares de cero absoluto para cada articulación.
- Carga los parámetros de política desde el archivo YAML especificado en `POLICY_CONFIG_PATH`:
  - `joint_kp`: Ganancias proporcionales por articulación.
  - `joint_kd`: Ganancias derivativas por articulación.
  - `torque_limit`: Límites de esfuerzo estricto para cada actuador.
  - `config_control_dt_`: Periodo del lazo de control (típicamente 1 ms = 1000 Hz).
  - `config_policy_dt_`: Periodo de inferencia de la política de RL (ej. 20 ms = 50 Hz).

---

### 2. Máquina de Estados del Robot
- **`STATE_IDLE`:** Los motores sostienen suavemente su posición actual o permanecen en amortiguamiento. Al pulsar el botón de activación (modo de inicio), conmuta a `STATE_RL_INIT`.
- **`STATE_RL_INIT`:** Rutina de transición suave donde las articulaciones se interpolan mediante `linear_interpolate` hacia las posturas articulares iniciales de balance (`rl_init_positions`).
- **`STATE_RL_RUNNING`:** Se aplican en tiempo real los targets generados por la red neuronal de locomoción.

---

### 3. Ciclo de Control y Sincronización
- **`control_loop()`:**
  - Envía periódicamente señales de mantenimiento (`feed()`) a cada controlador de motor para refrescar sus watchdogs de hardware.
  - Empaqueta y despacha los comandos PDO a través de `SocketCAN`.
  - Recibe las respuestas de telemetría de todos los motores (posición, velocidad, torque estimado).
  - Recibe la orientación y velocidades angulares de la IMU.
- **Comunicación UDP Inter-Procesos:**
  - Envía el vector serializado de observaciones al controlador de política de RL (Python o TorchScript).
  - Recibe el vector de acciones objetivo resultantes.
- **`stop()`:**
  - Envía tramas NMT y comandos de modo deshabilitado a todos los actuadores, asegurando una parada ordenada.
