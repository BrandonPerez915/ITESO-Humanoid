# humanoid.py

## Descripción General

El módulo `humanoid.py` proporciona la abstracción en Python de alto nivel para el robot bípedo/humanoide completo Berkeley Humanoid Lite. Gestiona los buses CAN correspondientes a las piernas izquierda y derecha (`can0` y `can1`), adquiere las lecturas de la IMU mediante un hilo serie de alta velocidad, procesa los comandos del gamepad e implementa la máquina de estados finitos que gobierna la seguridad y operación del robot.

---

## Máquina de Estados Finitos (`State`)

1. **`State.INVALID` (0):** Estado inicial o de fallo en caso de pérdida de comunicación o violación de límites físicos.
2. **`State.IDLE` (1):** Estado pasivo de reposo. Los motores se mantienen en modo amortiguamiento o manteniendo la posición actual con torques bajos para permitir la manipulación manual sin forzar la mecánica.
3. **`State.RL_INIT` (2):** Rutina de inicialización y despegue. El robot interpola suavemente desde su postura actual hacia la postura de equilibrio bípedo predefinida (`rl_init_positions`) mediante una función de interpolación lineal (`linear_interpolate`) durante un periodo controlado (por ejemplo 2 a 3 segundos).
4. **`State.RL_RUNNING` (3):** Estado de locomoción activa. En este estado se ejecuta la política de aprendizaje por refuerzo, enviando las acciones calculadas a los actuadores en cada ciclo y aplicando el control PD.

---

## Mapeo de Articulaciones y Canales CAN

Cada pierna se comunica a través de un canal SocketCAN dedicado mediante la clase `recoil.Bus`:
- **Pierna Izquierda (`can0`):**
  - ID 1: `left_hip_roll_joint`
  - ID 3: `left_hip_yaw_joint`
  - ID 5: `left_hip_pitch_joint`
  - ID 7: `left_knee_pitch_joint`
  - ID 11: `left_ankle_pitch_joint`
  - ID 13: `left_ankle_roll_joint`
- **Pierna Derecha (`can1`):**
  - ID 2: `right_hip_roll_joint`
  - ID 4: `right_hip_yaw_joint`
  - ID 6: `right_hip_pitch_joint`
  - ID 8: `right_knee_pitch_joint`
  - ID 12: `right_ankle_pitch_joint`
  - ID 14: `right_ankle_roll_joint`

---

## Componentes y Métodos Clave

- **`self.joint_axis_directions`:** Vector con signos ($\pm 1$) que corrige la dirección de giro positiva según el montaje físico de los motores en el chasis respecto al modelo URDF.
- **`self.position_offsets`:** Offsets angulares de cero absoluto cargados desde la calibración de encoders.
- **`step(actions)`:**
  - Aplica las acciones deseadas a los motores según el estado actual.
  - Adquiere los datos actuales de posición, velocidad y temperatura de cada motor mediante PDO de CAN.
  - Lee el cuaternión y velocidades angulares de la IMU.
  - Empaqueta y retorna el vector de observaciones completo.
