# motor_controller.cpp

## Descripción General

El archivo `motor_controller.cpp` implementa la clase `MotorController` en C++, responsable de la comunicación directa a nivel de tramas CAN con los controladores de motor Recoil. Codifica los comandos de control PD hacia los motores y decodifica los paquetes de respuesta con las mediciones articulares.

---

## Funcionalidades Principales

- **Gestión de Modos:**
  - `set_mode(uint8_t mode)`: Construye y envía una trama de cambio de modo (posición, velocidad, torque, damping o deshabilitado).
- **Alimentación de Watchdog (`feed`):**
  - Envía la trama periódica requerida por el firmware del motor para certificar que el host de control sigue vivo.
- **Empaquetado de Comandos (`command_position_velocity_torque`):**
  - Comprime posición de consigna, velocidad de consigna, torque feedforward, $K_p$ y $K_d$ en formato de punto fijo de 16 bits para transmitirlos eficientemente en una trama PDO.
- **Recepción y Decodificación de Telemetría:**
  - Al recibir una respuesta de telemetría por SocketCAN, extrae:
    - Posición angular actual (radianes).
    - Velocidad angular actual (rad/s).
    - Corriente de fase / Torque estimado ($N\cdot m$).
    - Temperatura del puente de potencia y del bobinado.
    - Banderas de error de hardware.
