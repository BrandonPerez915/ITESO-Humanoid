# test_joystick.py

## Descripción General

El script `test_joystick.py` es una utilidad interactiva para verificar la conexión, calibración de ejes y mapeo de botones de los mandos físicos (gamepads) conectados a la computadora a través de USB o Bluetooth.

---

## Funcionamiento

1. Instancia el controlador `Se2Gamepad()`.
2. Inicia el sondeo de eventos de Pygame.
3. Imprime continuamente en una sola línea de consola los valores leídos en tiempo real:
   - Velocidad lineal $V_x$ (eje longitudinal).
   - Velocidad lineal $V_y$ (eje lateral).
   - Velocidad angular $V_{\text{yaw}}$ (eje de giro).
   - Estado de los botones de cambio de modo (IDLE, RL_INIT, RL_RUNNING, ESTOP).
4. Facilita la comprobación de la zona muerta (deadband) y la respuesta táctil de los controles antes de operar el robot físicamente.
