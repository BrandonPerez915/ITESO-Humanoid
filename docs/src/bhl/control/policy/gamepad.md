# gamepad.py

## Descripción General

El módulo `gamepad.py` implementa los controladores de entrada interactivos para el operador humano. Proporciona soporte tanto para mandos de videojuegos físicos (Xbox, PlayStation, Logitech mediante `pygame`) como para control por teclado en entornos de consola o visualizadores de MuJoCo.

---

## Clases Principales

### 1. `Se2Gamepad`
- **Detección de Hardware:** Utiliza `pygame.joystick` para detectar y abrir mandos USB o Bluetooth conectados.
- **Mapeo de Ejes Analógicos ($SE(2)$):**
  - Palanca izquierda vertical: Velocidad longitudinal ($V_x$).
  - Palanca izquierda horizontal: Velocidad lateral ($V_y$).
  - Palanca derecha horizontal: Velocidad de rotación en guiñada ($\omega_z$).
  - Incluye zona muerta (deadband) para evitar derivas causadas por tolerancias mecánicas de los joysticks.
- **Botones de Estado y Seguridad:**
  - Botón **A / Cruz:** Transición a modo reposo/damping.
  - Botón **B / Círculo:** Parada de emergencia (E-Stop).
  - Botón **X / Cuadrado:** Inicialización de postura bípeda (`RL_INIT`).
  - Botón **Y / Triángulo:** Activación de locomoción por aprendizaje por refuerzo (`RL_RUNNING`).
- **Hilo de Sondeo:** Corre en un hilo en segundo plano a 50 Hz actualizando el diccionario `self.commands`.

---

### 2. `Se2Keyboard`
- Alternativa para desarrollo sin mando físico.
- Mapea teclas convencionales:
  - `W` / `S`: Adelante / Atrás.
  - `A` / `D`: Giro izquierda / derecha.
  - `Q` / `E`: Desplazamiento lateral (strafe).
  - `Espacio` / `R`: Modos de estado y reinicio.
