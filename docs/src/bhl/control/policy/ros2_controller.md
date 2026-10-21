# ros2_controller.py

## Descripción General

El módulo `ros2_controller.py` expone una interfaz de comunicación con el middleware **ROS 2**, permitiendo que sistemas de navegación autónoma, algoritmos de SLAM o interfaces remotas envíen comandos de velocidad al robot mediante el tópico estándar `/cmd_vel`.

---

## Componentes y Funcionamiento

### Clase `Se2Ros2`
- **Inicialización de ROS 2:**
  - Inicializa `rclpy` en un hilo independiente para no bloquear el bucle de control en tiempo real.
  - Crea un nodo ROS 2 denominado `bhl_ros2_controller`.
- **Suscripción al Tópico `/cmd_vel`:**
  - Se suscribe a mensajes del tipo `geometry_msgs/msg/Twist` con una cola QoS de 10 mensajes.
- **Diccionario de Comandos (`self.commands`):**
  - Mantiene actualizadas las variables en memoria:
    - `"velocity_x"`: Velocidad longitudinal ($V_x$, en m/s).
    - `"velocity_y"`: Velocidad lateral ($V_y$, en m/s).
    - `"velocity_yaw"`: Velocidad angular de guiñada ($\omega_z$, en rad/s).
    - `"mode_switch"`: Bandera de cambio de estado o parada de emergencia.
- **Temporizador de Inactividad (Watchdog):**
  - Si no se reciben mensajes en `/cmd_vel` tras un tiempo límite (timeout), restablece automáticamente las velocidades a cero para prevenir accidentes si se pierde la conexión de red.
- **Método `stop()`:**
  - Destruye el nodo y realiza el shutdown limpio de `rclpy`.
