# run_locomotion.py (Control)

## Descripción General

El script `run_locomotion.py` es el lanzador principal para ejecutar políticas de locomoción basadas en aprendizaje por refuerzo sobre el robot físico Berkeley Humanoid Lite. Integra la abstracción del robot real (`Humanoid`), la inferencia neuronal con `RlController` y la transmisión continua de telemetría a través de sockets UDP hacia estaciones de monitoreo y visualización.

---

## Flujo de Ejecución

1. **Carga de Configuración:**
   - Lee el archivo de configuración YAML especificado con `--config` mediante `Cfg.from_arguments()`.
   - Extrae la frecuencia de la política (por ejemplo 50 Hz).
2. **Inicialización de Red UDP:**
   - Crea un socket `UDP(("0.0.0.0", 11000), ("172.28.0.5", 11000))` para transmitir telemetría en vivo a la computadora host remota.
3. **Instanciación del Controlador de Red:**
   - Carga la política neural (ONNX o TorchScript) mediante `controller.load_policy()`.
4. **Instanciación y Secuencia de Seguridad del Robot:**
   - Inicializa el objeto `Humanoid()`, abriendo las conexiones con los actuadores en los buses CAN (`can0`, `can1`) y conectando con la IMU y el mando gamepad.
   - Pone todos los motores en modo amortiguamiento (`robot.enter_damping()`) para evitar movimientos intempestivos.
   - Llama a `robot.reset()` para calibrar la postura inicial y obtener las primeras observaciones.
5. **Bucle de Control Determinista:**
   - Regulado con `RateLimiter(1 / cfg.policy_dt)`:
     1. `actions = controller.update(obs)`: Calcula las acciones articulares con la red neuronal.
     2. `obs = robot.step(actions)`: Envía los ángulos objetivo a los motores por CAN y lee el nuevo estado cinemático e inercial.
     3. `udp.send_numpy(obs)`: Emite el paquete de telemetría en tiempo real.
     4. `rate.sleep()`: Duerme el tiempo exacto remanente para garantizar una frecuencia de muestreo constante.
6. **Manejo de Cierre:**
   - Al pulsar `Ctrl+C`, detiene y apaga los motores con `robot.stop()`.
