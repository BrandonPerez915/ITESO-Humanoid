# run_idle.py (Teleop)

## Descripción General

El script `run_idle.py` inicializa los dos brazos del robot Berkeley Humanoid Lite en un estado pasivo o de reposo seguro mediante la clase `Bimanual`. Configura ganancias PD bajas y un límite de torque conservador, permitiendo sostener los brazos en posición neutra sin forzar los actuadores ni provocar movimientos bruscos.

---

## Parámetros de Control y Ejecución

1. **Instanciación:**
   - Crea una instancia de `Bimanual()`, la cual conecta con las interfaces SocketCAN correspondientes a los brazos izquierdo y derecho.
2. **Configuración de Parámetros PD (`robot.run`):**
   - **$K_p = 20.0$:** Rigidez moderada para amortiguar desviaciones de posición.
   - **$K_d = 2.0$:** Amortiguamiento para evitar oscilaciones mecánicas.
   - **`torque_limit = 0.5` N·m:** Límite estricto de torque para garantizar la seguridad del operador en caso de contacto físico.
3. **Bucle a 100 Hz (`RateLimiter(100)`):**
   - Envía un vector objetivo de ceros a las 10 articulaciones de los brazos:
     `robot.step(np.zeros((10,), dtype=np.float32))`.
   - Lee y despliega en pantalla la retroalimentación de telemetría devuelta por los actuadores.
4. **Parada Segura:**
   - Al recibir `KeyboardInterrupt` (Ctrl+C), invoca `robot.stop()`, enviando comandos de deshabilitación a todos los motores antes de salir.

---

## Ejemplo de Ejecución

```bash
python src/scripts/teleop/run_idle.py
```
