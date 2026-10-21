# move_actuator.py

## Descripción General

El script `move_actuator.py` es una utilidad para probar el movimiento armónico de un actuador individual aislado. Aplica una trayectoria sinusoidal de posición mientras reporta continuamente la posición y velocidad angular leídas en tiempo real a 200 Hz.

---

## Parámetros de Control y Ejecución

- **Configuración Inicial:**
  - Escribe ganancias moderadas de posición en el motor: $K_p = 0.2$, $K_d = 0.005$.
  - Establece un límite de torque seguro: $\tau_{\text{limit}} = 0.2$ N·m.
  - Conmuta el motor a `recoil.Mode.POSITION` y envía la señal `feed()`.
- **Generación de Trayectoria:**
  - Genera una señal sinusoidal continua:
    $$\theta_{\text{target}}(t) = A \cdot \sin(2\pi f t)$$
    con frecuencia $f = 1.0$ Hz y amplitud $A = 1.0$ rad.
- **Comunicación en Tiempo Real (`RateLimiter(200.0)`):**
  - Utiliza la función de intercambio rápido `bus.write_read_pdo_2(device_id, target_angle, 0.0)` para transmitir la posición y recibir el estado en una sola transacción.
- **Parada Limpia:**
  - Al interrumpir con `Ctrl+C`, devuelve el motor a `recoil.Mode.IDLE` y cierra el bus.

---

## Ejemplo de Ejecución

```bash
python src/control/scripts/motor/move_actuator.py --channel can0 --id 1
```
