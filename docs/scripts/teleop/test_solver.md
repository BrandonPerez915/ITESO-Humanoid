# test_solver.py

## Descripción General

El script `test_solver.py` es una utilidad de depuración y prueba para el solucionador de cinemática inversa (IK) bimanual. Permite evaluar la convergencia, estabilidad numérica y tiempo de cálculo de la biblioteca **Pink** sin interactuar con los motores físicos o la conexión de hardware.

---

## Funcionamiento Interno

1. **Construcción del Modelo:**
   - Carga el URDF del robot en Pinocchio.
   - Crea un servidor visualizador local con Meshcat para inspeccionar el esqueleto cinemático y los marcos cartesianos.
2. **Definición de Tareas Cinemáticas:**
   - Tarea de seguimiento de posición y orientación para los extremos de los brazos izquierdo y derecho.
   - Tarea de postura (`PostureTask`) opcional para mantener los brazos cerca de una configuración neutra cuando existan redundancias cinemáticas.
3. **Generación de Trayectorias Sintéticas:**
   - Genera trayectorias analíticas en el espacio cartesiano (por ejemplo circunferencias o patrones helicoidales en el espacio 3D).
   - En cada paso del bucle:
     1. Actualiza el target de cada efector final.
     2. Ejecuta `solve_ik(configuration, tasks, dt, solver="quadprog")`.
     3. Integra la velocidad articular calculada para actualizar la configuración del robot.
     4. Publica el estado resultante en Meshcat a una tasa controlada por `RateLimiter`.

---

## Ejemplo de Ejecución

```bash
python src/scripts/teleop/test_solver.py
```
Abre la URL indicada en consola (usualmente `http://127.0.0.1:7000/static/`) para observar la respuesta del algoritmo IK en tiempo real.
