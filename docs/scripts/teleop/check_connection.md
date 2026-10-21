# check_connection.py (Teleop)

## Descripción General

El script `check_connection.py` es una herramienta diagnóstica diseñada para verificar la comunicación y la integridad de los buses CAN asociados a los actuadores de los brazos (sistema `Bimanual`).

---

## Flujo de Trabajo

1. **Instanciación:**
   - Instancia `Bimanual()`, abriendo los canales CAN asignados a los miembros superiores.
2. **Ciclo de Verificación (`check_connection()`):**
   - Itera de manera continua llamando a `robot.check_connection()`.
   - Consulta el estado de cada motor (IDs, latencias de respuesta, estados de error).
   - Informa en tiempo real si alguno de los actuadores pierde la sincronización de paquetes o no responde a los heartbeats del bus.
3. **Manejo de Cierre:**
   - Detiene el polling de forma segura ante una señal de interrupción por teclado (`Ctrl+C`), ejecutando `robot.stop()`.

---

## Ejemplo de Uso

```bash
python src/scripts/teleop/check_connection.py
```
