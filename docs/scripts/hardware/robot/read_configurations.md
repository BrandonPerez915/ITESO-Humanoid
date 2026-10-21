# read_configurations.py (Robot)

## Descripción General

El script `read_configurations.py` en `control/scripts/robot/` realiza una lectura exhaustiva de los parámetros de todos los actuadores del robot a través de los canales CAN y genera un archivo de respaldo `robot_configuration.json`.

---

## Flujo de Trabajo

1. Conecta con las interfaces SocketCAN asociadas al robot.
2. Itera sobre la lista completa de articulaciones de las piernas (`robot.joints`).
3. Para cada actuador, interroga vía comandos SDO:
   - Relación de reducción.
   - Ganancias PID de posición y velocidad.
   - Límites de torque y límites de carrera angular.
   - Frecuencia de tramas rápidas.
4. Exporta todos los valores leídos a un diccionario estructurado en formato JSON, sirviendo como respaldo o plantilla de configuración para calibraciones posteriores.
