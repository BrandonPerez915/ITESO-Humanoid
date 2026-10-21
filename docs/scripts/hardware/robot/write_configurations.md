# write_configurations.py (Robot)

## Descripción General

El script `write_configurations.py` en `control/scripts/robot/` automatiza la programación masiva de todos los parámetros de firmware de los 12 actuadores del robot Berkeley Humanoid Lite a partir de un archivo maestro `robot_configuration.json`.

---

## Flujo de Ejecución

1. **Carga del Archivo Maestro:**
   - Lee `robot_configuration.json`, el cual define para cada articulación (`left_hip_roll_joint`, `left_knee_pitch_joint`, etc.):
     - Frecuencia de tramas de telemetría (`fast_frame_frequency`).
     - Relación de reducción mecánica (`gear_ratio`).
     - Ganancias de control de posición y velocidad (`position_kp`, `position_ki`, `velocity_kp`, `velocity_ki`).
     - Límites de esfuerzo y corriente (`torque_limit`, límites de posición).
2. **Ping y Verificación Inicial:**
   - Itera por cada articulación conectada verificando presencia (`bus.ping(joint_id)`) y reportando la versión de firmware en hexadecimal.
3. **Escritura y Verificación de Parámetros:**
   - Programa cada parámetro vía SDO con un retardo de protección (`delay_t = 0.1` s) para evitar congestión en el bus CAN.
4. **Almacenamiento Persistente en Flash:**
   - Si `store_to_flash = True`, envía el comando de grabado en memoria no volátil, asegurando que los parámetros se preserven tras apagar el robot.
