# configure_parameter.py

## Descripción General

El script `configure_parameter.py` proporciona funciones especializadas para leer y programar parámetros de bajo nivel en la memoria no volátil (Flash/EEPROM) de los controladores Recoil mediante comandos SDO.

---

## Funciones de Configuración Disponibles

1. **`configure_fast_frame_rate(bus, device_id, fast_frame_rate)`:**
   - Modifica la frecuencia de emisión de los paquetes cíclicos de telemetría PDO (en Hz).
2. **`configure_gear_ratio(bus, device_id, gear_ratio)`:**
   - Programa la relación de reducción mecánica interna o externa para que las lecturas y comandos se expresen directamente en el eje de salida de la articulación.
3. **`configure_position_pd(bus, device_id, kp, kd)`:**
   - Ajusta las constantes de ganancia proporcional y derivativa del lazo de posición a nivel de firmware.
4. **`configure_torque_limit(bus, device_id, torque_limit)`:**
   - Establece el límite absoluto de torque admisible por seguridad.
5. **`configure_phase_order(bus, device_id, phase_order)`:**
   - Invierte el orden de las fases eléctricas en caso de giros en sentido inverso.
6. **`configure_position_limit(bus, device_id, lower_limit, upper_limit)`:**
   - Configura límites de software (software end-stops) para evitar que la articulación golpee contra partes del chasis.
7. **`configure_current_bandwidth(bus, device_id, bandwidth, kp, ki)`:**
   - Ajusta el ancho de banda del lazo PI de corriente FOC (Field Oriented Control).
