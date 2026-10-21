# calibrate_joints.py

## Descripción General

El script `calibrate_joints.py` realiza la calibración del punto cero angular de los encoders de cada una de las articulaciones del robot tras cada ciclo de encendido (power cycle). Al mover manualmente las articulaciones contra los topes mecánicos o posturas de referencia conocidas, calcula y almacena los desfases angulares en un archivo `calibration.yaml`.

---

## Procedimiento de Calibración

1. **Definición de Orientaciones de Ejes (`joint_axis_directions`):**
   - Mapea los signos relativos de cada articulación para asegurar que los ángulos medidos crezcan en la dirección estándar del modelo cinemático.
2. **Valores Angulares Ideales (`ideal_values`):**
   - Contiene la configuración angular exacta en radianes correspondiente a las posiciones de tope físico o calas mecánicas del robot para cadera, rodilla y tobillo de ambas piernas.
3. **Registro de Extremos y Límites:**
   - Lee de forma continua los valores de los encoders articulares mientras el operador mueve lentamente las extremidades hacia sus límites.
   - Registra los mínimos y máximos alcanzados (`limit_readings`).
4. **Guardado de Calibración:**
   - Una vez que el operador activa el botón de confirmación en el gamepad (`mode_switch == 1`), calcula:
     $$\text{offset}_i = \text{lectura\_límite}_i - \text{valor\_ideal}_i$$
   - Escribe el vector de offsets en el archivo `calibration.yaml`, el cual será consumido automáticamente por el controlador C++ (`real_humanoid.cpp`) y por las rutinas de inferencia.
