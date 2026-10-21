# events.py

## Descripción General

El módulo `events.py` proporciona funciones para la **aleatorización de dominio (Domain Randomization)** durante el entrenamiento en Isaac Lab. Al perturbar parámetros físicos y errores de calibración en la simulación, se asegura que las políticas aprendidas sean robustas frente a las imperfecciones del robot real (cierre de la brecha Sim-to-Real).

---

## Funciones de Aleatorización

### 1. `randomize_joint_default_pos(...)`
- **Objetivo:** Simular errores de calibración y holguras mecánicas en el punto cero de los encoders articulares.
- **Funcionamiento:**
  - Extrae las posiciones articulares por defecto (`default_joint_pos`) del robot.
  - Aplica una perturbación muestreada a partir de una distribución (`uniform`, `log_uniform` o `gaussian`) según los límites dados en `pos_distribution_params`.
  - Puede sumar (`"add"`), escalar (`"scale"`) o sobrescribir (`"abs"`) el ángulo articular.
  - Asigna las nuevas posiciones de referencia al tensor del robot en GPU.

---

### 2. `randomize_actuator_torque_constant(...)`
- **Objetivo:** Simular variaciones en las constantes de torque de los motores, caídas de voltaje de la batería, fricción en reductores y dispersión en las ganancias PD del hardware real.
- **Funcionamiento:**
  - Itera sobre los actuadores del robot (`asset.actuators.values()`).
  - Escala la rigidez ($K_p$) y el amortiguamiento ($K_d$) aplicando factores de aleatorización por entorno.
  - Escribe directamente los nuevos valores en la memoria física de simulación mediante:
    - `asset.write_joint_stiffness_to_sim(...)`
    - `asset.write_joint_damping_to_sim(...)`
