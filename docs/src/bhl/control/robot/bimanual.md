# bimanual.py

## Descripción General

El módulo `bimanual.py` define la abstracción de hardware para el control de los dos brazos del robot Berkeley Humanoid Lite. Proporciona una interfaz unificada para inicializar, calibrar y enviar consignas angulares a los 10 actuadores del tren superior a través de buses CAN independientes.

---

## Mapeo Cinemático de Brazos

- **Brazo Izquierdo (5 DOF):**
  - ID 1: Hombro Pitch (`shoulder_pitch`)
  - ID 3: Hombro Roll (`shoulder_roll`)
  - ID 5: Hombro Yaw (`shoulder_yaw`)
  - ID 7: Codo Pitch (`elbow_pitch`)
  - ID 9: Codo/Muñeca Roll/Yaw (`wrist_yaw`)
- **Brazo Derecho (5 DOF):**
  - ID 2: Hombro Pitch
  - ID 4: Hombro Roll
  - ID 6: Hombro Yaw
  - ID 8: Codo Pitch
  - ID 10: Codo/Muñeca Roll/Yaw

---

## Funcionalidades Principales

- **`run(kp, kd, torque_limit)`:**
  - Configura los modos de operación de los actuadores Recoil (modo posición con control de corriente/torque).
  - Ajusta los coeficientes proporcionales ($K_p$) y derivativos ($K_d$), así como el límite máximo de esfuerzo para evitar sobrecargas o roturas mecánicas.
- **`step(target_positions)`:**
  - Envía las posiciones articulares objetivo a los 10 actuadores de los brazos.
  - Recibe la telemetría simultánea (posiciones reales, corrientes y temperaturas).
  - Retorna un arreglo de NumPy con las observaciones articulares de los brazos.
- **`check_connection()`:**
  - Envía paquetes de diagnóstico (ping/heartbeat) a cada actuador para comprobar la integridad eléctrica y lógica del bus.
- **`stop()`:**
  - Desactiva el lazo cerrado de posición y coloca los motores en modo seguro (idle/damping) antes de cerrar los buses.
