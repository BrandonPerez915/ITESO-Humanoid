# run_teleop.py

## Descripción General

El script `run_teleop.py` implementa el sistema completo de teleoperación bimanual para los brazos del robot Berkeley Humanoid Lite. Resuelve la cinemática inversa (IK) cartesiana en tiempo real mediante programación cuadrática (QP) utilizando las bibliotecas **Pinocchio** y **Pink**, visualiza las metas espaciales en **Meshcat** y transmite comandos de posición a los actuadores físicos a través del controlador `Bimanual` o interfaz de red UDP.

---

## Componentes y Arquitectura

### 1. `TeleopIkSolver`
- **Modelo Cinemático:** Carga el URDF del robot (`berkeley_humanoid_lite.urdf`) en una envoltura de Pinocchio (`RobotWrapper`) con articulación flotante libre (`JointModelFreeFlyer`).
- **Efectores Finales:** Define los eslabones objetivo `arm_left_elbow_roll` y `arm_right_elbow_roll` (extremos de los antebrazos).
- **Formulación del Problema QP con Pink:**
  - **`FrameTask` (Efectores finales):** Ponderación de posición (`position_cost = 1.0`), ponderación de orientación (`orientation_cost = 0.1`) y amortiguamiento Levenberg-Marquardt (`lm_damping = 1.0`).
  - **`FrameTask` (Base):** Costo elevado (`position_cost = 1000.0`, `orientation_cost = 1000.0`) para mantener la base y el torso del robot anclados mientras los brazos se mueven.
  - **Solver QP:** Utiliza `quadprog` para resolver la optimización de velocidad articular con límites de posición.
- **Visualizador Meshcat:** Renderiza en un navegador web los marcos de referencia del robot, de los mandos HTC Vive y de los objetivos interpolados.

### 2. Adquisición de Datos de Teleoperación
- Recibe las poses cartesianas de los mandos HTC Vive o VR (`bridge_data`):
  - Matrices de transformación homogénea $SE(3)$ (4x4) para mano izquierda y derecha.
  - Estado de botones y nivel de gatillo analógico (trigger) para accionar pinzas/grippers.
- Incluye lógica de embrague (clutching) y calibración relativa: los brazos solo siguen a los mandos cuando el operador presiona el botón de habilitación.

### 3. Envío de Comandos a los Actuadores
- Conecta con la clase `Bimanual` para enviar las posiciones de articulación a los motores a través de buses CAN (`can0` y `can1`) a una frecuencia regulada por `RateLimiter` (típicamente 50-100 Hz).

---

## Ejemplo de Ejecución

```bash
python src/scripts/teleop/run_teleop.py
```
El visualizador Meshcat estará disponible por defecto en `http://127.0.0.1:7000/static/`.
