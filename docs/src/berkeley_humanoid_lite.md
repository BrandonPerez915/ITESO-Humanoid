# berkeley_humanoid_lite.py

## Descripción General

El script `berkeley_humanoid_lite.py` es el archivo central de definición de la cinemática, dinámica y actuadores del robot para el simulador **NVIDIA Isaac Lab** (basado en Isaac Sim / Omniverse). Proporciona las configuraciones de articulación (`ArticulationCfg`) tanto para la versión bípeda (12 grados de libertad) como para la versión humanoide completa (22 grados de libertad).

---

## Estructura de Articulaciones

El archivo define tres listas fundamentales que mapean las articulaciones del robot:

### 1. Articulaciones de las Piernas (`HUMANOID_LITE_LEG_JOINTS` - 12 DOF)
Cada pierna consta de 6 grados de libertad:
- **Cadera (Hip):**
  - `leg_left_hip_roll_joint` / `leg_right_hip_roll_joint` (Aducción / Abducción)
  - `leg_left_hip_yaw_joint` / `leg_right_hip_yaw_joint` (Rotación interna / externa)
  - `leg_left_hip_pitch_joint` / `leg_right_hip_pitch_joint` (Flexión / Extensión)
- **Rodilla (Knee):**
  - `leg_left_knee_pitch_joint` / `leg_right_knee_pitch_joint` (Flexión de rodilla)
- **Tobillo (Ankle):**
  - `leg_left_ankle_pitch_joint` / `leg_right_ankle_pitch_joint` (Flexión plantar / Dorsiflexión)
  - `leg_left_ankle_roll_joint` / `leg_right_ankle_roll_joint` (Inversión / Eversión)

### 2. Articulaciones de los Brazos (`HUMANOID_LITE_ARM_JOINTS` - 10 DOF)
Cada brazo consta de 5 grados de libertad:
- **Hombro (Shoulder):**
  - `arm_left_shoulder_pitch_joint` / `arm_right_shoulder_pitch_joint`
  - `arm_left_shoulder_roll_joint` / `arm_right_shoulder_roll_joint`
  - `arm_left_shoulder_yaw_joint` / `arm_right_shoulder_yaw_joint`
- **Codo (Elbow):**
  - `arm_left_elbow_pitch_joint` / `arm_right_elbow_pitch_joint`
  - `arm_left_elbow_roll_joint` / `arm_right_elbow_roll_joint`

### 3. Modelo Humanoide Completo (`HUMANOID_LITE_JOINTS` - 22 DOF)
Unión de ambas listas: 10 articulaciones superiores + 12 articulaciones inferiores.

---

## Configuraciones de Simulación (`ArticulationCfg`)

### 1. `HUMANOID_LITE_BIPED_CFG` (Robot Bípedo)
Configura el robot con las piernas únicamente (sin torso superior ni brazos):
- **Asset USD:** `berkeley_humanoid_lite_biped.usd`
- **Propiedades de Cuerpo Rígido (`RigidBodyPropertiesCfg`):**
  - Gravedad habilitada (`disable_gravity=False`).
  - Amortiguamiento lineal y angular en 0 para física realista.
  - Velocidad máxima lineal y angular limitada a 1000 rad/s o m/s.
  - Velocidad máxima de despenetración: 1.0 m/s para estabilidad de contactos.
- **Propiedades del Solver (`ArticulationRootPropertiesCfg`):**
  - 8 iteraciones de posición y 4 iteraciones de velocidad en PhysX.
  - Autocolisiones deshabilitadas (`enabled_self_collisions=False`).
- **Postura Inicial (`InitialStateCfg`):**
  - Pose semi-agachada para facilitar el balance y despegue:
    - Pitch de cadera: `-0.2 rad`
    - Pitch de rodilla: `+0.4 rad`
    - Pitch de tobillo: `-0.3 rad`
- **Modelo de Actuadores (`ImplicitActuatorCfg`):**
  - **Cadera y Rodillas (`legs`):**
    - Límite de torque: `6.0 N·m`
    - Límite de velocidad: `10.0 rad/s`
    - Rigidez PD ($K_p$): `20.0`
    - Amortiguamiento PD ($K_d$): `2.0`
    - Inercia del rotor (armature): `0.007 kg·m²`
  - **Tobillos (`ankles`):**
    - Límite de torque: `6.0 N·m`
    - Rigidez PD ($K_p$): `20.0`
    - Amortiguamiento PD ($K_d$): `2.0`
    - Inercia del rotor (armature): `0.002 kg·m²`

### 2. `HUMANOID_LITE_CFG` (Humanoide Completo)
Extiende la configuración bípeda incorporando el tren superior:
- **Asset USD:** `berkeley_humanoid_lite.usd`
- **Postura Inicial:** Brazos extendidos a 0 rad y piernas en la pose semi-agachada idéntica al bípedo.
- **Actuadores Adicionales:**
  - **Brazos (`arms`):**
    - Límite de torque: `4.0 N·m`
    - Límite de velocidad: `10.0 rad/s`
    - Rigidez PD ($K_p$): `10.0`
    - Amortiguamiento PD ($K_d$): `2.0`
    - Inercia del rotor (armature): `0.002 kg·m²`

---

## Uso e Integración

Este archivo se importa en las tareas de aprendizaje por refuerzo dentro de Isaac Lab:
```python
from bhl.envs.isaac import HUMANOID_LITE_BIPED_CFG, HUMANOID_LITE_CFG
# O retrocompatible:
# from src.berkeley_humanoid_lite import HUMANOID_LITE_BIPED_CFG, HUMANOID_LITE_CFG
```
Permite instanciar entornos paralelizados en GPU donde cada actor clona estas propiedades físicas y de control.
