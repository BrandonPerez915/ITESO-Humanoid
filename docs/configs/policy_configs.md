# Configuraciones de Políticas (configs/*.yaml)

## Descripción General

El directorio `src/configs/` alberga las especificaciones en formato **YAML** para desplegar las diferentes variantes de políticas de aprendizaje por refuerzo tanto en el simulador MuJoCo como en el robot físico.

---

## Archivos de Configuración Disponibles

1. **`policy_biped_50hz.yaml`:**
   - Política para robot bípedo (12 grados de libertad) ejecutándose a una tasa de inferencia de **50 Hz** ($\Delta t_{\text{policy}} = 0.02$ s).
   - Es la configuración estándar y más reactiva para marcha dinámica.
2. **`policy_biped_25hz_a.yaml` y `policy_biped_25hz_b.yaml`:**
   - Variantes para robot bípedo operando a **25 Hz** ($\Delta t_{\text{policy}} = 0.04$ s).
   - Reducen la carga de procesamiento en la CPU embebida.
3. **`policy_humanoid.yaml`:**
   - Configuración para el robot humanoide completo con tren superior (22 grados de libertad).
4. **`policy_humanoid_legs.yaml`:**
   - Variante donde se mantiene el torso y los brazos en posición neutra mientras la política controla activamente las piernas.
5. **`policy_video.yaml`:**
   - Ajustes específicos con tasas de integración suavizadas para renderizado y captura de demostraciones en video.

---

## Estructura de Parámetros en los Archivos YAML

```yaml
policy_checkpoint_path: "checkpoints/policy_biped_50hz.onnx" # Ruta al modelo ONNX
ip_robot_addr: 127.0.0.1       # Dirección IP del robot
ip_policy_obs_port: 10000      # Puerto UDP de observaciones
ip_host_addr: 127.0.0.1        # Dirección IP del host
ip_policy_acs_port: 10001      # Puerto UDP de acciones
control_dt: 0.004              # Paso temporal de control (250 Hz)
policy_dt: 0.02                # Paso temporal de la política RL (50 Hz)
physics_dt: 0.0005             # Paso temporal de integración física (2000 Hz)
num_joints: 12                 # Número de articulaciones

# Listado ordenado de articulaciones
joints:
  - leg_left_hip_roll_joint
  # ... articulaciones restantes

# Ganancias proporcionales PD
joint_kp: [20.0, 20.0, ...]

# Ganancias derivativas PD
joint_kd: [2.0, 2.0, ...]

# Límites máximos de torque (N·m)
effort_limits: [5.0, 5.0, ...]

# Postura neutra por defecto (radianes)
default_joint_positions: [0.0, 0.0, -0.2, 0.4, -0.3, 0.0, ...]
```
