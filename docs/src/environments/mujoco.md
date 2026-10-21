# mujoco.py

## Descripción General

El módulo `mujoco.py` implementa el entorno de simulación física y visualización del robot Berkeley Humanoid Lite utilizando la biblioteca **MuJoCo**. Proporciona un entorno determinista de alta fidelidad para simulación sim-to-sim, verificación de controladores de aprendizaje por refuerzo (RL) e inspección cinemática en tiempo real de la teleoperación y de la telemetría del robot real (sim-to-real).

---

## Clases y Funciones Principales

### 1. `quat_rotate_inverse(q: torch.Tensor, v: torch.Tensor) -> torch.Tensor`
Rota un vector tridimensional $v$ por el inverso del cuaternión $q = [w, x, y, z]$:
$$v' = v (2w^2 - 1) - 2w(q_{vec} \times v) + 2 q_{vec}(q_{vec} \cdot v)$$
Se utiliza para proyectar la gravedad y las velocidades globales al sistema de coordenadas local del torso del robot.

---

### 2. Clase `MujocoEnv`
Clase base que inicializa el modelo y los datos de MuJoCo:
- Carga el archivo de escena XML correspondiente (`bhl_scene.xml` para el modelo de 22 grados de libertad o `bhl_biped_scene.xml` para el modelo bípedo de 12 DOF).
- Configura el paso de integración numérica de la física (`self.mj_model.opt.timestep = cfg.physics_dt`).
- Lanza el visor pasivo interactivo (`mujoco.viewer.launch_passive`).

---

### 3. Clase `MujocoVisualizer(MujocoEnv)`
Diseñada para visualizar estados en tiempo real sin simular la dinámica física (modo fantasma o sim-to-real):
- **`reset()`:** Restablece la posición y orientación de la base del robot al origen.
- **`step(robot_observations: np.ndarray)`:**
  - Extrae del vector de observaciones: cuaternión de base ($q$), velocidad angular ($\omega$), posiciones articulares ($q_{pos}$), velocidades articulares ($q_{vel}$) y modo de control.
  - Sobrescribe directamente `mj_data.qpos` y `mj_data.qvel`.
  - Llama a `mujoco.mj_step` y sincroniza el visor (`self.mj_viewer.sync()`).

---

### 4. Clase `MujocoSimulator(MujocoEnv)`
Implementa la simulación física completa con lazo cerrado de control PD:

#### Parámetros e Inicialización:
- **Controlador de entrada:** Instancia dinámicamente `Se2Keyboard`, `Se2Gamepad` o `Se2Ros2` según la configuración (`cfg.control_mode`).
- **Sub-stepping:** Calcula los pasos de física por cada paso de la política:
  $$\text{physics\_substeps} = \text{round}\left(\frac{\Delta t_{\text{policy}}}{\Delta t_{\text{physics}}}\right)$$
  Por ejemplo, con $\Delta t_{\text{policy}} = 0.02$ s (50 Hz) y $\Delta t_{\text{physics}} = 0.001$ s (1000 Hz), se realizan 20 sub-pasos físicos por ciclo.
- **Ganancias PD y Límites:** Inicializa tensores con $K_p$, $K_d$ y límites de esfuerzo (`effort_limits`) para cada articulación.

#### Métodos Clave:
- **`reset() -> torch.Tensor`:** Posiciona el robot en la postura inicial configurada (`cfg.default_base_position` y `cfg.default_joint_positions`) y retorna el primer vector de observaciones.
- **`step(actions: torch.Tensor) -> torch.Tensor`:**
  - Itera a través de los `physics_substeps`.
  - En cada sub-paso llama a `_apply_actions(actions)` e integra la física (`mujoco.mj_step`).
  - Mantiene la tasa de ejecución en tiempo real mediante `time.sleep` compensando el tiempo de procesamiento.
- **`_apply_actions(actions: torch.Tensor)`:**
  - Calcula el torque articular mediante control PD:
    $$\tau = K_p (q_{\text{target}} - q_{\text{current}}) - K_d \dot{q}_{\text{current}}$$
  - Aplica recorte según los límites de torque:
    $$\tau_{\text{clipped}} = \text{clamp}(\tau, -\tau_{\text{max}}, \tau_{\text{max}})$$
  - Asigna los torques al vector de control del motor `mj_data.ctrl[:]`.
- **`_get_observations() -> torch.Tensor`:**
  - Compone el vector de observaciones para la política RL concatenando:
    1. Cuaternión de la base $[w, x, y, z]$.
    2. Velocidad angular de la base $[\omega_x, \omega_y, \omega_z]$.
    3. Posiciones articulares normalizadas.
    4. Velocidades articulares.
    5. Comandos del usuario: `[mode, v_x, v_y, \omega_z]`.

---

## Ejemplo de Integración

```python
from bhl.envs.mujoco import MujocoSimulator, Cfg

# Cargar configuración desde YAML o línea de comandos
cfg = Cfg.from_arguments()

# Instanciar el simulador
env = MujocoSimulator(cfg)
obs = env.reset()

# Ciclo de control
while True:
    actions = my_policy(obs)
    obs = env.step(actions)
```
