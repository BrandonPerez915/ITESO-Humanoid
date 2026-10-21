# velocity_env_cfg.py

## Descripción General

El archivo `velocity_env_cfg.py` define la configuración base del entorno de simulación para tareas de seguimiento de velocidad de locomoción (`LocomotionVelocityEnvCfg`), así como la configuración del escenario interactivo con terreno plano (`FlatTerrainSceneCfg`) en **Isaac Lab**.

---

## Estructuras y Clases de Configuración

### 1. `FlatTerrainSceneCfg(InteractiveSceneCfg)`
Define los elementos estáticos y dinámicos que componen la escena de simulación:
- **Terreno (`terrain`):** Instancia un plano infinito (`TerrainImporterCfg` con tipo `"plane"`). Configura propiedades de contacto con fricción estática y dinámica de `1.0` y materiales visuales PBR (NVIDIA Nucleus).
- **Robot (`robot`):** Espacio reservado (`MISSING`), completado por las configuraciones específicas del robot bípedo o humanoide (`ArticulationCfg`).
- **Sensores de Contacto (`contact_forces`):** Instancia `ContactSensorCfg` monitoreando colisiones en el robot (`{ENV_REGEX_NS}/robot/.*`), con un historial de 3 pasos (`history_length = 3`) y seguimiento activo de tiempo de vuelo en el aire (`track_air_time = True`).
- **Iluminación:** Luz distante principal (`DistantLightCfg` con intensidad 3000) y luz ambiental envolvente (`DomeLightCfg` con intensidad 1000).

---

### 2. `LocomotionVelocityEnvCfg(ManagerBasedRLEnvCfg)`
Hereda de la configuración base de entornos dirigidos por administradores (Manager-Based RL) de Isaac Lab:
- **Escenario (`scene`):** Configurado por defecto para simular `4096` entornos en paralelo con una separación de `2.5` metros entre robots.
- **Paso Temporal de Simulación y Decimación (`__post_init__`):**
  - **Física:** $\Delta t_{\text{sim}} = 0.005$ s (200 Hz).
  - **Decimación (`decimation = 4`):** La política de RL toma decisiones cada 4 pasos de física:
    $$\Delta t_{\text{policy}} = 4 \times 0.005\,\text{s} = 0.02\,\text{s} \quad (50\,\text{Hz})$$
  - **Duración del Episodio:** `episode_length_s = 20.0` s (1000 pasos de política).
  - **Optimización de GPU PhysX:**
    - `gpu_max_rigid_patch_count = 10 * 2**15` para alojar simultáneamente todos los parches de contacto en memoria VRAM.
    - Sincronización del periodo de actualización de los sensores de contacto con el paso de física.
