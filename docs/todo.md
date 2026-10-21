# Plan de Implementación y Simulación: Berkeley Humanoid Lite

Este documento recopila el análisis exhaustivo del ecosistema **Berkeley Humanoid Lite** (desarrollado por el laboratorio **Hybrid Robotics** de UC Berkeley) y define la hoja de ruta detallada en formato TODO (`[ ]`) para implementar, ejecutar y personalizar la simulación física y el entrenamiento por refuerzo (RL).

---

## 1. Análisis del Proyecto y Ecosistema de Repositorios

### 1.1 Repositorios Principales
1. **[HybridRobotics/berkeley-humanoid-lite](https://github.com/HybridRobotics/berkeley-humanoid-lite)** (Repositorio central):
   - **Estructura tipo Isaac Lab Extension**: Contiene las definiciones de entornos MDP, scripts de entrenamiento (RSL-RL), validación sim2sim (MuJoCo), pipelines de teleoperación y políticas preentrenadas en formato ONNX.
2. **[HybridRobotics/berkeley-humanoid-lite-assets](https://github.com/HybridRobotics/berkeley-humanoid-lite-assets)** (Submódulo de activos):
   - Archivos de descripción cinemática y dinámica: **URDF**, **MJCF (MuJoCo XML)**, **USD (Universal Scene Description para Isaac Sim)**, scripts CAD Onshape (`onshape-to-robot`) y mallas STL.
3. **[HybridRobotics/berkeley-humanoid-lite-lowlevel](https://github.com/HybridRobotics/berkeley-humanoid-lite-lowlevel)** (Submódulo de control bajo nivel):
   - Drivers de comunicación CAN (`start_can_transports.sh`), lectura de IMU, interfaz de mando/gamepad (`Se2Gamepad`), arquitectura de red UDP entre bucle de control y simulación/hardware, y controlador en C/C++ (`csrc/`).
4. **Repositorios relacionados del laboratorio**:
   - `HybridRobotics/isaac_berkeley_humanoid`: Entorno previo de entrenamiento en Isaac Gym para el Berkeley Humanoid original.
   - `HybridRobotics/berkeley_humanoid_description`: Descripción URDF original.
   - `recoil-motor-controller-besc`: Firmware para los controladores de motor brushless con FOC.

---

### 1.2 Arquitectura del Robot y Simulación

```
                           +-------------------------------------+
                           |      Mando / Joystick / Teleop      |
                           +------------------+------------------+
                                              | Velocidades (Vx, Vy, Wz) / IK
                                              v
      +---------------------------------------------------------------------------------+
      |                           BUCLE DE CONTROL (RL Controller)                      |
      | - Política ONNX (25 Hz / 50 Hz)                                                 |
      | - Observaciones: Cuaternión base, Vel. angular, Pos/Vel articular, Comandos     |
      | - Acciones: Posiciones articulares objetivo -> Control PD (Kp=20, Kd=2)         |
      +---------------------------------------+-----------------------------------------+
                                              | UDP (Puertos 10000 / 10001)
                      +-----------------------+-----------------------+
                      |                                               |
                      v                                               v
        +----------------------------+                 +----------------------------+
        |   MuJoCo (Sim2Sim Local)   |                 |  Isaac Lab (Entrenamiento) |
        | - Motor físico CPU/GPU     |                 | - Isaac Sim 4.5.0 + Lab 2.1|
        | - Compatible macOS & Linux |                 | - Simulación paralela GPU  |
        | - Modelos MJCF:            |                 | - Entrenamiento RSL-RL     |
        |   * bhl_scene.xml (22 DOF) |                 | - Exportación a ONNX       |
        |   * bhl_biped_scene.xml    |                 | - Modelos USD              |
        +----------------------------+                 +----------------------------+
```

### 1.3 Grados de Libertad (DOF)
- **Modo Bípedo (`Velocity-Berkeley-Humanoid-Lite-Biped-v0`)**: **12 DOF** (Piernas: Cadera Roll/Yaw/Pitch, Rodilla Pitch, Tobillo Pitch/Roll x 2).
- **Modo Humanoide Completo (`Velocity-Berkeley-Humanoid-Lite-v0`)**: **22 DOF** (12 DOF piernas + 10 DOF torso, brazos y cuello).

---

## 2. Hoja de Ruta de Tareas (TODO)

### Fase 1: Clonación y Preparación del Entorno Base
- [ ] **1.1. Clonar el repositorio y submódulos**
  - [ ] Clonar el repositorio raíz de forma recursiva:
    ```bash
    git clone --recurse-submodules https://github.com/HybridRobotics/berkeley-humanoid-lite.git
    cd berkeley-humanoid-lite
    ```
  - [ ] Verificar que `source/berkeley_humanoid_lite_assets` y `source/berkeley_humanoid_lite_lowlevel` estén correctamente inicializados (`git submodule status`).
- [ ] **1.2. Instalar el gestor de paquetes Python (`uv`)**
  - [ ] Instalar `uv` para entornos reproducibles y ultrarrápidos:
    ```bash
    curl -LsSf https://astral.sh/uv/install.sh | sh
    ```
- [ ] **1.3. Determinar la estrategia según el Sistema Operativo**:
  - [ ] **Entorno macOS (Equipo Local Actual)**:
    - Excelente para **Sim2Sim con MuJoCo** (evaluación inmediata con modelos ONNX preentrenados).
    - Creación de entorno virtual Python 3.10 o 3.11 con `uv`.
  - [ ] **Entorno Linux / Workstation con GPU NVIDIA (RTX 3080/4090/A6000)**:
    - Requerido para **Isaac Sim 4.5.0 + Isaac Lab 2.1.0** y entrenamiento masivo paralelo con CUDA.
    - Se puede utilizar máquina remota, dual-boot o contenedor Docker (`docker/docker-compose.yaml`).

---

### Fase 2: Puesta en Marcha de la Simulación Sim2Sim en MuJoCo (Local / macOS & Linux)
- [ ] **2.1. Instalar dependencias para MuJoCo**
  - [ ] Instalar paquetes esenciales:
    ```bash
    uv pip install mujoco numpy torch onnxruntime pygame pyyaml
    ```
  - [ ] Instalar los submódulos en modo editable:
    ```bash
    uv pip install -e ./source/berkeley_humanoid_lite/
    uv pip install -e ./source/berkeley_humanoid_lite_assets/
    uv pip install -e ./source/berkeley_humanoid_lite_lowlevel/
    ```
- [ ] **2.2. Verificar los archivos MJCF y mallas**
  - [ ] Confirmar existencia de:
    - `source/berkeley_humanoid_lite_assets/data/robots/berkeley_humanoid/berkeley_humanoid_lite/mjcf/bhl_scene.xml`
    - `source/berkeley_humanoid_lite_assets/data/robots/berkeley_humanoid/berkeley_humanoid_lite/mjcf/bhl_biped_scene.xml`
  - [ ] Probar carga visual rápida en el visualizador nativo de MuJoCo:
    ```bash
    python -m mujoco.viewer --mjcf=source/berkeley_humanoid_lite_assets/data/robots/berkeley_humanoid/berkeley_humanoid_lite/mjcf/bhl_scene.xml
    ```
- [ ] **2.3. Configurar interfaz de comandos (Mando / Gamepad o Teclado)**
  - [ ] Conectar mando compatible (Xbox, PlayStation o genérico USB/Bluetooth).
  - [ ] *Alternativa*: En caso de no tener mando físico, crear un wrapper de teclado o script virtual para publicar comandos de velocidad `(velocity_x, velocity_y, velocity_yaw)` vía UDP al puerto local `10000`.
- [ ] **2.4. Ejecutar la simulación Sim2Sim con modelos ONNX incluidos**
  - [ ] Probar bípedo a 25 Hz:
    ```bash
    uv run ./scripts/sim2sim/play_mujoco.py --config ./configs/policy_biped_25hz_a.yaml
    ```
  - [ ] Probar humanoide completo (22 DOF):
    ```bash
    uv run ./scripts/sim2sim/play_mujoco.py --config ./configs/policy_humanoid.yaml
    ```
  - [ ] Evaluar estabilidad del caminar, respuesta a cambios de velocidad y caída ante perturbaciones.

---

### Fase 3: Configuración del Entorno de Entrenamiento en Isaac Sim / Isaac Lab (Linux + NVIDIA GPU)
- [ ] **3.1. Requisitos de sistema en servidor/host Linux**
  - [ ] Ubuntu 22.04 o 24.04 con drivers NVIDIA recientes (>= 535) y CUDA 12.1.
  - [ ] Paquetes del sistema: `sudo apt install cmake build-essential net-tools git`.
- [ ] **3.2. Instalación de PyTorch e Isaac Sim / Isaac Lab**
  - [ ] Instalar PyTorch compatible con CUDA 12.1:
    ```bash
    pip install torch==2.5.1 torchvision==0.20.1 --index-url https://download.pytorch.org/whl/cu121
    ```
  - [ ] Instalar NVIDIA Isaac Sim 4.5.0:
    ```bash
    pip install "isaacsim[all,extscache]==4.5.0" --extra-index-url https://pypi.nvidia.com
    ```
  - [ ] Instalar Isaac Lab 2.1.0:
    ```bash
    pip install "isaaclab[isaacsim,all]==2.1.0" --extra-index-url https://pypi.nvidia.com
    ```
- [ ] **3.3. Configuración alternativa mediante Docker**
  - [ ] Revisar `docker/Dockerfile` y `docker/docker-compose.yaml`.
  - [ ] Construir y levantar el contenedor con soporte NVIDIA Container Toolkit:
    ```bash
    cd docker
    docker compose up -d
    docker compose exec berkeley_humanoid_lite bash
    ```
- [ ] **3.4. Registro y validación de tareas**
  - [ ] Listar entornos registrados en Isaac Lab para comprobar integración:
    ```bash
    python ./scripts/list_envs.py
    ```
  - [ ] Validar que aparezcan `Velocity-Berkeley-Humanoid-Lite-v0` y `Velocity-Berkeley-Humanoid-Lite-Biped-v0`.

---

### Fase 4: Entrenamiento por Refuerzo (Reinforcement Learning con RSL-RL)
- [ ] **4.1. Entrenamiento de locomoción bípeda (12 DOF)**
  - [ ] Ejecutar entrenamiento en modo headless (4096 entornos paralelos por defecto, ~6000 iteraciones / 2 horas):
    ```bash
    python ./scripts/rsl_rl/train.py --task Velocity-Berkeley-Humanoid-Lite-Biped-v0 --headless
    ```
  - [ ] Monitorear curvas de recompensa en TensorBoard / WandB:
    - Términos de seguimiento de velocidad lineal y angular.
    - Penalizaciones por contacto de pies, pares de torsión excesivos y aceleración articular.
- [ ] **4.2. Entrenamiento de humanoide completo (22 DOF)**
  - [ ] Ejecutar entrenamiento del cuerpo completo:
    ```bash
    python ./scripts/rsl_rl/train.py --task Velocity-Berkeley-Humanoid-Lite-v0 --headless
    ```
- [ ] **4.3. Visualización y Exportación de la Política Entrenada**
  - [ ] Cargar el último checkpoint y visualizar en Isaac Sim GUI:
    ```bash
    python ./scripts/rsl_rl/play.py --task Velocity-Berkeley-Humanoid-Lite-v0 --num_envs 16
    ```
  - [ ] Comprobar que `play.py` exporte automáticamente el modelo a `.onnx` y actualice `configs/policy_latest.yaml`.
- [ ] **4.4. Ciclo Sim-to-Sim de la nueva política**
  - [ ] Probar la nueva política exportada en MuJoCo:
    ```bash
    python ./scripts/sim2sim/play_mujoco.py --config ./configs/policy_latest.yaml
    ```

---

### Fase 5: Pipeline de Activos CAD y Modificación del Robot
- [ ] **5.1. Personalización de modelos con Onshape (Opcional si se modifica el hardware)**
  - [ ] Configurar credenciales de API Onshape (`ONSHAPE_ACCESS_KEY`, `ONSHAPE_SECRET_KEY`).
  - [ ] Instalar dependencias de conversión: `sudo apt install openscad meshlab`.
  - [ ] Exportar nuevo URDF desde Onshape:
    ```bash
    uv run ./source/berkeley_humanoid_lite_assets/scripts/export_onshape_to_urdf.py --config ./source/berkeley_humanoid_lite_assets/data/robots/berkeley_humanoid/berkeley_humanoid_lite/urdf/config.json
    ```
  - [ ] Convertir URDF a MJCF:
    ```bash
    uv run ./source/berkeley_humanoid_lite_assets/scripts/export_onshape_to_mjcf.py --config ./source/berkeley_humanoid_lite_assets/data/robots/berkeley_humanoid/berkeley_humanoid_lite/mjcf/config.json
    ```
  - [ ] Convertir URDF a USD para Isaac Sim:
    ```bash
    uv run ./source/berkeley_humanoid_lite_assets/scripts/convert_urdf_to_usd.py ./source/berkeley_humanoid_lite_assets/data/robots/berkeley_humanoid/berkeley_humanoid_lite/urdf/berkeley_humanoid_lite.urdf
    ```
- [ ] **5.2. Ajuste de parámetros dinámicos y de actuadores**
  - [ ] Calibrar fricciones, amortiguamientos y límites de esfuerzo (`effort_limits`) según las curvas reales del motor FOC y reductora cicloidal.
  - [ ] Ajustar ganancias PD en los configs YAML (`joint_kp`, `joint_kd`).

---

### Fase 6: Teleoperación, Manipulación y Cinemática Inversa (IK)
- [ ] **6.1. Pruebas del solver cinemático de brazos**
  - [ ] Ejecutar el test del solver IK:
    ```bash
    python ./scripts/teleop/test_solver.py
    ```
  - [ ] Probar el control de pinzas / grippers:
    ```bash
    python ./scripts/teleop/test_gripper.py
    ```
- [ ] **6.2. Teleoperación con SteamVR / VR Bridge**
  - [ ] Configurar `steamvr-bridge` si se dispone de visores/trackers de realidad virtual.
  - [ ] Ejecutar `scripts/teleop/run_teleop.py` para mapear posturas de brazos en tiempo real sobre la simulación.
- [ ] **6.3. Integración futura con LeRobot (Hugging Face)**
  - [ ] Monitorear el soporte de recolección de datos de teleoperación y aprendizaje por imitación (Imitation Learning / ACT / Diffusion Policy) en `lerobot`.

---

### Fase 7: Preparación para Despliegue Sim2Real
- [ ] **7.1. Bucle de control UDP aislado**
  - [ ] Probar la separación por hilos y sockets UDP entre el generador de observaciones y el inferenciador de la red neuronal.
  - [ ] Medir latencias de inferencia de ONNX Runtime (debe mantenerse holgadamente por debajo de 40 ms para 25 Hz o 20 ms para 50 Hz).
- [ ] **7.2. Inyección de perturbaciones y Randomización de Dominio**
  - [ ] Comprobar en Isaac Lab la configuración de `events.py` (ruido en observaciones IMU, variaciones de masa de eslabones, empujes laterales aleatorios, fricción del suelo).
  - [ ] Asegurar que la política sea resiliente antes de transferir a hardware físico.

---

## 3. Comandos Rápidos de Referencia

| Acción | Comando |
| :--- | :--- |
| **Sim2Sim MuJoCo (Bípedo 25Hz)** | `python ./scripts/sim2sim/play_mujoco.py --config ./configs/policy_biped_25hz_a.yaml` |
| **Sim2Sim MuJoCo (Humanoide 22 DOF)** | `python ./scripts/sim2sim/play_mujoco.py --config ./configs/policy_humanoid.yaml` |
| **Entrenar Bípedo (Isaac Lab)** | `python ./scripts/rsl_rl/train.py --task Velocity-Berkeley-Humanoid-Lite-Biped-v0 --headless` |
| **Entrenar Humanoide (Isaac Lab)** | `python ./scripts/rsl_rl/train.py --task Velocity-Berkeley-Humanoid-Lite-v0 --headless` |
| **Visualizar & Exportar ONNX** | `python ./scripts/rsl_rl/play.py --task Velocity-Berkeley-Humanoid-Lite-v0 --num_envs 16` |
| **Verificar Solver IK** | `python ./scripts/teleop/test_solver.py` |
| **Verificar Conexión CAN (Hardware)**| `python3 ./source/berkeley_humanoid_lite_lowlevel/berkeley_humanoid_lite_lowlevel/robot/check_connection.py` |
