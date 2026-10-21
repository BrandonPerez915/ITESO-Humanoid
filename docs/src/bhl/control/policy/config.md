# config.py

## Descripción General

El módulo `config.py` implementa el cargador y validador de configuración del sistema de control del Berkeley Humanoid Lite. Emplea **OmegaConf** para leer y fusionar parámetros desde archivos YAML, variables de entorno y argumentos de línea de comandos.

---

## Clase `Cfg`

La clase `Cfg` encapsula todos los parámetros operativos requeridos por la simulación y el robot real:

### 1. Parámetros Clave
- **Frecuencias y Tiempos:**
  - `policy_dt`: Periodo de muestreo de la política (ej. 0.02 s para 50 Hz).
  - `physics_dt`: Periodo de integración de la física (ej. 0.001 s para 1000 Hz).
  - `control_dt`: Periodo del lazo de control de bajo nivel (ej. 0.001 s).
- **Dimensiones y Topología:**
  - `num_joints`: Número total de articulaciones activas (12 para bípedo, 22 para humanoide completo).
  - `action_indices`: Lista de índices de articulaciones controladas activamente por la política.
- **Límites Físicos y Ganancias PD:**
  - `joint_kp`: Vector de coeficientes proporcionales para cada articulación.
  - `joint_kd`: Vector de coeficientes derivativos para cada articulación.
  - `effort_limits`: Vector de límites máximos de torque ($N\cdot m$).
  - `default_joint_positions`: Postura articular neutra por defecto.
- **Rutas de Modelos:**
  - `checkpoint_path`: Ruta al archivo de pesos `.onnx` o `.pt`.
  - `control_mode`: Modo de entrada (`"gamepad"`, `"keyboard"` o `"ros2"`).

### 2. Métodos de Carga
- **`Cfg.from_file(path: str)`:** Carga directamente la configuración desde un archivo YAML.
- **`Cfg.from_arguments()`:** Parsea automáticamente el argumento `--config` desde la línea de comandos, permitiendo intercambiar configuraciones en tiempo de ejecución de manera sencilla.
