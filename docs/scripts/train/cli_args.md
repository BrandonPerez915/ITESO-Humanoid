# cli_args.py

## Descripción General

El módulo `cli_args.py` centraliza la gestión y análisis de argumentos por línea de comandos para el framework de aprendizaje por refuerzo **RSL-RL** en su integración con Isaac Lab. Proporciona utilidades para parsear hiperparámetros de entrenamiento, gestión de checkpoints, nombres de ejecuciones y logging remoto.

---

## Funciones Principales

### 1. `add_rsl_rl_args(parser: argparse.ArgumentParser)`
Agrega un grupo específico de argumentos (`rsl_rl`) al analizador CLI:
- `--experiment_name`: Nombre del directorio base donde se almacenan los logs y pesos.
- `--run_name`: Sufijo identificador para la corrida actual.
- `--resume`: Booleano para reanudar el entrenamiento desde un punto de control.
- `--load_run`: Carpeta de la ejecución previa de la cual restaurar pesos.
- `--checkpoint`: Archivo específico `.pt` del checkpoint a cargar.
- `--logger`: Servicio de telemetría a emplear (`"wandb"`, `"tensorboard"`, o `"neptune"`).
- `--log_project_name`: Nombre del proyecto en Weights & Biases o Neptune.

---

### 2. `parse_rsl_rl_cfg(task_name: str, args_cli: argparse.Namespace) -> RslRlOnPolicyRunnerCfg`
- Extrae la configuración por defecto del agente PPO registrada para la tarea solicitada (`task_name`) mediante `load_cfg_from_registry`.
- Invoca `update_rsl_rl_cfg` para sobreescribir los valores por defecto con las opciones pasadas por la terminal.
- Retorna el objeto `RslRlOnPolicyRunnerCfg` listo para inicializar el runner.

---

### 3. `update_rsl_rl_cfg(agent_cfg: RslRlOnPolicyRunnerCfg, args_cli: argparse.Namespace)`
- Aplica las sobreescrituras explícitas:
  - Semilla del generador aleatorio (`seed`).
  - Flags de reanudación y checkpoints.
  - Proyectos remotos de telemetría (`wandb_project` y `neptune_project`).
