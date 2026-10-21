# play.py

## Descripción General

El script `play.py` permite cargar, evaluar e inspeccionar interactivamente puntos de control (checkpoints) entrenados con RSL-RL dentro del simulador Isaac Sim. Asimismo, incorpora utilidades automáticas para exportar la política entrenada a formatos portables como **TorchScript (JIT)** y **ONNX**, permitiendo su posterior despliegue en simuladores externos (MuJoCo) o en el robot físico.

---

## Flujo de Ejecución y Capacidades

1. **Carga del Entorno de Evaluación:**
   - Inicializa Isaac Sim con interfaz gráfica (o headless si se especifica).
   - Instancia el entorno con una menor cantidad de robots (por ejemplo 16 o 32 entornos para visualización cómoda).
2. **Localización y Carga de Checkpoint:**
   - Resuelve automáticamente la ruta al último punto de control mediante `get_checkpoint_path(...)` o el archivo indicado en `--checkpoint`.
   - Restaura los pesos del actor de la red neuronal.
3. **Exportación de la Política (ONNX / JIT):**
   - Extrae el sub-módulo de inferencia (`policy = runner.get_inference_policy(...)`).
   - Llama a `export_policy_as_jit` y `export_policy_as_onnx`.
   - Esto genera los archivos `.onnx` listos para ser consumidos por el controlador de bajo nivel de Berkeley Humanoid Lite.
4. **Bucle de Ejecución y Visualización:**
   - Itera de forma continua calculando acciones a partir de las observaciones recibidas del entorno.
   - Aplica las acciones al robot en Isaac Sim.
   - Permite al usuario interactuar mediante teclado o gamepad para enviar comandos de velocidad longitudinal, lateral y de giro.

---

## Argumentos Clave

| Argumento | Tipo | Descripción |
| :--- | :--- | :--- |
| `--task` | `str` | Nombre del entorno (obligatorio). |
| `--load_run` | `str` | Nombre de la carpeta de la ejecución entrenada. |
| `--checkpoint` | `str` | Ruta al archivo `.pt` específico a cargar. |
| `--num_envs` | `int` | Número de robots en escena durante la visualización. |
| `--video` | `flag` | Graba un clip de video de la simulación. |

---

## Ejemplo de Uso

```bash
# Visualizar la política entrenada y exportarla a ONNX
python src/scripts/rsl_rl/play.py \
  --task Template-Isaac-Velocity-Flat-Berkeley-Humanoid-Lite-v0 \
  --num_envs 16 \
  --load_run 2025-01-01_12-00-00
```
