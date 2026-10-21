# train.py

## Descripción General

El script `train.py` es el punto de entrada principal para entrenar políticas de locomoción bípeda y humanoide mediante el algoritmo **PPO (Proximal Policy Optimization)** utilizando la biblioteca de aprendizaje por refuerzo en GPU **RSL-RL** y **Isaac Lab**.

---

## Flujo de Entrenamiento y Arquitectura

1. **Inicialización de Isaac Sim en GPU:**
   - Lee los argumentos de simulación (número de entornos, renderizado, grabación de video).
   - Inicia `AppLauncher` para inicializar el contexto de CUDA, Omniverse y PhysX.
2. **Carga y Fusión de Configuraciones (Hydra + CLI):**
   - Utiliza el decorador `@hydra_task_config(args_cli.task, "rsl_rl_cfg_entry_point")`.
   - Carga la configuración del entorno (`ManagerBasedRLEnvCfg`) y del agente (`RslRlOnPolicyRunnerCfg`).
   - Aplica semillas deterministas (`torch.manual_seed`, semillas de entorno).
   - Ajusta optimizaciones de precisión como TensorFloat-32 (`torch.backends.cuda.matmul.allow_tf32 = True`).
3. **Instanciación del Entorno Vectorizado:**
   - Instancia miles de entornos paralelos en GPU (típicamente 4096 robots simultáneos).
   - Envuelve el entorno en `RslRlVecEnvWrapper` para adaptar tensores de PyTorch al formato requerido por RSL-RL.
4. **Instanciación del Runner PPO:**
   - Instancia `OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=log_dir, device=agent_cfg.device)`.
   - Configura el logging (TensorBoard / W&B) y la grabación periódica de videos de evaluación en caso de habilitar `--video`.
5. **Bucle de Optimización:**
   - Ejecuta `runner.learn(num_learning_iterations=agent_cfg.max_iterations, init_at_random_ep_len=True)`.
   - Durante el ciclo:
     - Recolecta trayectorias en paralelo.
     - Calcula estimaciones de ventajas con GAE (Generalized Advantage Estimation).
     - Actualiza las redes de actor y crítico mediante descensos de gradiente.
     - Guarda checkpoints periódicos en el disco.
6. **Finalización:**
   - Guarda el modelo final entrenado y cierra la instancia de simulación limpiamente.

---

## Argumentos de Línea de Comandos Más Importantes

| Argumento | Tipo | Descripción |
| :--- | :--- | :--- |
| `--task` | `str` | Nombre del entorno registrado (ej. `Template-Isaac-Velocity-Flat-Berkeley-Humanoid-Lite-v0`). |
| `--num_envs` | `int` | Número de entornos paralelos simulados en GPU (ej. 4096). |
| `--max_iterations` | `int` | Cantidad total de iteraciones de entrenamiento. |
| `--video` | `flag` | Activa la captura periódica de video de la simulación. |
| `--logger` | `str` | Servicio de logs: `tensorboard`, `wandb` o `neptune`. |
| `--checkpoint` | `str` | Ruta al archivo `.pt` para reanudar el entrenamiento. |

---

## Ejemplo de Ejecución

```bash
# Entrenar la política del robot bípedo con 4096 entornos paralelos
python src/scripts/rsl_rl/train.py \
  --task Template-Isaac-Velocity-Flat-Berkeley-Humanoid-Lite-v0 \
  --num_envs 4096 \
  --headless \
  --logger tensorboard
```
