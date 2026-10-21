# rsl_rl_ppo_cfg.py (Hiperparámetros de PPO)

## Descripción General

El archivo `rsl_rl_ppo_cfg.py` establece la arquitectura de redes neuronales y los hiperparámetros del algoritmo **PPO (Proximal Policy Optimization)** gestionado por el paquete `rsl_rl`.

---

## Hiperparámetros Principales

### 1. Algoritmo (`PpoCfg`)
- **Tasa de aprendizaje (Learning Rate):** $1.0 \times 10^{-3}$ con schedule adaptativo.
- **Factor de descuento ($\gamma$):** $0.99$.
- **Parámetro GAE ($\lambda$):** $0.95$.
- **Clip de PPO ($\epsilon$):** $0.2$.
- **Coeficiente de entropía:** $0.01$ (fomenta exploración inicial).
- **Mini-batches:** Divide las transiciones en sub-lotes para actualizar con Adam.
- **Épocas por iteración:** Típicamente 5 épocas por cada colección de rollout.

### 2. Red Neuronal de Actor y Crítico (`PolicyCfg`)
- **Arquitectura:** Redes multicapa (MLP) totalmente conectadas.
- **Capas ocultas:** Típicamente `[512, 256, 128]` neuronas.
- **Funciones de activación:** `ELU` (Exponential Linear Unit) para gradientes suaves y estabilidad numérica.
- **Varianza de salida:** Desviación estándar inicial de acciones parametrizada y entrenable.

### 3. Configuración del Runner (`RslRlOnPolicyRunnerCfg`)
- **Horizonte de rollout (`num_steps_per_env`):** 24 pasos por entorno antes de cada optimización.
- **Total de iteraciones:** 1500 a 3000 iteraciones.
- **Intervalo de guardado de checkpoints:** Cada 50 a 100 iteraciones.
