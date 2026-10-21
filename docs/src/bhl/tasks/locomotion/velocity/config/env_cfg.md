# env_cfg.py (Configuración de Tarea MDP)

## Descripción General

Los archivos `env_cfg.py` en `config/biped/` y `config/humanoid/` definen el Proceso de Decisión de Markov (MDP) completo para el entrenamiento de locomoción: espacio de comandos, observaciones del actor y crítico, acciones, recompensas, aleatorización de dominio y condiciones de término.

---

## Estructura del MDP

### 1. Comandos (`CommandsCfg`)
- Genera comandos uniformes de velocidad planar:
  - Velocidad longitudinal ($V_x$): $[-0.5, 0.5]$ m/s.
  - Velocidad lateral ($V_y$): $[-0.25, 0.25]$ m/s.
  - Velocidad de giro ($\omega_z$): $[-1.0, 1.0]$ rad/s.
  - Orientación angular: $[-\pi, \pi]$ rad.
- Intervalo de remuestreo: cada 10 segundos.

### 2. Espacio de Observaciones (`ObservationsCfg`)
- **Grupo de Política (Actor):**
  - Comandos de velocidad normalizados.
  - Velocidad angular del torso (con ruido aditivo uniforme $\pm 0.3$ rad/s).
  - Vector de gravedad proyectado en el torso (con ruido $\pm 0.05$).
  - Posiciones relativas de las articulaciones (con ruido $\pm 0.05$ rad).
  - Velocidades relativas de las articulaciones (con ruido $\pm 2.0$ rad/s).
  - Últimas acciones ejecutadas en el paso anterior.
- **Grupo de Crítico:**
  - Incluye todas las observaciones anteriores sin ruido artificial + velocidad lineal real del torso en el mundo (`base_lin_vel`) como información privilegiada (Asymmetric Actor-Critic).

### 3. Espacio de Acciones (`ActionsCfg`)
- Control por posición articular:
  $$q_{\text{target}} = q_{\text{default}} + \text{scale} \times a_t$$
  con $\text{scale} = 0.25$ rad.

### 4. Ponderaciones de Recompensa (`RewardsCfg`)
- **Seguimiento de comandos:** Recompensas altas para velocidad lineal en $XY$ y giro en $Z$.
- **Calidad de marcha:** Recompensa por tiempo de vuelo de los pies y alternancia de apoyo.
- **Regularización y suavidad:** Penalizaciones por velocidad articular, aceleración, cambios bruscos de acción (smoothness), deslizamiento de pies y contacto de partes no deseadas (torso, rodillas) con el suelo.
