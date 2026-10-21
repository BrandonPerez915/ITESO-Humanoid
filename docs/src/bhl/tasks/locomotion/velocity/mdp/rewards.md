# rewards.py

## Descripción General

El módulo `rewards.py` contiene las funciones de recompensa y penalización específicas para entrenar políticas de marcha bípeda y humanoide estable en Isaac Lab. Modela el despegue de los pies, previene el arrastre de pies (foot slip/slide) y premia el seguimiento preciso de velocidades longitudinales, laterales y angulares.

---

## Funciones de Recompensa

### 1. `feet_air_time(...) -> torch.Tensor`
- **Propósito:** Recompensa el tiempo en que los pies permanecen en el aire durante la fase de balanceo (swing), incentivando zancadas amplias y dinámicas.
- **Cálculo:**
  $$R_{\text{air}} = \sum_{\text{pies}} (t_{\text{air}} - t_{\text{umbral}}) \cdot \mathbb{I}_{\text{primer contacto}}$$
- **Condición:** Si la velocidad comandada es menor a $0.1$ m/s (robot en reposo), la recompensa se anula para evitar que marche en su lugar innecesariamente.

---

### 2. `feet_air_time_positive_biped(...) -> torch.Tensor`
- **Propósito:** Variante especializada para robots bípedos que incentiva la alternancia estricta de pies (solo un pie en apoyo a la vez):
  $$\text{single\_stance} = (\text{pies en contacto} == 1)$$
- Premia el tiempo de permanencia en el modo de balanceo hasta un umbral máximo (`threshold`).

---

### 3. `feet_slide(...) -> torch.Tensor`
- **Propósito:** Penaliza el deslizamiento de las suelas contra el suelo para evitar marcha inestable y desgaste virtual de actuadores.
- **Cálculo:**
  Calcula la norma de la velocidad lineal del pie en el plano XY multiplicada por el estado binario del sensor de contacto:
  $$P_{\text{slide}} = \sum_{\text{pies}} \|\mathbf{v}_{xy}^{\text{pie}}\| \cdot \mathbb{I}(\|\mathbf{f}_{\text{contacto}}\| > 1.0\,\text{N})$$

---

### 4. `track_lin_vel_xy_yaw_frame_exp(...) -> torch.Tensor`
- **Propósito:** Recompensa el seguimiento de comandos de velocidad lineal en $X$ e $Y$ en el sistema de coordenadas alineado con la orientación (yaw) del torso:
  $$R_{\text{lin\_vel}} = \exp\left( - \frac{\|\mathbf{v}_{xy}^{\text{cmd}} - \mathbf{v}_{xy}^{\text{robot}}\|^2}{\sigma^2} \right)$$
- El uso de un kernel exponencial gaussiano asegura un gradiente suave y una recompensa máxima normalizada en 1 cuando el error es nulo.

---

### 5. `track_ang_vel_z_world_exp(...) -> torch.Tensor`
- **Propósito:** Recompensa el seguimiento de comandos de giro en guiñada (velocidad angular en $Z$):
  $$R_{\text{ang\_vel}} = \exp\left( - \frac{(\omega_z^{\text{cmd}} - \omega_z^{\text{robot}})^2}{\sigma^2} \right)$$
