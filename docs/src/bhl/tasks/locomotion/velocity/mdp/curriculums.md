# curriculums.py

## Descripción General

El módulo `curriculums.py` implementa mecanismos de **aprendizaje por currículo (Curriculum Learning)** para modular progresivamente la dificultad de los terrenos o tareas según el desempeño del robot.

---

## Funciones Principales

### 1. `terrain_levels_vel(...) -> torch.Tensor`
- **Objetivo:** Incrementar o reducir la complejidad del terreno según la distancia recorrida por el robot frente al comando de velocidad solicitado.
- **Lógica de Decisión:**
  1. Calcula la distancia neta recorrida por cada robot respecto a su origen de spawn:
     $$d = \|\mathbf{p}_{xy}^{\text{robot}} - \mathbf{p}_{xy}^{\text{origen}}\|$$
  2. **Promoción de Nivel (`move_up`):** Si el robot recorre más de la mitad del tamaño de la celda de terreno, avanza a un terreno más accidentado (mayor rugosidad, escalones o pendientes).
  3. **Democión de Nivel (`move_down`):** Si el robot no logra avanzar al menos la mitad de la distancia esperada según el comando de velocidad:
     $$d < \|\mathbf{v}_{xy}^{\text{cmd}}\| \cdot t_{\text{episodio}} \times 0.5$$
     retrocede a un nivel de terreno más sencillo.
  4. Actualiza los orígenes de entorno en `TerrainImporter` y retorna el nivel promedio de dificultad de todos los robots.
