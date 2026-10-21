# terminations.py

## Descripción General

El módulo `terminations.py` implementa condiciones de parada anticipada (terminación de episodio) para el entrenamiento de aprendizaje por refuerzo. Define cuándo un robot ha fallado o se ha desviado más allá de los límites operacionales tolerables, provocando el reinicio del entorno.

---

## Funciones Principales

### 1. `terrain_out_of_bounds(...) -> torch.Tensor`
- **Propósito:** Detectar si el robot se ha desplazado fuera del mapa o demasiado cerca del borde del terreno generado procedimentalmente.
- **Funcionamiento:**
  - Para terrenos planos (`"plane"`), retorna `False` ya que el plano es infinito.
  - Para generadores de terreno en malla (`"generator"`):
    1. Calcula el ancho y largo total del mapa considerando el número de sub-terrenos y el ancho del borde perimetral (`border_width`).
    2. Compara la posición global del robot en $X$ e $Y$:
       $$|x_{\text{robot}}| > \frac{\text{ancho\_mapa}}{2} - \text{buffer} \quad \lor \quad |y_{\text{robot}}| > \frac{\text{largo\_mapa}}{2} - \text{buffer}$$
    3. Si el robot cruza este margen de seguridad (`distance_buffer`), se marca el episodio como terminado.

---

## Integración con Isaac Lab

Esta función se registra como un término de terminación (`DoneTerm` o `TerminationTermCfg`) dentro de la clase de configuración del entorno (`TerminationsCfg`). Al activarse, el administrador de terminaciones de Isaac Lab resetea únicamente los entornos que satisfacen la condición, optimizando el uso de la GPU.
