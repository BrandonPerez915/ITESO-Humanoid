# test_inference.py

## Descripción General

El script `test_inference.py` es una utilidad de benchmarking y verificación de inferencia para los modelos de redes neuronales de control (`.onnx` y `.pt`).

---

## Funcionamiento

1. Carga la política especificada en el archivo de configuración YAML.
2. Genera vectores sintéticos de observación con valores representativos de posición, velocidad y comandos.
3. Ejecuta cientos de iteraciones de inferencia consecutivas midiendo los tiempos con contadores de alta resolución (`time.perf_counter()`).
4. Imprime estadísticas clave:
   - Tiempo medio de inferencia (milisegundos).
   - Percentiles 95 y 99 de latencia.
   - Rendimiento máximo en frames por segundo (FPS).
   - Valida que la latencia esté holgadamente por debajo del periodo de muestreo requerido por la política (ej. $< 2$ ms para control a 50 Hz).
