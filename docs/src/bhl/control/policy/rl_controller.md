# rl_controller.py

## Descripción General

El módulo `rl_controller.py` es el motor de inferencia de redes neuronales para el control en tiempo real del robot Berkeley Humanoid Lite. Se encarga de cargar políticas entrenadas en formatos **PyTorch (`.pt`)** o **ONNX (`.onnx`)**, procesar el vector de observaciones proveniente del hardware o del simulador, calcular la rotación inversa de cuaterniones para alinear la gravedad, e interactuar con el lazo de bajo nivel vía sockets UDP o llamadas directas.

---

## Arquitectura de Clases

### 1. Clase Base `Policy(ABC)`
Interfaz abstracta para todas las implementaciones de políticas:
- **`forward(observations: np.ndarray) -> np.ndarray`:** Método abstracto que toma las observaciones del robot y devuelve las acciones articulares.

---

### 2. Clase `TorchPolicy(Policy)`
- Utiliza PyTorch (`torch.load`) para cargar modelos exportados con TorchScript o guardados como `nn.Module`.
- Pasa el modelo a modo de evaluación (`self.model.eval()`).
- Convierte tensores entre CPU/GPU y NumPy de manera transparente.

---

### 3. Clase `OnnxPolicy(Policy)`
- Utiliza el runtime optimizado **ONNX Runtime (`ort.InferenceSession`)**, garantizando inferencias de menos de 1 milisegundo en CPUs embebidas (como Intel NUC o Raspberry Pi 5).
- Detecta automáticamente los nombres de los tensores de entrada (`"obs"` o `"onnx::Gemm_0"`).

---

### 4. Clase `RlController`
Orquestador principal del ciclo de control de la política:
- **`quat_rotate_inverse(q: np.ndarray, v: np.ndarray)`:**
  Proyecta vectores 3D (por ejemplo el vector de gravedad $[0, 0, -1]$) al marco de referencia local del cuerpo del robot usando el cuaternión de orientación $[w, x, y, z]$.
- **`load_policy()`:** Lee la ruta del checkpoint desde la configuración (`cfg.checkpoint_path`) e inicializa la clase correspondiente (`TorchPolicy` u `OnnxPolicy`).
- **`update(observations: np.ndarray) -> np.ndarray`:**
  - Extrae las lecturas de IMU, articulaciones y comandos.
  - Normaliza y escala las variables según los parámetros de configuración.
  - Ejecuta la inferencia de la red.
  - Retorna las posiciones articulares objetivo listas para los controladores PD de los motores.
