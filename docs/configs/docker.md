# docker-compose.yaml y Dockerfile

## Descripción General

El directorio `src/configs/docker/` contiene las recetas de contenedorización para ejecutar **Isaac Sim**, **Isaac Lab** y el entorno de entrenamiento de Berkeley Humanoid Lite en entornos aislados con aceleración por GPU NVIDIA.

---

## Componentes

### 1. `Dockerfile`
- Basado en la imagen oficial de NVIDIA Isaac Lab (`ISAACLAB_BASE_IMAGE`).
- Instala dependencias de compilación para C/C++, CMake, SocketCAN, YAML-CPP, Pybind11 y las bibliotecas de aprendizaje por refuerzo (`rsl-rl`, `onnxruntime`, `loop_rate_limiters`, `pink`, `pinocchio`, `meshcat`).

### 2. `docker-compose.yaml`
- **Integración con GPUs NVIDIA:**
  - Habilita el runtime de NVIDIA con acceso a todas las GPUs del host (`capabilities: [gpu]`).
- **Modo de Red Host (`network_mode: host`):**
  - Permite la comunicación en tiempo real sobre sockets UDP y transporte SocketCAN sin sobrecargas de enrutamiento ni aislamiento de puertos de Docker.
- **Volúmenes Montados:**
  - Vincula el código fuente del proyecto (`source: ../`) en el contenedor para permitir edición y desarrollo interactivo con cambios en vivo.

---

### 3. `Dockerfile.ros2_humble` y `docker-compose.ros2.yaml`
- **Entorno ROS 2 Humble:**
  - Basado en `osrf/ros:humble-desktop` con `ros-humble-rosbridge-server` preinstalado.
  - Expone el puerto `9090` (WebSocket) para comunicar clientes externos (macOS con `roslibpy`, Foxglove Studio o visualizadores web).
  - Incluye PyTorch CPU y ONNX Runtime para ejecutar el nodo de política `ros2_policy_node.py` dentro del contenedor.
