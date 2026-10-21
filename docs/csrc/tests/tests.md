# Pruebas Unitarias de C++ (control/tests)

## Descripción General

La carpeta `control/tests/` contiene una suite de pruebas unitarias en C++ destinadas a verificar el funcionamiento aislado de cada componente crítico del sistema de control en tiempo real antes de integrarlo en el lazo cerrado principal (`RealHumanoid`).

---

## Módulos de Prueba

### 1. `test-imu.cpp`
- **Propósito:** Evaluar la inicialización, velocidad de transmisión y precisión del driver C++ de la IMU.
- **Validación:** Abre la interfaz `/dev/ttyUSB0` a 460800 baudios, comprueba la recepción de tramas y muestra en consola el cuaternión normalizado y la velocidad angular medida al mover la IMU manualmente.

---

### 2. `test-torch.cpp`
- **Propósito:** Validar la inferencia en C++ utilizando la biblioteca **LibTorch (C++ API de PyTorch)**.
- **Validación:** Carga un modelo TorchScript `.pt` exportado, alimenta un tensor de observaciones ficticio en CPU o GPU y mide el tiempo de ejecución de la inferencia para asegurar que cumpla con los requerimientos de tiempo real (inferencia $< 1$ ms).

---

### 3. `test-udp.cpp`
- **Propósito:** Probar la biblioteca C de sockets UDP no bloqueantes (`udp.c` / `udp.h`).
- **Validación:** Envía y recibe arreglos de flotantes (`float[]`) en bucle local (`127.0.0.1`), verificando que no existan fugas de memoria, corrupción de bytes ni bloqueos involuntarios del hilo ejecutor.

---

### 4. `test-yaml.cpp`
- **Propósito:** Validar el parseo de archivos de configuración mediante `yaml-cpp`.
- **Validación:** Carga `calibration.yaml` y los archivos de política, extrayendo vectores de ganancias PD, offsets de articulaciones y límites de esfuerzo para comprobar que los tipos numéricos y dimensiones coincidan con `N_JOINTS`.
