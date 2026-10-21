# imu.cpp

## Descripción General

El archivo `imu.cpp` implementa la clase `Imu` en C++ para la comunicación serie de baja latencia con el sensor inercial del robot físico. Configura el puerto serie UART en Linux mediante la API `termios` y procesa los paquetes binarios para suministrar la orientación y velocidad angular en tiempo real al bucle de control.

---

## Funcionalidades Principales

- **Configuración de Puerto Serie (`termios`):**
  - Establece el modo raw (`cfmakeraw`), deshabilita control de flujo por hardware/software y configura la velocidad de transmisión (460800 o 921600 baudios).
  - Configura lecturas no bloqueantes o con timeout mínimo para no retrasar el lazo a 1000 Hz.
- **Sincronización y Validación de Tramas:**
  - Busca los bytes de sincronización de cabecera en el flujo serie.
  - Verifica la integridad del paquete mediante checksum o CRC.
- **Extracción de Variables Cinemáticas:**
  - Convierte los campos binarios en flotantes de precisión simple:
    - Cuaternión de orientación $[w, x, y, z]$.
    - Velocidades angulares tridimensionales $[\omega_x, \omega_y, \omega_z]$.
    - Aceleraciones lineales $[a_x, a_y, a_z]$.
