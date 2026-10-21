# main.cpp

## Descripción General

El archivo `main.cpp` es el punto de entrada ejecutable del programa de control en tiempo real en C++ para el robot físico Berkeley Humanoid Lite. Inicializa el objeto principal `RealHumanoid`, registra el manejador de señales de parada segura del sistema operativo y lanza el lazo de control determinista a alta frecuencia.

---

## Estructura del Código

1. **Instancia Global del Robot:**
   - Declara la variable global `RealHumanoid humanoid;`.
2. **Manejo de Señales (`handle_keyboard_interrupt`):**
   - Captura la señal `SIGINT` (generada al pulsar `Ctrl+C`).
   - Invoca el método `humanoid.stop()`, el cual garantiza que todos los actuadores se apaguen de manera segura (deshabilitando los puentes H o pasando a modo damping) antes de terminar el proceso, previniendo caídas bruscas o daños mecánicos.
3. **Función `main()`:**
   - Registra el manejador con `signal(SIGINT, handle_keyboard_interrupt);`.
   - Inicia la ejecución del robot con `humanoid.run();`.
