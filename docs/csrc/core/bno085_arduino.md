# bno085_arduino.ino

## Descripción General

El archivo `bno085_arduino.ino` contiene el firmware para un microcontrolador (Arduino / Teensy / ESP32 / RP2040) encargado de muestrear el sensor inercial **BNO085** a través de I2C o SPI, extraer los reportes de fusión de rotación (cuaterniones a 400 Hz) y transmitirlos mediante UART USB al ordenador principal del robot.

---

## Flujo del Firmware

1. **Configuración Inicial (`setup`):**
   - Inicializa el puerto USB Serial a alta velocidad (460800 o 921600 baudios).
   - Inicializa el bus I2C/SPI para comunicarse con el sensor BNO085 (usando la biblioteca SH2 de Hillcrest Laboratories / Bosch Sensortec).
   - Configura el sensor para reportar:
     - Vector de rotación del juego (Game Rotation Vector): Cuaternión sin magnetómetro para evitar perturbaciones debidas a las corrientes de los motores.
     - Giroscopio calibrado.
2. **Bucle Principal (`loop`):**
   - Comprueba interrupciones o disponibilidad de datos del sensor (`bno08x.wasReset()`, `bno08x.getSensorEvent(...)`).
   - Empaqueta los datos en una estructura binaria con cabecera de sincronización:
     - Cabecera fija (ej. `0xAA, 0x55`).
     - Cuaternión: $w, x, y, z$ (float32).
     - Velocidades angulares: $\omega_x, \omega_y, \omega_z$ (float32).
     - Checksum o byte final.
   - Envía el paquete por UART mediante `Serial.write()`.
