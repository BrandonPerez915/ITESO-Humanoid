# test_imu.py

## Descripción General

El script `test_imu.py` comprueba el funcionamiento, la tasa de refresco y la precisión de la Unidad de Medición Inercial (IMU) montada en el chasis del robot.

---

## Funcionamiento

1. Abre el puerto serie UART del sensor inercial usando `SerialImu(baudrate=Baudrate.BAUD_460800)`.
2. Lanza el hilo de adquisición ininterrumpida.
3. Imprime periódicamente en pantalla:
   - Cuaternión de orientación $[w, x, y, z]$.
   - Ángulos de Euler convertidos (Roll, Pitch, Yaw) en grados para fácil interpretación humana.
   - Velocidades angulares del giroscopio en rad/s.
   - Frecuencia efectiva de recepción de paquetes.
4. Permite comprobar que el marco de coordenadas del sensor coincida con el sistema de coordenadas del robot (+X hacia adelante, +Y hacia la izquierda, +Z hacia arriba).
