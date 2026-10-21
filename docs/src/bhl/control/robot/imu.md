# imu.py

## Descripción General

El módulo `imu.py` gestiona la comunicación serie con la Unidad de Medición Inercial (IMU) montada en el torso del robot (sensor BNO085 o similar con filtro de fusión de sensores en chip). Adquiere a alta frecuencia el cuaternión de orientación espacial y las velocidades angulares del giroscopio, proporcionando los datos de balance requeridos por las políticas de locomoción.

---

## Componentes y Arquitectura

### 1. `Baudrate` (Enumeración)
Soporta velocidades de transmisión UART estándar y de alta velocidad:
- `BAUD_115200`
- `BAUD_460800` (predeterminada para transmisión sin latencia a 200-500 Hz)
- `BAUD_921600`

---

### 2. Clase `SerialImu`
- **Inicialización:**
  - Abre el puerto serie correspondiente (por ejemplo `/dev/ttyUSB0` o `/dev/ttyACM0`).
  - Configura el búfer de recepción circular y sincronización de tramas mediante cabeceras predefinidas (`preamble` o encabezados de paquete binario).
- **Procesamiento de Tramas:**
  - Parsea el paquete binario entrante que contiene:
    - Cuaternión de orientación fusionada: $[w, x, y, z]$.
    - Velocidades angulares del giroscopio calibrado: $[\omega_x, \omega_y, \omega_z]$ (rad/s).
    - Aceleraciones lineales del acelerómetro: $[a_x, a_y, a_z]$ (m/s²).
- **Ejecución Asíncrona (`run_forever`):**
  - Lanza un hilo demonio que lee de forma ininterrumpida el puerto serie.
  - Almacena las lecturas más recientes en variables atómicas protegidas para que el bucle de control principal pueda acceder a la orientación más fresca con latencia sub-milisegundo.
- **Métodos de Acceso:**
  - `get_quaternion() -> np.ndarray`: Devuelve $[w, x, y, z]$.
  - `get_angular_velocity() -> np.ndarray`: Devuelve $[\omega_x, \omega_y, \omega_z]$.
