# can.py (SocketCAN Interface)

## Descripción General

El módulo `can.py` proporciona la capa de abstracción para la comunicación sobre el protocolo **SocketCAN** en entornos Linux (soporta adaptadores USB-to-CAN como Candlelight, CANable, PEAK-System o puertos nativos MCP2515/CAN-FD).

---

## Componentes y Funcionamiento

### Clase `Bus`
- **Inicialización:**
  - Se vincula a una interfaz de red CAN del sistema operativo (por ejemplo `"can0"`, `"can1"`).
  - Configura el socket a nivel de enlace de datos AF_CAN (`PF_CAN`, `SOCK_RAW`, `CAN_RAW`).
  - Habilita o deshabilita el soporte para tramas de datos flexibles (CAN-FD) y tramas estándar de 8 bytes.
- **Transmisión de Mensajes (`send`):**
  - Empaqueta el identificador CAN (11 bits estándar o 29 bits extendidos), la longitud de datos (DLC) y el payload de bytes.
  - Escribe directamente en el descriptor de archivo del socket de red de Linux.
- **Recepción de Mensajes (`recv`):**
  - Lee tramas entrantes con soporte para timeout de bloqueo configurable.
  - Desempaqueta el identificador de arbitraje, los flags de error y los bytes de carga útil.
- **Gestión de Recursos:**
  - Métodos `open()` y `close()` para liberar descriptores de socket al finalizar la ejecución.
