# util.py (Recoil Utilities)

## Descripción General

El módulo `util.py` agrupa funciones auxiliares para el manejo de operaciones a nivel de bit, verificación de redundancia cíclica (CRC) y formateo de identificadores y payloads del protocolo Recoil.

---

## Funcionalidades

- **Cálculo de CRC / Checksums:** Rutinas para verificar la validez de tramas críticas recibidas de los motores y evitar comandos corruptos por ruido electromagnético en las líneas CAN.
- **Manipulación de Bits y Endianness:** Funciones de ordenamiento de bytes (conversión entre big-endian y little-endian) para coincidir con la arquitectura del microcontrolador de los drivers de los actuadores (habitualmente ARM Cortex-M4/M7).
- **Formateo y Debugging:** Herramientas para imprimir tramas CAN en hexadecimal en la consola para depuración rápida de fallas de comunicación.
