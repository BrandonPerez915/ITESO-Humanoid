# serializer.c

## Descripción General

El archivo `serializer.c` implementa funciones compactas en C para serializar y deserializar arreglos numéricos y estructuras de datos en flujos de bytes binarios directos, garantizando la compatibilidad de memoria y alineación entre aplicaciones de C/C++ y librerías de Python (como NumPy a través de ctypes o wrappers C++).

---

## Funcionalidades

- **Serialización de Flotantes:** Convierte vectores de posiciones, velocidades y comandos en un bloque contiguo de memoria con orden de bytes uniforme.
- **Deserialización con Validación:** Reconstruye vectores numéricos asegurando que el tamaño del paquete coincida exactamente con las dimensiones esperadas antes de alimentar los controladores articulares.
