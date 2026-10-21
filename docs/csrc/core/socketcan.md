# socketcan.cpp

## Descripción General

El archivo `socketcan.cpp` proporciona un wrapper en C++ de alto rendimiento sobre la API nativa de **SocketCAN** del kernel de Linux. Permite abrir canales CAN (`can0`, `can1`, `can2`, etc.), configurar sockets no bloqueantes o con timeout, y transmitir y recibir tramas con mínima sobrecarga de CPU.

---

## Métodos Clave

- **`SocketCAN::open(const std::string& interface_name)`:**
  - Crea un socket raw de tipo `PF_CAN`.
  - Resuelve el índice de la interfaz de red usando `ioctl(sock, SIOCGIFINDEX, &ifr)`.
  - Enlaza el socket a la interfaz mediante `bind(...)`.
- **`SocketCAN::send(const struct can_frame& frame)`:**
  - Realiza una llamada al sistema `write()` pasando el descriptor del socket y la estructura `can_frame`.
- **`SocketCAN::receive(struct can_frame& frame)`:**
  - Utiliza `read()` para obtener la siguiente trama disponible en el búfer del kernel.
- **`SocketCAN::close()`:**
  - Cierra de forma segura el descriptor de archivo del socket.
