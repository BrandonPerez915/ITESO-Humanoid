# udp.c

## Descripción General

El archivo `udp.c` implementa una interfaz en lenguaje C puro para comunicación mediante sockets **UDP (User Datagram Protocol)**. Permite el intercambio bidireccional y sin bloqueo de arreglos de flotantes (`float[]`) y paquetes de telemetría entre el bucle en tiempo real de C++ y procesos externos de alto nivel (como visualizadores gráficos en MuJoCo o políticas de inferencia en Python).

---

## Funciones Principales

- **`udp_init(udp_t* udp, const char* local_ip, int local_port, const char* remote_ip, int remote_port)`:**
  - Crea un socket UDP estándar (`AF_INET`, `SOCK_DGRAM`).
  - Configura el socket en modo no bloqueante (`O_NONBLOCK`) usando `fcntl()`.
  - Enlaza el socket a la dirección local y almacena los datos de la dirección destino remota.
- **`udp_send(udp_t* udp, const void* data, size_t size)`:**
  - Envía un búfer binario hacia la dirección remota preconfigurada usando `sendto()`.
- **`udp_recv(udp_t* udp, void* data, size_t max_size)`:**
  - Lee los datos disponibles en el socket sin bloquear el hilo de ejecución mediante `recvfrom()`.
- **`udp_close(udp_t* udp)`:**
  - Cierra el descriptor del socket.
