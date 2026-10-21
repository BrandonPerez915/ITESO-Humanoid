# can_transports.sh (Scripts de Gestión SocketCAN)

## Descripción General

Los scripts `start_can_transports.sh` y `stop_can_transports.sh` gestionan el ciclo de vida y configuración de las interfaces de red SocketCAN en el sistema operativo Linux (Ubuntu / Debian).

---

## 1. `start_can_transports.sh`

### Propósito
Configura y levanta las interfaces de bus CAN (`can0`, `can1`, `can2`, `can3`) requeridas para comunicarse con los actuadores Recoil del robot.

### Comandos y Parámetros Típicos:
```bash
# Configuración de bitrate a 1 Mbps y longitud de cola de transmisión
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 txqueuelen 1000
sudo ip link set can0 up

sudo ip link set can1 type can bitrate 1000000
sudo ip link set can1 txqueuelen 1000
sudo ip link set can1 up
```
- **Bitrate:** 1,000,000 bps (1 Mbps) para alta velocidad de transmisión en bucle cerrado.
- **`txqueuelen`:** Longitud de la cola de transmisión ampliada a 1000 paquetes para prevenir la pérdida de tramas en ráfagas de control.

---

## 2. `stop_can_transports.sh`

### Propósito
Desactiva y baja limpiamente las interfaces de red CAN al concluir las sesiones de prueba o antes de apagar el equipo:
```bash
sudo ip link set can0 down
sudo ip link set can1 down
```
Esto libera los buffers en el kernel de Linux y previene estados residuales de bus-off o errores de desbordamiento de cola.
