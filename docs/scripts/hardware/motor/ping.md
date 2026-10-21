# ping.py (Motor)

## Descripción General

El script `ping.py` envía una solicitud de eco/verificación de presencia a través del bus CAN a un actuador Recoil específico, determinando si el dispositivo está en línea y respondiendo adecuadamente.

---

## Argumentos de Línea de Comandos

- `--channel`: Interfaz de red CAN (por ejemplo `can0`, `can1`).
- `--id`: Identificador numérico del actuador en el bus CAN.

---

## Ejemplo de Ejecución

```bash
# Probar conexión con el motor ID 1 en el bus can0
python src/control/scripts/motor/ping.py --channel can0 --id 1
```
Salida en consola:
- `Motor is online` si el motor responde a la trama de arbitraje.
- `Motor is offline` en caso de timeout o ausencia de respuesta.
