# test_gripper.py

## Descripción General

El script `test_gripper.py` es una utilidad para probar y calibrar pinzas o manos robóticas conectadas mediante un puerto serie UART (`/dev/ttyUSB0`) a una tasa de 115200 baudios.

---

## Protocolo de Comunicación

El script empaqueta datos binarios utilizando el módulo `struct` de Python con formato little-endian `<ffb`:
- **Campo 1 (`float` 32-bit):** Posición o fuerza deseada para el dedo o canal 1.
- **Campo 2 (`float` 32-bit):** Posición o fuerza deseada para el dedo o canal 2.
- **Campo 3 (`byte` 8-bit):** Código de comando o máscara de control (`0x0C`).

### Valores de Referencia Típicos:
- `0.25`: Pinza completamente abierta.
- `0.85`: Pinza cerrada (contacto suave con objetos).
- `0.90`: Pinza fuertemente cerrada (agarre firme).

---

## Bucle de Control

1. Abre el puerto serie `/dev/ttyUSB0` a 115200 baudios.
2. Empaqueta el mensaje con las posiciones deseadas.
3. Envía los bytes mediante `ser.write(data)`.
4. Lee e imprime la respuesta de telemetría de la pinza devuelta por microcontrolador a bordo (`ser.readline()`).

---

## Ejemplo de Ejecución

```bash
# Asignar permisos si es necesario y ejecutar
sudo chmod 666 /dev/ttyUSB0
python src/scripts/teleop/test_gripper.py
```
