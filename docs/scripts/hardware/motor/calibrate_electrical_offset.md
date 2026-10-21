# calibrate_electrical_offset.py

## Descripción General

El script `calibrate_electrical_offset.py` inicia la secuencia de calibración eléctrica interna del actuador Recoil. Esta rutina alinea el cero del encoder magnético con las fases eléctricas del estator del motor sin escobillas (BLDC/PMSM) y detecta la secuencia de fases y pares de polos.

---

## Proceso de Calibración

1. Conecta con el canal SocketCAN especificado (`channel`) a 1 Mbps.
2. Envía el comando para colocar el actuador en modo de calibración:
   `bus.set_mode(device_id, recoil.Mode.CALIBRATION)`
3. Durante esta secuencia, el driver del motor excita secuencialmente las fases U, V, W generando pequeñas rotaciones y vibraciones para medir inductancias y resistencia de fase, calculando el offset eléctrico.
4. Espera 20 segundos para que la rutina interna del firmware del motor concluya y almacene los resultados en la memoria flash interna.
5. Cierra la conexión de bus.

---

## Ejemplo de Ejecución

```bash
python src/control/scripts/motor/calibrate_electrical_offset.py --channel can0 --id 1
```
*Nota: Asegurarse de que el motor tenga libertad de giro mecánico sin obstáculos antes de iniciar.*
