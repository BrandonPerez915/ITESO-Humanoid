# read_configurations.py (Motor)

## Descripción General

El script `read_configurations.py` en `control/scripts/motor/` realiza un volcado completo (dump) e inspección de todos los registros de configuración y parámetros de estado almacenados en un actuador Recoil individual.

---

## Parámetros Inspeccionados

- ID de dispositivo y versión de firmware.
- Frecuencia de actualización de tramas PDO.
- Relación de reducción mecánica programada.
- Constantes de control PD de posición y velocidad.
- Constantes de corriente FOC ($K_p$, $K_i$).
- Límites de posición angular (mínimo y máximo).
- Límite de torque, voltaje de bus y temperatura límite.

---

## Ejemplo de Uso

```bash
python src/control/scripts/motor/read_configurations.py --channel can0 --id 1
```
