# export_onshape_to_mjcf.py

## Descripción General

El script `export_onshape_to_mjcf.py` gestiona la conversión de ensamblajes mecánicos desde **Onshape** directamente al formato **MJCF (MuJoCo XML)**. Al igual que el exportador de URDF, orquesta las mallas STL, las geometrías de colisión generadas mediante OpenSCAD y además añade automáticamente la especificación de sensores requerida por las políticas de control en MuJoCo.

---

## Flujo de Trabajo

1. **Lectura y Validación de Configuración:**
   - Carga `config.json` para extraer la URL del documento Onshape y el nombre del archivo de salida.
2. **Preparación de Colisionadores OpenSCAD:**
   - Duplica los archivos `.scad` desde `robot_dir/scad` a la carpeta de trabajo para sustituir piezas complejas con aproximaciones de colisión optimizadas.
3. **Exportación con `onshape-to-robot`:**
   - Lanza el proceso que descarga la masa, inercias y cinemática en formato MuJoCo.
4. **Organización de Mallas:**
   - Centraliza las mallas STL generadas en la carpeta de mallas compartida `robot_dir/meshes`.
   - Limpia los directorios temporales de generación.
5. **Corrección de Rutas de Mallas:**
   - Ajusta las etiquetas de compilador `<compiler meshdir="...">` o rutas de mallas hacia `../meshes/`.
6. **Inyección Automática de Sensores:**
   - Inserta una suite de sensores bajo la etiqueta `<sensor>`:
     - Encoders de posición para 22 articulaciones (`jointpos`).
     - Tacómetros de velocidad para 22 articulaciones (`jointvel`).
     - Sensores de torque de actuador (`jointactuatorfrc`).
     - Sensores para la unidad IMU (`framequat`, `gyro`, `accelerometer`, `framepos`, `framelinvel`).

---

## Argumentos de Línea de Comandos

| Argumento | Tipo | Valor por Defecto | Descripción |
| :--- | :--- | :--- | :--- |
| `--config` | `str` | `./data/robots/berkeley_humanoid/berkeley_humanoid_lite/mjcf/config.json` | Ruta al archivo `config.json` del exportador MJCF. |

---

## Ejemplo de Uso

```bash
python src/utils/export_onshape_to_mjcf.py --config src/assets/mjcf/config.json
```
