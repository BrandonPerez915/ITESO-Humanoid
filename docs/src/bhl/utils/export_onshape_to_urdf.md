# export_onshape_to_urdf.py

## Descripción General

El script `export_onshape_to_urdf.py` automatiza la exportación de la geometría CAD del robot desde la plataforma en la nube **Onshape** hacia un archivo **URDF** estandarizado utilizando la herramienta `onshape-to-robot`. Además, gestiona los colisionadores geométricos personalizados modelados en OpenSCAD y consolida las mallas STL resultantes.

---

## Flujo de Ejecución y Arquitectura

1. **Lectura de Configuración:**
   - Carga el archivo `config.json` especificado por `--config`.
   - Extrae el nombre de salida del robot (`output_filename`), el directorio de destino URDF y el directorio de assets padre.
2. **Inyección de Geometría OpenSCAD (`scad/`):**
   - Si existe una carpeta `scad/` con geometrías de colisión analíticas (cilindros, esferas, pies planos, suelas de zapato, extrusiones de aluminio), las copia temporalmente a una subcarpeta `assets/` dentro del directorio URDF.
   - Esto permite que `onshape-to-robot` reemplace mallas visuales pesadas con formas de colisión simplificadas.
3. **Invocación de `onshape-to-robot`:**
   - Ejecuta por shell: `onshape-to-robot {urdf_dir}`.
   - Esta utilidad descarga los ensamblajes de Onshape a través de su API REST, calcula centros de masa, tensores de inercia, cinemática de juntas y exporta las mallas STL.
4. **Consolidación de Mallas:**
   - Mueve las mallas generadas en `assets/merged/` hacia el directorio compartido `meshes/`.
   - Limpia y elimina la carpeta temporal `assets/`.
5. **Reescritura de Enlaces de Malla:**
   - Modifica el archivo URDF generado para actualizar las rutas relativas de las mallas visuales y de colisión:
     - De `assets/merged/` a `../meshes/`.

---

## Argumentos de Línea de Comandos

| Argumento | Tipo | Valor por Defecto | Descripción |
| :--- | :--- | :--- | :--- |
| `--config` | `str` | `./data/robots/berkeley_humanoid/berkeley_humanoid_lite/urdf/config.json` | Ruta al archivo JSON con las credenciales y URLs del ensamblaje de Onshape. |

---

## Ejemplo de Uso

```bash
python src/utils/export_onshape_to_urdf.py --config src/assets/urdf/config.json
```
