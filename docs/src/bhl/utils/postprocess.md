# postprocess.py

## Descripción General

El script `postprocess.py` es una herramienta de posprocesamiento para modelos de simulación cinemática y dinámica generados a partir de exportadores de CAD. Su propósito es adaptar y enriquecer los archivos URDF y MJCF (MuJoCo XML) con rutas relativas de mallas correctas y una suite completa de sensores requerida para el aprendizaje por refuerzo y control en tiempo real.

---

## Funcionalidades Principales

### 1. `postprocess_urdf(name)`
- **Objetivo:** Adecuar las referencias de mallas en el archivo URDF.
- **Funcionamiento:**
  - Lee el archivo ubicado en `data/urdf/{name}.urdf`.
  - Reemplaza el esquema de paquetes de ROS (`package://`) por rutas relativas locales (`./`).
  - Reescribe el archivo modificado para permitir la carga directa sin depender de un espacio de trabajo ROS indexado.

### 2. `postprocess_mjcf(name)`
- **Objetivo:** Inyectar definiciones de sensores para MuJoCo.
- **Funcionamiento:**
  - Abre el modelo `data/mjcf/{name}.xml`.
  - Inyecta un bloque `<sensor>` completo justo antes de la etiqueta `<equality/>`.
  - Los sensores agregados abarcan:
    - **Posición Articular (`<jointpos>`):** 10 articulaciones de brazos y 12 de piernas (posiciones angulares en radianes).
    - **Velocidad Articular (`<jointvel>`):** Velocidades angulares de las 22 articulaciones (rad/s).
    - **Torques de Actuadores (`<jointactuatorfrc>`):** Monitoreo de fuerza y torque aplicado por cada motor.
    - **Sensor IMU en el Torso:**
      - `<framequat name="imu_quat" objtype="site" objname="imu" />`: Cuaternión de orientación $[w, x, y, z]$.
      - `<gyro name="imu_gyro" site="imu" />`: Velocidad angular $[\omega_x, \omega_y, \omega_z]$.
      - `<accelerometer name="imu_acc" site="imu" />`: Aceleración lineal $[a_x, a_y, a_z]$.
      - `<framepos name="frame_pos" objtype="site" objname="imu" />`: Posición cartesiana global.
      - `<framelinvel name="frame_vel" objtype="site" objname="imu" />`: Velocidad lineal en coordenadas globales.

---

## Argumentos de Línea de Comandos

| Argumento | Tipo | Valor por Defecto | Descripción |
| :--- | :--- | :--- | :--- |
| `--mode` | `str` | `"urdf"` | Modo de operación: `"urdf"` o `"mjcf"`. |
| `--name` | `str` | `"berkeley-humanoid-lite"` | Nombre base del archivo sin extensión. |

---

## Ejemplo de Ejecución

```bash
# Para posprocesar el archivo URDF
python src/utils/postprocess.py --mode urdf --name berkeley_humanoid_lite

# Para posprocesar el modelo MJCF de MuJoCo
python src/utils/postprocess.py --mode mjcf --name berkeley_humanoid_lite
```
