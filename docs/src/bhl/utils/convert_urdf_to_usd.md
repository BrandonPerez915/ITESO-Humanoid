# convert_urdf_to_usd.py

## Descripción General

El script `convert_urdf_to_usd.py` convierte modelos en formato **URDF (Unified Robot Description Format)** al formato **USD (Universal Scene Description)** de Pixar, utilizado como estándar nativo en **NVIDIA Isaac Sim** y **Omniverse**. Utiliza la extensión `isaacsim.asset.importer.urdf` para parsear la cinemática, las propiedades dinámicas de cuerpos rígidos, las articulaciones de PhysX y las referencias a mallas.

---

## Flujo de Trabajo

1. **Inicialización de Omniverse / Isaac Sim:**
   - Inicia el entorno `AppLauncher` de Isaac Lab en modo headless o con interfaz gráfica para acceder a los subsistemas de PhysX y USD.
2. **Configuración del Importador URDF (`UrdfConverter`):**
   - Configura opciones como la consolidación de articulaciones fijas (`merge_fixed_joints`), la rigidez (`joint_stiffness`), el amortiguamiento (`joint_damping`) y el tipo de control (`position`, `velocity` o `none`).
3. **Generación del Escenario USD:**
   - Importa las etiquetas `<link>`, `<joint>`, `<inertial>`, `<visual>`, `<collision>` y `<site>`.
   - Permite activar la propiedad `make_instanceable` para clonación ultra-eficiente en GPU con miles de robots en paralelo.
4. **Exportación y Guardado:**
   - Serializa la estructura de prims (`PhysicsScene`, `ArticulationRoot`, `RigidBodyAPI`, `DriveAPI`) a un archivo `.usd` listo para simulación y entrenamiento con Isaac Lab.

---

## Argumentos de Línea de Comandos

| Argumento | Tipo | Valor por Defecto | Descripción |
| :--- | :--- | :--- | :--- |
| `input` | `str` | *(Requerido)* | Ruta al archivo URDF de entrada. |
| `--fix-base` | `flag` | `False` | Si se activa, ancla la base del robot al mundo (para manipuladores fijos). |
| `--merge-joints` | `flag` | `False` | Fusiona eslabones conectados por articulaciones fijas para acelerar la física. |
| `--joint-stiffness` | `float` | `100.0` | Rigidez por defecto para los drives articulares ($K_p$). |
| `--joint-damping` | `float` | `1.0` | Amortiguamiento por defecto para los drives articulares ($K_d$). |
| `--joint-target-type` | `str` | `"position"` | Modo de control: `"position"`, `"velocity"` o `"none"`. |
| `--import-sites` | `flag` | `True` | Importa elementos `<site>` (utilizados para sensores de contacto e IMU). |
| `--make-instanceable` | `flag` | `True` | Convierte el asset en instanciable para entrenamiento paralelo masivo. |

---

## Ejemplo de Uso

```bash
# Convertir el robot humanoide completo a USD
python src/utils/convert_urdf_to_usd.py \
  src/assets/urdf/berkeley_humanoid_lite.urdf \
  --joint-stiffness 20.0 \
  --joint-damping 2.0 \
  --make-instanceable
```
