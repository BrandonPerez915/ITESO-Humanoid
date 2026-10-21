# Modelos y Assets del Robot (src/assets)

## Descripción General

El directorio `src/assets/` alberga las representaciones cinemáticas, geométricas y dinámicas del Berkeley Humanoid Lite en múltiples formatos de simulación y robótica: **MJCF (MuJoCo)**, **URDF (ROS)**, **USD (NVIDIA Isaac Sim)**, **SCAD (OpenSCAD)** y **Meshes (STL)**.

---

## Subdirectorios y Formatos

### 1. `mjcf/` (MuJoCo XML Format)
- **`berkeley_humanoid_lite.xml`:** Definición del robot humanoide completo de 22 grados de libertad.
- **`berkeley_humanoid_lite_biped.xml`:** Modelo de las extremidades inferiores (12 grados de libertad).
- **`bhl_scene.xml` / `bhl_biped_scene.xml`:** Escenas de simulación que incluyen el plano de suelo con fricción, iluminación, cámara y configuración del integrador de física (Runge-Kutta o Euler semiexplícito a paso de 0.0005 a 0.001 s).
- **`config.json`:** Configuración utilizada por `onshape-to-robot` para generar el modelo MJCF desde la nube de CAD.

---

### 2. `urdf/` (Unified Robot Description Format)
- **`berkeley_humanoid_lite.urdf`:** Archivo estándar de cinemática y dinámica utilizado por ROS, PyBullet, Pinocchio y Pink para cálculo de Jacobianos, cinemática inversa y visualización.
- **`berkeley_humanoid_lite_biped.urdf`:** Versión bípeda sin el tren superior.
- **`config.json`:** Parámetros de extracción de Onshape a URDF (enlaces, uniones continuas/revolutas, masas y tensores de inercia).

---

### 3. `usd/` (Universal Scene Description - NVIDIA Omniverse / Isaac Sim)
- **`berkeley_humanoid_lite.usd`:** Escenario USD jerárquico que describe la articulación de PhysX con sus APIs de cuerpos rígidos, límites articulares y motores implícitos.
- **`berkeley_humanoid_lite_biped.usd`:** Versión instanciable para entrenamiento paralelo masivo con 4096 robots en Isaac Lab.
- **`configuration/`:** Submódulos USD que desacoplan la definición base (`base.usd`), las colisiones físicas (`physics.usd`) y los sensores de contacto e IMU (`sensor.usd`).

---

### 4. `scad/` (Geometría Analítica OpenSCAD)
Contiene modelos paramétricos en OpenSCAD que `onshape-to-robot` utiliza para generar mallas de colisión primitivas y convexas en lugar de mallas triangulares complejas:
- `left_shoe.scad` / `right_shoe.scad`: Suelas y zapatos del robot con coeficientes de fricción y contacto optimizados.
- `flat_foot.scad`: Modelo de pie plano para pruebas iniciales de estabilidad estática.
- `geometry_stud_lowerleg.scad` / `geometry_stud_upperleg.scad`: Cilindros y cápsulas envolventes para muslos y canillas.
- `aluminum_extrusion_100mm_center.scad`: Perfil estructural del torso.

---

### 5. `meshes/` (Mallas STL)
Almacena todas las mallas geométricas de superficie exportadas en formato STL para renderizado visual en MuJoCo, Meshcat e Isaac Sim (ej. `base_visual.stl`, `arm_left_shoulder_pitch_visual.stl`, `leg_left_knee_pitch_visual.stl`).
