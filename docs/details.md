# Guía Detallada para Principiantes: Berkeley Humanoid Lite

Esta guía está diseñada para que cualquier persona, **sin conocimientos previos en robótica, simulación o inteligencia artificial**, pueda entender qué es este proyecto, cómo funciona cada una de sus partes y cómo ponerlo a funcionar en su computadora paso a paso.

---

## 1. ¿Qué es Berkeley Humanoid Lite?

El **Berkeley Humanoid Lite** es un robot humanoide de código abierto creado por el laboratorio de investigación **Hybrid Robotics** de la Universidad de California, Berkeley (UC Berkeley).

* **El Problema:** La mayoría de los robots humanoides de investigación (como Boston Dynamics Atlas, Unitree H1 o Figure) cuestan entre $30,000 y $150,000 USD, lo que impide que estudiantes y entusiastas puedan experimentar con ellos.
* **La Solución:** Berkeley Humanoid Lite fue diseñado para construirse con componentes accesibles y piezas impresas en 3D, reduciendo el costo a menos de **$5,000 USD**.
* **El Objetivo del Software:** Antes de gastar dinero en piezas físicas o arriesgarse a romper motores reales, todo el comportamiento del robot (cómo se balancea, camina y esquiva caídas) se diseña, prueba y valida en una **simulación por computadora**.

---

## 2. Conceptos Clave Explicados con Manzanas

Para no perderte en tecnicismos, aquí tienes la explicación sencilla de los términos que verás todo el tiempo:

### 2.1. Grados de Libertad (DOF - Degrees of Freedom)
* **¿Qué es?** Es la cantidad de motores o articulaciones independientes que tiene el robot.
* **En este robot:**
  * **12 DOF (Modo Bípedo):** Solo las piernas (6 motores en la pierna izquierda y 6 en la derecha: cadera, rodilla y tobillo). Es la versión ideal para aprender a caminar.
  * **22 DOF (Modo Humanoide Completo):** Las 12 articulaciones de las piernas + 10 articulaciones adicionales para la cintura, cuello, hombros, codos y brazos.

### 2.2. Simulación Física (Sim2Sim y Sim2Real)
* **¿Por qué simular?** Si un algoritmo falla en la vida real, el robot cae al suelo y rompe una pieza de $200 USD. En el simulador, simplemente reinicias el programa en un milisegundo sin consecuencias.
* **Sim2Sim (De un simulador a otro):** El robot aprende a caminar en un simulador masivo (Isaac) y luego se comprueba en otro simulador independiente (MuJoCo) para verificar que camine bien y no tenga "trampas" virtuales.
* **Sim2Real (De la simulación a la realidad):** Tomar el programa que funcionó en el simulador y cargarlo en los motores físicos del robot real.

### 2.3. Motores de Simulación: MuJoCo vs. Isaac Sim
* **MuJoCo (Multi-Joint dynamics with Contact):**
  * Es un motor de física extremadamente rápido y preciso desarrollado originalmente para biomecánica (hoy mantenido por Google DeepMind).
  * **Ventaja gigante:** Funciona de forma **nativa en cualquier computadora**, incluyendo MacBooks (Apple Silicon M1/M2/M3/M4) e Intel, utilizando solo el procesador (CPU) sin requerir tarjetas gráficas especiales.
* **NVIDIA Isaac Sim / Isaac Lab:**
  * Es un simulador creado por NVIDIA para videojuegos y robótica que usa tarjetas gráficas GeForce/RTX.
  * Su superpoder es simular **4.000 robots al mismo tiempo** para acelerar el entrenamiento de semanas a solo 2 horas. Requiere una computadora Linux con GPU NVIDIA.

### 2.4. Archivos de Anatomía del Robot: URDF y MJCF
* Un robot en la computadora no es solo un dibujo 3D; la computadora necesita saber cuánto pesa cada pierna, dónde está su centro de gravedad y qué articulación conecta con cuál.
* **URDF (.urdf):** Formato universal basado en XML que describe la jerarquía del robot (como el esqueleto y sus conexiones).
* **MJCF (.xml):** Formato optimizado específicamente para el simulador MuJoCo. Incluye información de contactos, rozamiento del suelo, amortiguadores y resortes virtuales.

### 2.5. El "Cerebro" del Robot: Redes Neuronales y Archivos ONNX
* Los robots tradicionales usaban ecuaciones matemáticas muy complejas para calcular cada paso.
* Los robots modernos usan **Aprendizaje por Refuerzo (Reinforcement Learning)**: una red neuronal ("cerebro artificial") prueba millones de movimientos aleatorios en la simulación; si avanza sin caerse recibe una "recompensa", si se cae recibe un "castigo".
* **Archivo .onnx:** Es el resultado final de ese aprendizaje. Es una red neuronal ya educada y congelada lista para usar. Ocupa unos pocos megabytes y le dice al robot cómo mover las piernas en tiempo real.

---

## 3. ¿Cómo Funciona el Sistema Completo? (La Analogía del Cuerpo Humano)

Cada paso del robot ocurre en un ciclo continuo que se repite entre 25 y 2000 veces por segundo:

```
  +-------------------------------------------------------------+
  | 1. LOS SENTIDOS (Sensores en MuJoCo)                        |
  |    - IMU (Oído interno): Mide inclinación y aceleración     |
  |    - Encoders (Propiocepción): Sabe el ángulo de cada motor |
  +------------------------------+------------------------------+
                                 |
                                 v
  +-------------------------------------------------------------+
  | 2. EL CEREBRO (Política ONNX / Red Neuronal)                |
  |    Recibe los datos de los sentidos + tu orden de velocidad |
  |    (por ejemplo: "avanza a 0.5 m/s hacia adelante")         |
  |    Calcula a qué ángulo debe moverse cada pierna.           |
  +------------------------------+------------------------------+
                                 |
                                 v
  +-------------------------------------------------------------+
  | 3. LOS MÚSCULOS (Control PD en MuJoCo)                      |
  |    Aplica fuerza (torque) a los motores para llevarlos al   |
  |    ángulo ordenado por el cerebro.                          |
  +------------------------------+------------------------------+
                                 |
                                 v
  +-------------------------------------------------------------+
  | 4. EL MUNDO (Física de MuJoCo)                              |
  |    El pie toca el suelo, la gravedad empuja hacia abajo,    |
  |    el robot avanza un paso y el ciclo vuelve a empezar.     |
  +-------------------------------------------------------------+
```

---

## 4. Guía Práctica: Tu Primera Simulación en Mac (Paso a Paso)

No necesitas instalar máquinas virtuales ni comprar hardware adicional. Todo lo siguiente corre **directamente en tu Mac**.

### Paso 1: Abrir la Terminal
1. En tu Mac, presiona `Cmd + Espacio` para abrir Spotlight.
2. Escribe `Terminal` y presiona `Enter`.

### Paso 2: Instalar herramientas base (Homebrew y Python)
Si ya tienes Homebrew y Python instalado, puedes saltar al Paso 3. Si no:
1. Instala Homebrew (el instalador de programas estándar para Mac):
   ```zsh
   /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
   ```
2. Instala `uv` (la herramienta moderna más rápida del mundo para gestionar proyectos de Python sin enredos):
   ```zsh
   curl -LsSf https://astral.sh/uv/install.sh | sh
   ```
   *(Cierra la terminal y vuelve a abrirla para que reconozca el comando `uv`)*.

### Paso 3: Descargar el Proyecto
Entra a tu carpeta de trabajo y descarga el proyecto con todos sus componentes:
```zsh
cd /Users/brandonperez/Documents/Robotica/Humanoid
git clone --recurse-submodules https://github.com/HybridRobotics/berkeley-humanoid-lite.git
cd berkeley-humanoid-lite
```

> **Nota para principiantes:** El comando `--recurse-submodules` es vital. Este proyecto no es solo una carpeta, sino que incluye dentro las carpetas de los modelos 3D (`assets`) y de los controladores (`lowlevel`). Ese parámetro descarga todo junto.

### Paso 4: Crear el Entorno Virtual e Instalar Paquetes
Un "entorno virtual" es como una caja aislada para que las librerías de este proyecto no interfieran con otros programas de tu computadora:

```zsh
# 1. Crear entorno virtual con Python 3.10 o 3.11
uv venv --python 3.11

# 2. Activar el entorno
source .venv/bin/activate

# 3. Instalar librerías esenciales para simular
uv pip install mujoco numpy torch onnxruntime pygame pyyaml

# 4. Conectar los paquetes del robot al entorno
uv pip install -e ./source/berkeley_humanoid_lite/
uv pip install -e ./source/berkeley_humanoid_lite_assets/
uv pip install -e ./source/berkeley_humanoid_lite_lowlevel/
```

### Paso 5: ¡Ver al Robot en Pantalla! (Sim2Sim MuJoCo)
El proyecto ya trae cerebros preentrenados (`.onnx`) en la carpeta `checkpoints/`. Puedes lanzar la simulación de inmediato con este comando:

```zsh
python ./scripts/sim2sim/play_mujoco.py --config ./configs/policy_biped_25hz_a.yaml
```

Se abrirá una ventana gráfica 3D interactiva de MuJoCo:
* **Clic izquierdo + arrastrar:** Rotar la cámara en 3D.
* **Clic derecho + arrastrar:** Acercar / alejar el zoom (Zoom in / Zoom out).
* **Clic rueda central + arrastrar:** Desplazar la cámara lateralmente.
* **Espacio:** Pausar o reanudar la simulación.
* **Retroceso (Backspace):** Reiniciar la posición del robot al origen.

---

## 5. ¿Cómo Controlar la Dirección del Robot?

Por defecto, el script busca un mando de videojuegos (Xbox, PlayStation o gamepad USB/Bluetooth conectado a tu Mac):
* **Palanca izquierda arriba/abajo:** Velocidad hacia adelante / atrás ($V_x$).
* **Palanca izquierda lateral:** Desplazamiento lateral ($V_y$).
* **Palanca derecha lateral:** Girar sobre su propio eje ($Yaw$).

> **¿No tienes un mando físico?**
> No te preocupes. En el archivo `todo.md` está contemplada la opción de sustituir el lector del mando por un pequeño script de teclado o introducir valores fijos de velocidad (por ejemplo, velocidad constante de 0.4 m/s hacia adelante) para observar al robot caminar sin necesidad de comprar ningún accesorio.

---

## 6. Estructura de las Carpetas del Proyecto (Tu Mapa de Navegación)

Cuando abras la carpeta en tu editor de código (como VS Code o Cursor), verás esta estructura:

```
berkeley-humanoid-lite/
├── checkpoints/       <-- Cerebros ya entrenados (archivos .onnx listos para usar)
├── configs/           <-- Configuraciones (pesos, frecuencias de control, puertos)
├── scripts/           <-- Los programas que puedes ejecutar
│   ├── sim2sim/       <-- scripts para probar en MuJoCo (aquí está play_mujoco.py)
│   ├── rsl_rl/        <-- scripts para entrenar con Isaac Lab (en servidores GPU)
│   └── teleop/        <-- scripts para mover los brazos con mandos o cinemática
├── source/            <-- El código fuente principal
│   ├── berkeley_humanoid_lite/        <-- Definición de tareas y físicas
│   ├── berkeley_humanoid_lite_assets/ <-- Modelos 3D, URDF, MJCF (esqueleto del robot)
│   └── berkeley_humanoid_lite_lowlevel/ <-- Control de motores y joystick
├── todo.md            <-- Tu lista de tareas pendientes paso a paso
└── details.md         <-- Este documento explicativo
```

---

## 7. Preguntas Frecuentes de Principiantes (FAQ)

### ¿Tengo que saber matemáticas complejas (matrices, cuaterniones) para empezar?
No. Para simular y aprender cómo interactúan los componentes, las librerías matemáticas (`numpy`, `torch`, `mujoco`) ya hacen todos los cálculos por ti. A medida que quieras diseñar tus propios movimientos o modificar el robot, podrás ir aprendiendo esos conceptos progresivamente.

### ¿Por qué existe Isaac Lab si MuJoCo ya funciona en mi Mac?
* **MuJoCo:** Es excelente para simulación en tiempo real (1 o 2 robots con gráficos realistas interactivos).
* **Isaac Lab:** Es una herramienta para **entrenamiento masivo**. Coloca 4.096 robots en una cuadrícula gigante dentro de una tarjeta NVIDIA, los hace caer millones de veces por minuto y en 2 horas obtiene un archivo `.onnx` nuevo.
* **El flujo de trabajo profesional es:** Entrenar en Isaac Lab (en la nube) -> Exportar a `.onnx` -> Probar en MuJoCo (en tu Mac) -> Instalar en el robot real.

### ¿Cuánto cuesta entrenar si no tengo tarjeta NVIDIA?
Si algún día decides crear tus propios cerebros desde cero y no conformarte con los que ya vienen en `checkpoints/`, puedes usar servicios en la nube como **RunPod** o **Vast.ai**. Alquilas una GPU potente (como una RTX 4090) por unos **$0.30 a $0.40 USD la hora**. Un entrenamiento completo dura 2 horas, costando menos de **$1.00 USD en total**.

---

## 8. Glosario Rápido de Términos

* **Actuador:** El motor combinado con su reductora que hace girar una articulación.
* **CAN bus:** Protocolo de cables por donde viajan las órdenes a los motores en el robot físico.
* **Cuaternión ($q = [w, x, y, z]$):** Forma matemática de representar hacia dónde está inclinado el cuerpo del robot en el espacio 3D sin sufrir bloqueos angulares.
* **IMU (Inertial Measurement Unit):** Sensor con giróscopo y acelerómetro (equivalente al oído interno de los humanos) que detecta caídas e inclinaciones.
* **Latencia:** El tiempo que tarda el cerebro en pensar una orden y el motor en empezar a moverse. Si la latencia es alta, el robot se cae.
* **Torque:** La fuerza de giro que ejerce cada motor (medida en Newton-metros, $N\cdot m$).
