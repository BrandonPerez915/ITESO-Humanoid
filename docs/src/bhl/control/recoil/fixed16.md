# fixed16.py

## Descripción General

El módulo `fixed16.py` implementa funciones de conversión entre números de punto flotante (`float`) y representación en **punto fijo de 16 bits (`int16` / `uint16`)**. Este formato es esencial para comprimir múltiples valores analógicos (ángulos de posición, velocidades angulares y torques) dentro de los 8 bytes limitados de una trama estándar de CAN clásica, minimizando la latencia del bus.

---

## Funciones Principales

### 1. `float_to_fixed16(value: float, min_val: float, max_val: float) -> int`
- Escala linealmente un valor continuo dentro del rango $[min\_val, max\_val]$ a un entero de 16 bits con signo o sin signo:
  $$\text{norm} = \text{clamp}\left(\frac{\text{value} - min\_val}{max\_val - min\_val}, 0.0, 1.0\right)$$
  $$\text{int16\_val} = \text{round}(\text{norm} \times 65535)$$

---

### 2. `fixed16_to_float(int_val: int, min_val: float, max_val: float) -> float`
- Operación inversa: toma los 2 bytes desempaquetados de una trama CAN y los restituye a unidades del Sistema Internacional (radianes, rad/s o $N\cdot m$):
  $$\text{float\_val} = min\_val + \left(\frac{\text{int\_val}}{65535}\right) \times (max\_val - min\_val)$$
