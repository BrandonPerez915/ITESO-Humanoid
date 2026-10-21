# run_idle.py (Control)

## Descripción General

El script `run_idle.py` inicializa el robot completo en un modo pasivo o de amortiguamiento (`damping`). Es la herramienta de seguridad recomendada tras encender el robot o antes de iniciar pruebas de locomoción dinámicas, permitiendo verificar que todos los actuadores respondan y que la telemetría fluya sin aplicar torques motores que puedan comprometer la integridad mecánica.

---

## Funcionamiento

1. Instancia el objeto `Humanoid()`.
2. Conecta con las interfaces SocketCAN y la unidad IMU.
3. Envía comandos de modo amortiguamiento a todos los actuadores.
4. Mantiene un lazo periódico con `RateLimiter` imprimiendo en pantalla las posiciones actuales y temperaturas para supervisión del operador.
5. Permite mover manualmente las piernas del robot con suavidad mientras se observan las lecturas de los encoders.
