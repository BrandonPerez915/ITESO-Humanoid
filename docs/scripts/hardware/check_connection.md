# check_connection.py (Control)

## Descripción General

El script `check_connection.py` es una utilidad de diagnóstico rápido para verificar el estado de la comunicación en los buses CAN de las extremidades inferiores del robot Berkeley Humanoid Lite.

---

## Funcionamiento

1. Inicializa la conexión con los buses de las piernas (`can0` y `can1`) instanciando la clase `Humanoid`.
2. Ejecuta un ciclo continuo enviando solicitudes de telemetría a cada ID de actuador configurado.
3. Despliega en consola el estado de conexión de cada articulación (OK, pérdida de paquetes o timeout).
4. Permite detectar rápidamente cables CAN desconectados, problemas de terminación de 120 ohmios en la línea diferencial o actuadores sin alimentación eléctrica.
