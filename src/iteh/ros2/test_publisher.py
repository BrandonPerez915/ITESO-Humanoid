#!/usr/bin/env python3
"""
Simple ROS 2 publisher script to test ITESO Humanoid simulation control.
Publishes Twist messages to /cmd_vel.
"""

import sys
import time

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
except ImportError:
    print("[ERROR] 'rclpy' no está instalado en este entorno.")
    print("Puedes probar con la terminal de ROS 2 directamente:")
    print("  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.3}}' -r 10")
    sys.exit(1)


class CmdVelTestPublisher(Node):
    def __init__(self):
        super().__init__('bhl_cmd_vel_tester')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        print("\n" + "=" * 55)
        print("   PUBLICADOR DE PRUEBA ROS 2 -> /cmd_vel")
        print("=" * 55)
        print("Secuencia de prueba automática:")
        print("  1. Caminar hacia adelante (Vx = 0.35 m/s) por 4s")
        print("  2. Girar a la izquierda (Vyaw = 0.4 rad/s) por 4s")
        print("  3. Detenerse (Vx = 0, Vyaw = 0)")
        print("=" * 55 + "\n")

    def send_velocity(self, vx=0.0, vy=0.0, vyaw=0.0):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(vyaw)
        self.publisher_.publish(msg)


def main():
    rclpy.init()
    node = CmdVelTestPublisher()

    try:
        # 1. Caminar hacia adelante
        print("-> [1/3] Enviando: Caminar hacia adelante (Vx = 0.35 m/s)...")
        start = time.time()
        while time.time() - start < 4.0:
            node.send_velocity(vx=0.35)
            time.sleep(0.1)

        # 2. Girar
        print("-> [2/3] Enviando: Giro hacia la izquierda (Vyaw = 0.4 rad/s)...")
        start = time.time()
        while time.time() - start < 4.0:
            node.send_velocity(vx=0.2, vyaw=0.4)
            time.sleep(0.1)

        # 3. Detener
        print("-> [3/3] Enviando: Detenerse...")
        for _ in range(5):
            node.send_velocity(vx=0.0, vy=0.0, vyaw=0.0)
            time.sleep(0.1)

        print("\n[OK] Secuencia de prueba completada exitosamente.")

    except KeyboardInterrupt:
        print("\nPrueba cancelada por el usuario.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
