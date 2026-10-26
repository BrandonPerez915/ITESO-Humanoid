# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

"""
ROS 2 Controller Module for Berkeley Humanoid Lite.

Subscribes to:
  - /cmd_vel (geometry_msgs/msg/Twist): Linear and angular velocity commands.
  - /robot_mode (std_msgs/msg/Int32, optional): Control modes (1: IDLE, 2: INIT, 3: RL_RUNNING).
"""

from __future__ import annotations

import time
import threading
from typing import Dict, Optional, Any

# Check if native rclpy is available
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Int32
    RCLPY_AVAILABLE = True
except (ImportError, ModuleNotFoundError):
    RCLPY_AVAILABLE = False
    class Node:  # type: ignore
        def __init__(self, *args: Any, **kwargs: Any) -> None: pass
        def create_subscription(self, *args: Any, **kwargs: Any) -> Any: pass
    class Twist:  # type: ignore
        def __init__(self) -> None:
            self.linear = type("Vector3", (), {"x": 0.0, "y": 0.0, "z": 0.0})()
            self.angular = type("Vector3", (), {"x": 0.0, "y": 0.0, "z": 0.0})()
    class Int32:  # type: ignore
        def __init__(self) -> None:
            self.data: int = 0
    class QoSProfile:  # type: ignore
        def __init__(self, *args: Any, **kwargs: Any) -> None: pass
    class ReliabilityPolicy:  # type: ignore
        BEST_EFFORT = 1
    class HistoryPolicy:  # type: ignore
        KEEP_LAST = 1

# Check if roslibpy is available (WebSocket bridge fallback, e.g. for macOS with Docker/Remote ROS 2)
try:
    import roslibpy
    ROSLIBPY_AVAILABLE = True
except ImportError:
    ROSLIBPY_AVAILABLE = False


class Se2Ros2:
    """
    ROS 2 / ROS bridge controller for Berkeley Humanoid Lite.
    Provides standard commands dict: velocity_x, velocity_y, velocity_yaw, mode_switch.
    """
    def __init__(self,
                 node_name: str = "bhl_sim_bridge",
                 cmd_vel_topic: str = "/cmd_vel",
                 mode_topic: str = "/robot_mode",
                 watchdog_timeout: float = 0.5,
                 bridge_host: Optional[str] = None,
                 bridge_port: int = 9090) -> None:
        self.node_name = node_name
        self.cmd_vel_topic = cmd_vel_topic
        self.mode_topic = mode_topic
        self.watchdog_timeout = watchdog_timeout
        self.bridge_host = bridge_host
        self.bridge_port = bridge_port

        self.commands: Dict[str, float] = {
            "velocity_x": 0.0,
            "velocity_y": 0.0,
            "velocity_yaw": 0.0,
            "mode_switch": 3,  # RL running mode by default
        }

        self._last_cmd_time = 0.0
        self._stopped = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._watchdog_thread: Optional[threading.Thread] = None

        self.backend = None
        self._node = None
        self._client = None

        if RCLPY_AVAILABLE and bridge_host is None:
            self.backend = "rclpy"
        elif ROSLIBPY_AVAILABLE and bridge_host is not None:
            self.backend = "roslibpy"
        elif RCLPY_AVAILABLE:
            self.backend = "rclpy"
        elif ROSLIBPY_AVAILABLE:
            self.backend = "roslibpy"
        else:
            self.backend = "none"

    def run(self) -> None:
        if self.backend == "rclpy":
            self._start_rclpy()
        elif self.backend == "roslibpy":
            self._start_roslibpy()
        else:
            print("\n" + "!" * 62)
            print("[ERROR] Ni 'rclpy' (ROS 2 nativo) ni 'roslibpy' están instalados en este entorno.")
            print("Para habilitar ROS 2:")
            print("  - Si usas ROS 2 en Linux o macOS con RoboStack: activa el entorno con ROS 2.")
            print("  - Si usas ROS 2 en Docker o en otra máquina: instala roslibpy (`pip install roslibpy`)")
            print("    e inicia rosbridge (`ros2 launch rosbridge_server rosbridge_websocket_launch.xml`).")
            print("!" * 62 + "\n")
            return

        # Iniciar watchdog de seguridad en segundo plano
        self._watchdog_thread = threading.Thread(target=self._watchdog_loop, daemon=True)
        self._watchdog_thread.start()

    def _start_rclpy(self) -> None:
        print("\n" + "=" * 62)
        print("          BERKELEY HUMANOID LITE - CONTROL VÍA ROS 2")
        print("=" * 62)
        print(f"  Tópico de velocidad  : {self.cmd_vel_topic} [geometry_msgs/msg/Twist]")
        print(f"  Tópico de modo       : {self.mode_topic} [std_msgs/msg/Int32]")
        print(f"  Watchdog timeout     : {self.watchdog_timeout} s (frena si se pierde la señal)")
        print("=" * 62)
        print("  Comandos de prueba desde otra terminal:")
        print("    ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.3}}' -r 10")
        print("    ros2 run teleop_twist_keyboard teleop_twist_keyboard\n")

        if not rclpy.ok():
            rclpy.init()

        outer = self

        class BhlNode(Node):
            def __init__(self):
                super().__init__(outer.node_name)
                qos = QoSProfile(
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=10
                )
                self.sub_vel = self.create_subscription(
                    Twist,
                    outer.cmd_vel_topic,
                    self._on_cmd_vel,
                    qos
                )
                self.sub_mode = self.create_subscription(
                    Int32,
                    outer.mode_topic,
                    self._on_mode,
                    10
                )

            def _on_cmd_vel(self, msg: Twist):
                outer.commands["velocity_x"] = float(msg.linear.x)
                outer.commands["velocity_y"] = float(msg.linear.y)
                outer.commands["velocity_yaw"] = float(msg.angular.z)
                outer._last_cmd_time = time.time()
                print(f"\r[ROS2 /cmd_vel] Vx: {msg.linear.x:+.2f} | Vy: {msg.linear.y:+.2f} | Vyaw: {msg.angular.z:+.2f}    ", end="", flush=True)

            def _on_mode(self, msg: Int32):
                outer.commands["mode_switch"] = int(msg.data)
                print(f"\n[ROS2 /robot_mode] Modo actualizado a: {msg.data}")

        self._node = BhlNode()
        self._thread = threading.Thread(target=lambda: rclpy.spin(self._node), daemon=True)
        self._thread.start()

    def _start_roslibpy(self) -> None:
        host = self.bridge_host or "localhost"
        print(f"[INFO] Conectando a ROS 2 vía rosbridge WebSocket: ws://{host}:{self.bridge_port}")
        self._client = roslibpy.Ros(host=host, port=self.bridge_port)
        try:
            self._client.run()
            print("[INFO] Conexión con rosbridge establecida.")
        except Exception as e:
            print(f"[ERROR] No se pudo conectar a rosbridge: {e}")
            return

        listener = roslibpy.Topic(self._client, self.cmd_vel_topic, 'geometry_msgs/Twist')

        def on_msg(message):
            lin = message.get('linear', {})
            ang = message.get('angular', {})
            self.commands['velocity_x'] = float(lin.get('x', 0.0))
            self.commands['velocity_y'] = float(lin.get('y', 0.0))
            self.commands['velocity_yaw'] = float(ang.get('z', 0.0))
            self._last_cmd_time = time.time()
            print(f"\r[ROS2 /cmd_vel] Vx: {self.commands['velocity_x']:+.2f} | Vy: {self.commands['velocity_y']:+.2f} | Vyaw: {self.commands['velocity_yaw']:+.2f}    ", end="", flush=True)

        listener.subscribe(on_msg)

    def _watchdog_loop(self) -> None:
        """Detiene el robot si deja de recibir mensajes por más tiempo que watchdog_timeout."""
        while not self._stopped.is_set():
            time.sleep(0.05)
            if self._last_cmd_time > 0 and (time.time() - self._last_cmd_time > self.watchdog_timeout):
                if (abs(self.commands["velocity_x"]) > 1e-4 or
                    abs(self.commands["velocity_y"]) > 1e-4 or
                    abs(self.commands["velocity_yaw"]) > 1e-4):
                    self.commands["velocity_x"] = 0.0
                    self.commands["velocity_y"] = 0.0
                    self.commands["velocity_yaw"] = 0.0
                    print("\r[ROS2 Watchdog] Sin comandos recientes en /cmd_vel -> Robot detenido.       ", end="", flush=True)

    def stop(self) -> None:
        self._stopped.set()
        if self._node and RCLPY_AVAILABLE:
            try:
                self._node.destroy_node()
                rclpy.shutdown()
            except Exception:
                pass
        if self._client and ROSLIBPY_AVAILABLE:
            try:
                self._client.terminate()
            except Exception:
                pass


if __name__ == "__main__":
    controller = Se2Ros2()
    controller.run()
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        controller.stop()
