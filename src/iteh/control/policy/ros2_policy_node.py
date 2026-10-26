# Copyright (c) 2025, The Berkeley Humanoid Lite / ITESO Humanoid Project Developers.

"""
ROS 2 Policy Execution Node for ITESO Humanoid / Berkeley Humanoid Lite.

This node encapsulates the reinforcement learning control policy (PyTorch / ONNX)
inside a standard ROS 2 node lifecycle, fully decoupling policy inference from
the underlying physical hardware or simulator.

Subscriptions:
  - /cmd_vel (geometry_msgs/msg/Twist): Linear and angular velocity commands.
  - /joint_states (sensor_msgs/msg/JointState): Measured joint positions and velocities.
  - /imu/data or /imu (sensor_msgs/msg/Imu): Base orientation quaternion and angular velocity.
  - /robot_mode (std_msgs/msg/Int32, optional): Control modes (1: IDLE, 2: RL_INIT, 3: RL_RUNNING).

Publications:
  - /joint_commands (sensor_msgs/msg/JointState): Target joint positions.
  - /forward_command_controller/commands (std_msgs/msg/Float64MultiArray, optional):
      Compatible with ros2_control forward controllers.
  - ~/status (std_msgs/msg/String): Diagnostic status of the node and mode state machine.
  - ~/debug_observations (std_msgs/msg/Float32MultiArray): Observation vector fed to policy.

Services:
  - ~/reset_policy (std_srvs/srv/Trigger): Resets policy observation buffer and history.
"""

from __future__ import annotations

import time
import sys
from typing import Dict, List, Optional, Any
import numpy as np

# Verify ROS 2 dependencies
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import JointState, Imu
    from std_msgs.msg import Int32, String, Float32MultiArray, Float64MultiArray
    from std_srvs.srv import Trigger
    RCLPY_AVAILABLE = True
except (ImportError, ModuleNotFoundError):
    RCLPY_AVAILABLE = False
    class Node:  # type: ignore
        def __init__(self, *args: Any, **kwargs: Any) -> None: pass
        def create_publisher(self, *args: Any, **kwargs: Any) -> Any: pass
        def create_subscription(self, *args: Any, **kwargs: Any) -> Any: pass
        def create_service(self, *args: Any, **kwargs: Any) -> Any: pass
        def create_timer(self, *args: Any, **kwargs: Any) -> Any: pass
        def get_logger(self) -> Any: return self
        def get_clock(self) -> Any: return type("Clock", (), {"now": lambda: type("Time", (), {"to_msg": lambda: None})()})()
        def info(self, *args: Any) -> None: pass
        def warn(self, *args: Any) -> None: pass
        def error(self, *args: Any) -> None: pass

    class Twist:  # type: ignore
        def __init__(self) -> None:
            self.linear = type("Vector3", (), {"x": 0.0, "y": 0.0, "z": 0.0})()
            self.angular = type("Vector3", (), {"x": 0.0, "y": 0.0, "z": 0.0})()

    class JointState:  # type: ignore
        def __init__(self) -> None:
            self.header = type("Header", (), {"stamp": None, "frame_id": ""})()
            self.name: list[str] = []
            self.position: list[float] = []
            self.velocity: list[float] = []
            self.effort: list[float] = []

    class Imu:  # type: ignore
        def __init__(self) -> None:
            self.header = type("Header", (), {"stamp": None, "frame_id": ""})()
            self.orientation = type("Quaternion", (), {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0})()
            self.angular_velocity = type("Vector3", (), {"x": 0.0, "y": 0.0, "z": 0.0})()
            self.linear_acceleration = type("Vector3", (), {"x": 0.0, "y": 0.0, "z": 0.0})()

    class Int32:  # type: ignore
        def __init__(self) -> None: self.data: int = 0
    class String:  # type: ignore
        def __init__(self) -> None: self.data: str = ""
    class Float32MultiArray:  # type: ignore
        def __init__(self) -> None: self.data: list[float] = []
    class Float64MultiArray:  # type: ignore
        def __init__(self) -> None: self.data: list[float] = []
    class Trigger:  # type: ignore
        class Request: pass
        class Response:
            def __init__(self) -> None:
                self.success: bool = False
                self.message: str = ""
    class QoSProfile:  # type: ignore
        def __init__(self, *args: Any, **kwargs: Any) -> None: pass
    class ReliabilityPolicy:  # type: ignore
        BEST_EFFORT = 1
    class HistoryPolicy:  # type: ignore
        KEEP_LAST = 1
    qos_profile_sensor_data = None  # type: ignore

from iteh.control.policy.config import Cfg
from iteh.control.policy.rl_controller import RlController

class RobotMode:
    INVALID = 0
    IDLE = 1
    RL_INIT = 2
    RL_RUNNING = 3

class Ros2PolicyNode(Node):
    """
    ROS 2 Node for RL Policy Execution.
    """

    def __init__(self, node_name: str = "humanoid_policy_node", **kwargs) -> None:
        if not RCLPY_AVAILABLE:
            raise RuntimeError(
                "rclpy o las dependencias de mensajes de ROS 2 no están disponibles en este entorno.\n"
                "Asegúrate de haber ejecutado 'source /opt/ros/<distro>/setup.bash' o de correr dentro "
                "del contenedor Docker/entorno con ROS 2 habilitado."
            )

        super().__init__(node_name, **kwargs)
        # Declaración de parámetros con valores por defecto
        self.declare_parameter("config", "biped_25hz.yaml")
        self.declare_parameter("policy_rate", 0.0)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("mode_topic", "/robot_mode")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("joint_commands_topic", "/joint_commands")
        self.declare_parameter("forward_commands_topic", "/forward_command_controller/commands")
        self.declare_parameter("publish_joint_states", True)
        self.declare_parameter("publish_forward_commands", True)
        self.declare_parameter("publish_all_joints", True)
        self.declare_parameter("watchdog_timeout", 0.5)
        self.declare_parameter("sensor_timeout", 1.0)
        self.declare_parameter("init_duration", 2.0)
        self.declare_parameter("auto_start_after_init", True)

        # Leer valores de parámetros
        config_name = self.get_parameter("config").get_parameter_value().string_value
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        mode_topic = self.get_parameter("mode_topic").get_parameter_value().string_value
        joint_states_topic = self.get_parameter("joint_states_topic").get_parameter_value().string_value
        imu_topic = self.get_parameter("imu_topic").get_parameter_value().string_value
        self.joint_commands_topic = self.get_parameter("joint_commands_topic").get_parameter_value().string_value
        self.forward_commands_topic = self.get_parameter("forward_commands_topic").get_parameter_value().string_value
        self.publish_joint_states = self.get_parameter("publish_joint_states").get_parameter_value().bool_value
        self.publish_forward_commands = self.get_parameter("publish_forward_commands").get_parameter_value().bool_value
        self.publish_all_joints = self.get_parameter("publish_all_joints").get_parameter_value().bool_value
        self.watchdog_timeout = self.get_parameter("watchdog_timeout").get_parameter_value().double_value
        self.sensor_timeout = self.get_parameter("sensor_timeout").get_parameter_value().double_value
        self.init_duration = self.get_parameter("init_duration").get_parameter_value().double_value
        self.auto_start_after_init = self.get_parameter("auto_start_after_init").get_parameter_value().bool_value

        self.get_logger().info(f"Cargando configuración de política: {config_name}")
        self.cfg = Cfg.load(config_name, control_mode="ros2")

        self.controller = RlController(self.cfg)
        self.controller.load_policy()

        # Determinar frecuencia de ejecución
        param_rate = self.get_parameter("policy_rate").get_parameter_value().double_value
        if param_rate > 0.0:
            self.policy_rate = param_rate
        else:
            self.policy_rate = 1.0 / float(self.cfg.policy_dt)

        self.timer_period = 1.0 / self.policy_rate
        self.get_logger().info(f"Frecuencia de la política configurada a: {self.policy_rate:.1f} Hz (dt={self.timer_period:.4f}s)")

        # Mapeo de articulaciones
        self.num_joints = int(self.cfg.num_joints)
        self.joint_names: List[str] = list(self.cfg.joints)
        self.joint_name_to_idx: Dict[str, int] = {name: i for i, name in enumerate(self.joint_names)}

        # Articulaciones controladas por la política
        if hasattr(self.cfg, "action_indices") and self.cfg.action_indices is not None and len(self.cfg.action_indices) == self.cfg.num_actions:
            self.action_indices = list(self.cfg.action_indices)
        else:
            self.action_indices = list(range(self.cfg.num_actions))

        self.default_joint_positions = np.array(self.cfg.default_joint_positions, dtype=np.float32)

        # -------------------------------------------------------------
        # 3. Buffers de Estado y Observación
        # -------------------------------------------------------------
        self.mode = RobotMode.IDLE
        self.command_velocity = np.zeros(3, dtype=np.float32)  # [vx, vy, vyaw]
        self.measured_joint_positions = self.default_joint_positions.copy()
        self.measured_joint_velocities = np.zeros(self.num_joints, dtype=np.float32)
        self.measured_base_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)  # [w, x, y, z]
        self.measured_base_ang_vel = np.zeros(3, dtype=np.float32)  # [wx, wy, wz]

        # Señales de presencia de datos
        self._has_imu = False
        self._has_joint_states = False
        self._last_cmd_vel_time = 0.0
        self._last_imu_time = 0.0
        self._last_joint_states_time = 0.0

        # Control de inicialización suave (RL_INIT)
        self._init_start_time = 0.0
        self._init_start_positions = self.default_joint_positions.copy()

        # Telemetría de desempeño
        self._step_count = 0
        self._last_log_time = time.time()
        self._last_inference_ms = 0.0

        # -------------------------------------------------------------
        # 4. Suscriptores ROS 2
        # -------------------------------------------------------------
        # Comandos de velocidad (/cmd_vel)
        self.sub_cmd_vel = self.create_subscription(
            Twist,
            cmd_vel_topic,
            self._on_cmd_vel,
            10
        )

        # Modo de operación (/robot_mode)
        self.sub_mode = self.create_subscription(
            Int32,
            mode_topic,
            self._on_robot_mode,
            10
        )

        # Sensores: QoS optimizado para latencia mínima
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # IMU (/imu/data)
        self.sub_imu = self.create_subscription(
            Imu,
            imu_topic,
            self._on_imu,
            sensor_qos
        )

        # Articulaciones (/joint_states)
        self.sub_joint_states = self.create_subscription(
            JointState,
            joint_states_topic,
            self._on_joint_states,
            sensor_qos
        )

        # -------------------------------------------------------------
        # 5. Publicadores ROS 2
        # -------------------------------------------------------------
        if self.publish_joint_states:
            self.pub_joint_commands = self.create_publisher(
                JointState,
                self.joint_commands_topic,
                10
            )

        if self.publish_forward_commands:
            self.pub_forward_commands = self.create_publisher(
                Float64MultiArray,
                self.forward_commands_topic,
                10
            )

        self.pub_status = self.create_publisher(String, f"{self.get_name()}/status", 10)
        self.pub_debug_obs = self.create_publisher(Float32MultiArray, f"{self.get_name()}/debug_observations", 10)

        # -------------------------------------------------------------
        # 6. Servicios ROS 2
        # -------------------------------------------------------------
        self.srv_reset = self.create_service(Trigger, f"{self.get_name()}/reset_policy", self._on_srv_reset)

        # -------------------------------------------------------------
        # 7. Temporizador Principal del Bucle de Inferencia
        # -------------------------------------------------------------
        self.timer = self.create_timer(self.timer_period, self._timer_callback)

        self.get_logger().info("=" * 60)
        self.get_logger().info(" NODO DE POLÍTICA ROS 2 INICIALIZADO EXITOSAMENTE")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"  Tópico cmd_vel       : {cmd_vel_topic}")
        self.get_logger().info(f"  Tópico joint_states   : {joint_states_topic}")
        self.get_logger().info(f"  Tópico imu            : {imu_topic}")
        self.get_logger().info(f"  Tópico joint_commands : {self.joint_commands_topic}")
        self.get_logger().info(f"  Modo inicial          : {self._mode_str(self.mode)}")
        self.get_logger().info("=" * 60)

    # -----------------------------------------------------------------
    # Callbacks de Suscripción
    # -----------------------------------------------------------------
    def _on_cmd_vel(self, msg: Twist) -> None:
        self.command_velocity[0] = float(msg.linear.x)
        self.command_velocity[1] = float(msg.linear.y)
        self.command_velocity[2] = float(msg.angular.z)
        self._last_cmd_vel_time = time.time()

    def _on_robot_mode(self, msg: Int32) -> None:
        new_mode = int(msg.data)
        if new_mode in (RobotMode.IDLE, RobotMode.RL_INIT, RobotMode.RL_RUNNING):
            self._set_mode(new_mode)
        else:
            self.get_logger().warn(f"Modo recibido desconocido: {new_mode}")

    def _on_imu(self, msg: Imu) -> None:
        # ROS 2 Imu quaternion es [x, y, z, w], RlController espera [w, x, y, z]
        self.measured_base_quat[0] = float(msg.orientation.w)
        self.measured_base_quat[1] = float(msg.orientation.x)
        self.measured_base_quat[2] = float(msg.orientation.y)
        self.measured_base_quat[3] = float(msg.orientation.z)

        # Velocidad angular [wx, wy, wz] en rad/s
        self.measured_base_ang_vel[0] = float(msg.angular_velocity.x)
        self.measured_base_ang_vel[1] = float(msg.angular_velocity.y)
        self.measured_base_ang_vel[2] = float(msg.angular_velocity.z)

        self._has_imu = True
        self._last_imu_time = time.time()

    def _on_joint_states(self, msg: JointState) -> None:
        if not msg.name or not msg.position:
            return

        # Mapear por nombre de articulación para no depender del orden del mensaje
        for name, pos, *rest in zip(msg.name, msg.position):
            idx = self.joint_name_to_idx.get(name)
            if idx is not None:
                self.measured_joint_positions[idx] = float(pos)

        if msg.velocity:
            for name, vel in zip(msg.name, msg.velocity):
                idx = self.joint_name_to_idx.get(name)
                if idx is not None:
                    self.measured_joint_velocities[idx] = float(vel)

        self._has_joint_states = True
        self._last_joint_states_time = time.time()

    # -----------------------------------------------------------------
    # Callbacks de Servicio
    # -----------------------------------------------------------------
    def _on_srv_reset(self, request, response) -> Trigger.Response:
        self.controller.reset()
        self.command_velocity.fill(0.0)
        self.get_logger().info("Servicio reset_policy ejecutado: buffers reiniciados.")
        response.success = True
        response.message = "Buffers de la política reiniciados correctamente."
        return response

    # -----------------------------------------------------------------
    # Manejo de Modos y Seguridad
    # -----------------------------------------------------------------
    def _mode_str(self, mode: int) -> str:
        if mode == RobotMode.IDLE:
            return "IDLE (1)"
        elif mode == RobotMode.RL_INIT:
            return "RL_INIT (2)"
        elif mode == RobotMode.RL_RUNNING:
            return "RL_RUNNING (3)"
        return f"DESCONOCIDO ({mode})"

    def _set_mode(self, new_mode: int) -> None:
        if self.mode == new_mode:
            return

        self.get_logger().info(f"Transición de modo: {self._mode_str(self.mode)} -> {self._mode_str(new_mode)}")

        if new_mode == RobotMode.RL_INIT:
            self._init_start_time = time.time()
            self._init_start_positions = self.measured_joint_positions.copy()

        elif new_mode == RobotMode.RL_RUNNING:
            self.controller.reset()

        self.mode = new_mode

    # -----------------------------------------------------------------
    # Bucle Principal de Ejecución (Timer Callback)
    # -----------------------------------------------------------------
    def _timer_callback(self) -> None:
        now = time.time()

        # 1. Watchdog para /cmd_vel (seguridad si se pierde la comunicación)
        if self._last_cmd_vel_time > 0 and (now - self._last_cmd_vel_time > self.watchdog_timeout):
            if np.any(np.abs(self.command_velocity) > 1e-4):
                self.command_velocity.fill(0.0)
                self.get_logger().warn(
                    f"Watchdog /cmd_vel activado (> {self.watchdog_timeout}s sin comandos). Velocidad forzada a cero.",
                    throttle_duration_sec=3.0
                )

        # 2. Verificación de salud de sensores
        sensors_ok = True
        if not self._has_imu or not self._has_joint_states:
            sensors_ok = False
            self.get_logger().warn(
                f"Esperando telemetría de sensores: IMU={self._has_imu}, JointStates={self._has_joint_states}",
                throttle_duration_sec=2.0
            )
        elif (now - self._last_imu_time > self.sensor_timeout) or (now - self._last_joint_states_time > self.sensor_timeout):
            sensors_ok = False
            self.get_logger().warn(
                f"Telemetría de sensores obsoleta (> {self.sensor_timeout}s). Manteniendo postura segura.",
                throttle_duration_sec=2.0
            )

        # 3. Ejecución según el modo del robot
        target_positions = self.default_joint_positions.copy()

        if not sensors_ok or self.mode == RobotMode.IDLE:
            # En IDLE mantenemos la posición medida actual o postura por defecto
            if self._has_joint_states:
                target_positions = self.measured_joint_positions.copy()

        elif self.mode == RobotMode.RL_INIT:
            # Interpolación suave hacia default_joint_positions para incorporarse de pie
            elapsed = now - self._init_start_time
            progress = min(max(elapsed / max(self.init_duration, 1e-3), 0.0), 1.0)
            target_positions = self._init_start_positions * (1.0 - progress) + self.default_joint_positions * progress

            if progress >= 1.0:
                if self.auto_start_after_init:
                    self.get_logger().info("Secuencia de incorporación RL_INIT completada. Iniciando RL_RUNNING.")
                    self._set_mode(RobotMode.RL_RUNNING)
                else:
                    self.get_logger().info("RL_INIT completado. Esperando comando para activar RL_RUNNING.", throttle_duration_sec=5.0)

        elif self.mode == RobotMode.RL_RUNNING:
            # ---------------------------------------------------------
            # Inferencia de la Política de Refuerzo (RL)
            # ---------------------------------------------------------
            t_start = time.perf_counter()

            # Construir vector de observaciones de bajo nivel:
            # [base_quat (4), base_ang_vel (3), joint_pos (num_actions), joint_vel (num_actions), mode (1), cmd_vel (3)]
            controlled_joint_pos = self.measured_joint_positions[self.action_indices]
            controlled_joint_vel = self.measured_joint_velocities[self.action_indices]

            robot_observations = np.concatenate([
                self.measured_base_quat,                     # 4
                self.measured_base_ang_vel,                  # 3
                controlled_joint_pos,                        # num_actions
                controlled_joint_vel,                        # num_actions
                np.array([float(self.mode)], dtype=np.float32),  # 1
                self.command_velocity,                       # 3
            ], axis=0).astype(np.float32)

            # Inferencia de la red neuronal
            policy_actions_scaled = self.controller.update(robot_observations)

            self._last_inference_ms = (time.perf_counter() - t_start) * 1000.0

            # Aplicar acciones escaladas a las articulaciones controladas
            target_positions[self.action_indices] = policy_actions_scaled

            # Publicar observaciones para depuración/análisis
            debug_msg = Float32MultiArray()
            debug_msg.data = self.controller.policy_observations.flatten().tolist()
            self.pub_debug_obs.publish(debug_msg)

        # 4. Publicar Comandos Articulares
        self._publish_commands(target_positions)

        # 5. Publicar Diagnóstico y Telemetría
        self._step_count += 1
        if now - self._last_log_time >= 1.0:
            status_str = (
                f"Modo: {self._mode_str(self.mode)} | "
                f"Cmd: [Vx={self.command_velocity[0]:+.2f}, Vy={self.command_velocity[1]:+.2f}, Wz={self.command_velocity[2]:+.2f}] | "
                f"Inferencia: {self._last_inference_ms:.2f} ms | "
                f"Pasos: {self._step_count}"
            )
            msg_status = String()
            msg_status.data = status_str
            self.pub_status.publish(msg_status)
            self._last_log_time = now

    # -----------------------------------------------------------------
    # Publicación de Comandos
    # -----------------------------------------------------------------
    def _publish_commands(self, target_positions: np.ndarray) -> None:
        stamp = self.get_clock().now().to_msg()

        # Publicar como sensor_msgs/JointState
        if self.publish_joint_states:
            js_msg = JointState()
            js_msg.header.stamp = stamp
            js_msg.header.frame_id = "base_link"

            if self.publish_all_joints:
                js_msg.name = self.joint_names
                js_msg.position = [float(p) for p in target_positions]
            else:
                js_msg.name = [self.joint_names[i] for i in self.action_indices]
                js_msg.position = [float(target_positions[i]) for i in self.action_indices]

            self.pub_joint_commands.publish(js_msg)

        # Publicar como Float64MultiArray para ros2_control forward controllers
        if self.publish_forward_commands:
            fc_msg = Float64MultiArray()
            if self.publish_all_joints:
                fc_msg.data = [float(p) for p in target_positions]
            else:
                fc_msg.data = [float(target_positions[i]) for i in self.action_indices]

            self.pub_forward_commands.publish(fc_msg)


def main(args=None):
    if not RCLPY_AVAILABLE:
        print("[ERROR] rclpy no está instalado en este entorno de Python.")
        print("Para ejecutar el nodo ROS 2, utiliza un entorno con ROS 2 (Humble, Iron, Jazzy, etc.).")
        sys.exit(1)

    rclpy.init(args=args)
    node = Ros2PolicyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[INFO] Detención solicitada por el usuario.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("[INFO] Nodo de política ROS 2 finalizado limpiamente.")


if __name__ == "__main__":
    main()
