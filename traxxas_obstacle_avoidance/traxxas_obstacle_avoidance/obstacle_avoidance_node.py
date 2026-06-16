#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import signal
from enum import Enum

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, String


class AvoidState(Enum):
    IDLE = 0
    STEER_OUT = 1
    PASS_OBSTACLE = 2
    STEER_BACK = 3
    SEARCH_LANE = 4


class ReactiveObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('reactive_obstacle_avoidance_node')

        # ============================================================
        # Parámetros generales
        # ============================================================
        self.declare_parameter('control_hz', 20.0)

        # Dirección de la maniobra: 'left' o 'right'
        self.declare_parameter('evade_side', 'left')

        # Tiempos por fase [s]
        self.declare_parameter('steer_out_time', 0.45)
        self.declare_parameter('pass_obstacle_time', 0.90)
        self.declare_parameter('steer_back_time', 0.45)
        self.declare_parameter('search_lane_timeout', 2.50)

        # Cuántos ciclos seguidos debe verse línea para soltar control
        self.declare_parameter('lane_detect_stable_cycles', 6)

        # Si vuelve a aparecer obstáculo durante SEARCH_LANE o PASS_OBSTACLE,
        # puedes extender PASS_OBSTACLE con este tiempo extra
        self.declare_parameter('extra_pass_time_on_new_obstacle', 0.40)

        # ============================================================
        # PWM / comandos
        # ============================================================
        self.declare_parameter('dir_center', 2642)
        self.declare_parameter('dir_max_right', 3276)
        self.declare_parameter('dir_min_left', 1669)

        self.declare_parameter('throttle_center', 2457)
        self.declare_parameter('throttle_cruise', 2600)
        self.declare_parameter('throttle_search', 2550)

        # Un sesgo suave durante búsqueda de línea
        self.declare_parameter('search_bias_pwm', 180)

        # ============================================================
        # Topics
        # ============================================================
        self.declare_parameter('obstacle_topic', '/obstacle')
        self.declare_parameter('lane_detect_topic', '/signal/detected')
        self.declare_parameter('obstacle_active_topic', '/traxxas/obstacle_active')
        self.declare_parameter('direction_topic', '/direction_servo')
        self.declare_parameter('throttle_topic', '/throttle_motor')

        # ============================================================
        # Leer parámetros
        # ============================================================
        self.control_hz = float(self.get_parameter('control_hz').value)

        self.evade_side = str(self.get_parameter('evade_side').value).strip().lower()

        self.steer_out_time = float(self.get_parameter('steer_out_time').value)
        self.pass_obstacle_time = float(self.get_parameter('pass_obstacle_time').value)
        self.steer_back_time = float(self.get_parameter('steer_back_time').value)
        self.search_lane_timeout = float(self.get_parameter('search_lane_timeout').value)

        self.lane_detect_stable_cycles_required = int(
            self.get_parameter('lane_detect_stable_cycles').value
        )

        self.extra_pass_time_on_new_obstacle = float(
            self.get_parameter('extra_pass_time_on_new_obstacle').value
        )

        self.dir_center = int(self.get_parameter('dir_center').value)
        self.dir_max_right = int(self.get_parameter('dir_max_right').value)
        self.dir_min_left = int(self.get_parameter('dir_min_left').value)

        self.throttle_center = int(self.get_parameter('throttle_center').value)
        self.throttle_cruise = int(self.get_parameter('throttle_cruise').value)
        self.throttle_search = int(self.get_parameter('throttle_search').value)

        self.search_bias_pwm = int(self.get_parameter('search_bias_pwm').value)

        self.obstacle_topic = str(self.get_parameter('obstacle_topic').value)
        self.lane_detect_topic = str(self.get_parameter('lane_detect_topic').value)
        self.obstacle_active_topic = str(self.get_parameter('obstacle_active_topic').value)
        self.direction_topic = str(self.get_parameter('direction_topic').value)
        self.throttle_topic = str(self.get_parameter('throttle_topic').value)

        if self.evade_side not in ('left', 'right'):
            self.get_logger().warn(
                f"evade_side='{self.evade_side}' inválido. Usando 'left'."
            )
            self.evade_side = 'left'

        # ============================================================
        # Estado interno
        # ============================================================
        self.state = AvoidState.IDLE
        self.state_start_time = self.get_clock().now()

        self.avoidance_active = False
        self.last_obstacle_seen = False
        self.lane_detected = False
        self.lane_detect_stable_count = 0

        # Anti rebote de trigger
        self.trigger_latched = False

        # ============================================================
        # Publishers
        # ============================================================
        self.pub_direction = self.create_publisher(String, self.direction_topic, 10)
        self.pub_throttle = self.create_publisher(String, self.throttle_topic, 10)
        self.pub_obstacle_active = self.create_publisher(Bool, self.obstacle_active_topic, 10)

        # ============================================================
        # Subscribers
        # ============================================================
        self.create_subscription(
            Bool,
            self.obstacle_topic,
            self.obstacle_callback,
            10
        )

        self.create_subscription(
            Bool,
            self.lane_detect_topic,
            self.lane_detect_callback,
            10
        )

        # ============================================================
        # Timer
        # ============================================================
        period = 1.0 / max(self.control_hz, 1.0)
        self.timer = self.create_timer(period, self.control_loop)

        # Estado inicial
        self._pub_obstacle_active(False)
        self._publish_safe_stop()

        self.get_logger().info(
            "ReactiveObstacleAvoidanceNode listo | "
            f"side={self.evade_side} | "
            f"times=({self.steer_out_time:.2f}, {self.pass_obstacle_time:.2f}, "
            f"{self.steer_back_time:.2f}, search={self.search_lane_timeout:.2f})"
        )

    # ================================================================
    # Callbacks
    # ================================================================
    def obstacle_callback(self, msg: Bool):
        detected = bool(msg.data)
        self.last_obstacle_seen = detected

        # Trigger solo por flanco de subida
        if detected and not self.trigger_latched:
            self.trigger_latched = True

            if self.state == AvoidState.IDLE:
                self.start_maneuver()
            else:
                # Si ya estamos maniobrando, solo extendemos la parte recta
                # para dar más margen si reaparece obstáculo.
                if self.state in (AvoidState.PASS_OBSTACLE, AvoidState.SEARCH_LANE):
                    self.get_logger().warn(
                        "Obstáculo re-detectado durante la maniobra. Extendiendo evasión."
                    )
                    self.transition_to(AvoidState.PASS_OBSTACLE)
        elif not detected:
            self.trigger_latched = False

    def lane_detect_callback(self, msg: Bool):
        self.lane_detected = bool(msg.data)

        if self.lane_detected:
            self.lane_detect_stable_count += 1
        else:
            self.lane_detect_stable_count = 0

    # ================================================================
    # Control principal
    # ================================================================
    def control_loop(self):
        if self.state == AvoidState.IDLE:
            return

        # Mientras maniobra, sostén ownership
        self._pub_obstacle_active(True)

        elapsed = self._state_elapsed()

        if self.state == AvoidState.STEER_OUT:
            servo_pwm = self._servo_out_pwm()
            throttle_pwm = self.throttle_cruise

            self._pub_direction(servo_pwm)
            self._pub_throttle(throttle_pwm)

            if elapsed >= self.steer_out_time:
                self.transition_to(AvoidState.PASS_OBSTACLE)

        elif self.state == AvoidState.PASS_OBSTACLE:
            servo_pwm = self.dir_center
            throttle_pwm = self.throttle_cruise

            self._pub_direction(servo_pwm)
            self._pub_throttle(throttle_pwm)

            pass_time = self.pass_obstacle_time
            if self.last_obstacle_seen:
                pass_time += self.extra_pass_time_on_new_obstacle

            if elapsed >= pass_time:
                self.transition_to(AvoidState.STEER_BACK)

        elif self.state == AvoidState.STEER_BACK:
            servo_pwm = self._servo_back_pwm()
            throttle_pwm = self.throttle_cruise

            self._pub_direction(servo_pwm)
            self._pub_throttle(throttle_pwm)

            if elapsed >= self.steer_back_time:
                self.transition_to(AvoidState.SEARCH_LANE)

        elif self.state == AvoidState.SEARCH_LANE:
            servo_pwm = self._search_servo_pwm()
            throttle_pwm = self.throttle_search

            self._pub_direction(servo_pwm)
            self._pub_throttle(throttle_pwm)

            # Si la línea se detecta estable varios ciclos, soltamos control
            if self.lane_detect_stable_count >= self.lane_detect_stable_cycles_required:
                self.get_logger().warn(
                    f"Línea detectada de forma estable "
                    f"({self.lane_detect_stable_count} ciclos). Regresando control."
                )
                self.finish_maneuver()
                return

            # Failsafe: si no encuentra línea en cierto tiempo, igual soltamos
            # para no secuestrar el control eternamente.
            if elapsed >= self.search_lane_timeout:
                self.get_logger().warn(
                    "Timeout buscando línea. Regresando control al nodo principal."
                )
                self.finish_maneuver()
                return

    # ================================================================
    # Estados
    # ================================================================
    def start_maneuver(self):
        if self.state != AvoidState.IDLE:
            return

        self.avoidance_active = True
        self.lane_detect_stable_count = 0

        self._pub_obstacle_active(True)
        self.transition_to(AvoidState.STEER_OUT)

        self.get_logger().warn(
            f"Trigger /obstacle recibido. Iniciando evasión hacia {self.evade_side}."
        )

    def finish_maneuver(self):
        self.state = AvoidState.IDLE
        self.avoidance_active = False
        self.lane_detect_stable_count = 0

        # Deja el carro en un estado razonable antes de ceder control
        self._pub_direction(self.dir_center)
        self._pub_throttle(self.throttle_center)

        # Soltar ownership
        self._pub_obstacle_active(False)

        self.get_logger().warn("Maniobra terminada. Control devuelto al nodo principal.")

    def transition_to(self, new_state: AvoidState):
        self.state = new_state
        self.state_start_time = self.get_clock().now()
        self.get_logger().info(f"Estado -> {self.state.name}")

    # ================================================================
    # Helpers de tiempo
    # ================================================================
    def _state_elapsed(self) -> float:
        now = self.get_clock().now()
        return (now.nanoseconds - self.state_start_time.nanoseconds) * 1e-9

    # ================================================================
    # Helpers de comandos
    # ================================================================
    def _servo_out_pwm(self) -> int:
        return self.dir_min_left if self.evade_side == 'left' else self.dir_max_right

    def _servo_back_pwm(self) -> int:
        return self.dir_max_right if self.evade_side == 'left' else self.dir_min_left

    def _search_servo_pwm(self) -> int:
        # Búsqueda suave en la misma dirección donde terminó la maniobra
        if self.evade_side == 'left':
            return int(self.dir_center - abs(self.search_bias_pwm))
        return int(self.dir_center + abs(self.search_bias_pwm))

    def _pub_direction(self, value: int):
        msg = String()
        msg.data = str(int(value))
        self.pub_direction.publish(msg)

    def _pub_throttle(self, value: int):
        msg = String()
        msg.data = str(int(value))
        self.pub_throttle.publish(msg)

    def _pub_obstacle_active(self, flag: bool):
        msg = Bool()
        msg.data = bool(flag)
        self.pub_obstacle_active.publish(msg)

    def _publish_safe_stop(self):
        self._pub_direction(self.dir_center)
        self._pub_throttle(self.throttle_center)

    # ================================================================
    # Cierre limpio
    # ================================================================
    def safe_shutdown(self):
        self.get_logger().warn("Apagado seguro: centrando dirección y neutralizando throttle.")
        for _ in range(5):
            self._publish_safe_stop()
            self._pub_obstacle_active(False)
            time.sleep(0.05)


def main(args=None):
    rclpy.init(args=args)
    node = ReactiveObstacleAvoidanceNode()

    def stop_handler(sig, frame):
        node.safe_shutdown()
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, stop_handler)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.safe_shutdown()
        except Exception:
            pass

        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()