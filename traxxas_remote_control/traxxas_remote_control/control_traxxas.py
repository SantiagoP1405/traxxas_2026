import os
import logging
from datetime import datetime

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


# ─────────────────────────────────────────────
# CALIBRACIÓN PWM → GRADOS REALES
# ─────────────────────────────────────────────
def pwm_to_deg(pwm: int) -> float:
    pwm_min, pwm_center, pwm_max = 1669, 2642, 3276
    max_right = 20.0
    max_left  = 20.0

    if pwm >= pwm_center:
        norm = (pwm - pwm_center) / (pwm_max - pwm_center)
        return norm * max_right
    else:
        norm = (pwm - pwm_center) / (pwm_center - pwm_min)
        return norm * max_left


# ─────────────────────────────────────────────
# CONFIGURACIÓN DEL LOGGER DE ARCHIVO
# ─────────────────────────────────────────────
def setup_file_logger(name: str = "joy_control") -> logging.Logger:
    """
    Crea un logger de Python que escribe en un .txt junto al script.
    El archivo se nombra con timestamp para no sobreescribir sesiones anteriores.
    """
    script_dir = os.path.dirname(os.path.abspath(__file__))
    timestamp  = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path   = os.path.join(script_dir, f"{name}_{timestamp}.txt")

    file_logger = logging.getLogger(name)
    file_logger.setLevel(logging.DEBUG)

    handler = logging.FileHandler(log_path, encoding="utf-8")
    handler.setLevel(logging.DEBUG)
    formatter = logging.Formatter(
        fmt="[%(asctime)s] %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S"
    )
    handler.setFormatter(formatter)
    file_logger.addHandler(handler)

    # Evitar que suba al root logger (no duplicar en consola desde aquí)
    file_logger.propagate = False

    print(f"[FILE LOGGER] Guardando log en: {log_path}")
    return file_logger


class JoyMicroRosControl(Node):
    # PWM values para STEERING (14 bits, 100Hz)
    STEER_MAX    = 3276
    STEER_CENTER = 2525
    STEER_MIN    = 1669

    # PWM values para THROTTLE (14 bits, 100Hz)
    THROTTLE_MAX    = 2750
    THROTTLE_CENTER = 2457
    THROTTLE_MIN    = 2100

    def __init__(self):
        super().__init__('joystick_microros_controller')

        # Logger de archivo (independiente del logger de ROS2)
        self.file_logger = setup_file_logger("joy_control")

        self.throttle_fwd_ = 0.0
        self.throttle_rev_ = 0.0
        self.steering_     = 0.0

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscriber_  = self.create_subscription(Joy, '/joy', self.joy_callback, qos_profile)
        self.pub_steering = self.create_publisher(String, 'direction_servo', qos_profile)
        self.pub_throttle = self.create_publisher(String, 'throttle_motor',  qos_profile)
        self.pub_led      = self.create_publisher(String, 'led_power',        qos_profile)

        self.timer_ = self.create_timer(0.1, self.timer_callback)

        self.log('Joystick MicroROS Controller iniciado')
        self.log('R2: Adelante | L2: Reversa | Stick Izq: Direccion')
        self.log(f'Steering PWM: {self.STEER_MIN} - {self.STEER_CENTER} - {self.STEER_MAX}')
        self.log(f'Throttle PWM: {self.THROTTLE_MIN} - {self.THROTTLE_CENTER} - {self.THROTTLE_MAX}')

    # ──────────────────────────────────────────
    # Método auxiliar: terminal + archivo
    # ──────────────────────────────────────────
    def log(self, msg: str, level: str = "info") -> None:
        """Escribe el mensaje en la terminal (ROS logger) y en el archivo .txt."""
        ros_logger = self.get_logger()
        getattr(ros_logger, level)(msg)   # info / warn / error según `level`
        self.file_logger.info(msg)        # siempre INFO en archivo

    def joy_callback(self, msg):
        self.steering_     = msg.axes[0]
        self.throttle_fwd_ = (1.0 - msg.axes[4]) / 2.0
        self.throttle_rev_ = (1.0 - msg.axes[3]) / 2.0

    def timer_callback(self):
        # === STEERING ===
        if self.steering_ >= 0:
            pwm_steering = int(self.STEER_CENTER - (self.STEER_CENTER - self.STEER_MIN) * self.steering_)
        else:
            pwm_steering = int(self.STEER_CENTER + (self.STEER_MAX - self.STEER_CENTER) * abs(self.steering_))

        real_deg = pwm_to_deg(pwm_steering)
        self.log(
            f"PWM_steer={pwm_steering}  ->  {real_deg:+.1f}°  "
            f"({'IZQ' if real_deg < 0 else 'DER' if real_deg > 0 else 'RECTO'})"
        )

        # === THROTTLE ===
        if self.throttle_fwd_ > 0.05:
            pwm_throttle = int(self.THROTTLE_CENTER + (self.THROTTLE_MAX - self.THROTTLE_CENTER) * self.throttle_fwd_)
        elif self.throttle_rev_ > 0.05:
            pwm_throttle = int(self.THROTTLE_CENTER - (self.THROTTLE_CENTER - self.THROTTLE_MIN) * self.throttle_rev_)
        else:
            pwm_throttle = self.THROTTLE_CENTER

        # === Publicar ===
        msg_steer      = String()
        msg_steer.data = str(pwm_steering)
        self.pub_steering.publish(msg_steer)

        msg_throttle      = String()
        msg_throttle.data = str(pwm_throttle)
        self.pub_throttle.publish(msg_throttle)

        # === LEDs ===
        led_msg = String()
        if abs(self.steering_) > 0.3:
            led_msg.data = "L" if self.steering_ > 0 else "R"
        elif self.throttle_rev_ > 0.05:
            led_msg.data = "S"
        else:
            led_msg.data = "F"
        self.pub_led.publish(led_msg)


def main(args=None):
    rclpy.init(args=args)
    node = JoyMicroRosControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
