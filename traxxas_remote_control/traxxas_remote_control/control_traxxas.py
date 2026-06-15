import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class JoyMicroRosControl(Node):
    # PWM values para STEERING (14 bits, 100Hz)
    STEER_MAX = 3276    # Máxima derecha
    STEER_CENTER = 2642 # Centro
    STEER_MIN = 1669     # Máxima izquierda
    
    # PWM values para THROTTLE (14 bits, 100Hz)
    THROTTLE_MAX = 2710    # Máximo adelante
    THROTTLE_CENTER = 2457  # Neutro
    THROTTLE_MIN = 2200     # Máximo reversa
    
    def __init__(self):
        super().__init__('joystick_microros_controller')
        
        # Estado actual
        self.throttle_fwd_ = 0.0   # R2 (axes[5]): adelante
        self.throttle_rev_ = 0.0   # L2 (axes[2]): reversa
        self.steering_ = 0.0       # Stick izq (axes[0])
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscriber para el joystick
        self.subscriber_ = self.create_subscription(Joy, '/joy', self.joy_callback, qos_profile)
        
        # Publishers para micro-ROS (String messages)
        # L = dirección (steering), R = velocidad (throttle)
        self.pub_steering = self.create_publisher(String, 'direction_servo', qos_profile)
        self.pub_throttle = self.create_publisher(String, 'throttle_motor', qos_profile)
        self.pub_led = self.create_publisher(String, 'led_power', qos_profile)
        
        # Timer para publicar a 100Hz
        self.timer_ = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info('Joystick MicroROS Controller iniciado')
        self.get_logger().info('R2: Adelante | L2: Reversa | Stick Izq: Dirección')
        self.get_logger().info(f'Steering PWM: {self.STEER_MIN} - {self.STEER_CENTER} - {self.STEER_MAX}')
        self.get_logger().info(f'Throttle PWM: {self.THROTTLE_MIN} - {self.THROTTLE_CENTER} - {self.THROTTLE_MAX}')
    
    def joy_callback(self, msg):
        # Stick izquierdo horizontal (axes[0]) para dirección
        # +1 = izquierda, -1 = derecha
        self.steering_ = msg.axes[0]
        
        # R2 (axes[5]) para adelante: 1.0 = sin presionar, -1.0 = full presionado
        # Normalizar a rango 0.0 (sin presionar) a 1.0 (full presionado)
        self.throttle_fwd_ = (1.0 - msg.axes[4]) / 2.0
        
        # L2 (axes[2]) para reversa: 1.0 = sin presionar, -1.0 = full presionado
        # Normalizar a rango 0.0 (sin presionar) a 1.0 (full presionado)
        self.throttle_rev_ = (1.0 - msg.axes[3]) / 2.0
    
    def timer_callback(self):
        # === STEERING (L_motor_pwm) ===
        # steering_: +1 = izquierda, -1 = derecha
        # Mapear: izquierda (+1) = MIN, centro (0) = CENTER, derecha (-1) = MAX
        if self.steering_ >= 0:
            # Girando izquierda (0 a +1) -> CENTER a MIN
            pwm_steering = int(self.STEER_CENTER - (self.STEER_CENTER - self.STEER_MIN) * self.steering_)
        else:
            # Girando derecha (0 a -1) -> CENTER a MAX
            pwm_steering = int(self.STEER_CENTER + (self.STEER_MAX - self.STEER_CENTER) * abs(self.steering_))
        
        # === THROTTLE (R_motor_pwm) ===
        # Prioridad: si ambos triggers están presionados, adelante tiene prioridad
        if self.throttle_fwd_ > 0.05:
            # Adelante: CENTER a MAX (gradual según presión de R2)
            pwm_throttle = int(self.THROTTLE_CENTER + (self.THROTTLE_MAX - self.THROTTLE_CENTER) * self.throttle_fwd_)
        elif self.throttle_rev_ > 0.05:
            # Reversa: CENTER a MIN (gradual según presión de L2)
            pwm_throttle = int(self.THROTTLE_CENTER - (self.THROTTLE_CENTER - self.THROTTLE_MIN) * self.throttle_rev_)
        else:
            # Sin presionar ningún trigger = neutro
            pwm_throttle = self.THROTTLE_CENTER
        
        # Publicar steering
        msg_steer = String()
        msg_steer.data = str(pwm_steering)
        self.pub_steering.publish(msg_steer)
        
        # Publicar throttle
        msg_throttle = String()
        msg_throttle.data = str(pwm_throttle)
        self.pub_throttle.publish(msg_throttle)
        
        # === LEDs ===
        led_msg = String()
        if self.throttle_rev_ > 0.05:
            # Reversa tiene prioridad: solo luces rojas
            led_msg.data = "S"
        elif abs(self.steering_) > 0.3:
            # Indicadores de giro (solo cuando NO va en reversa)
            led_msg.data = "L" if self.steering_ > 0 else "R"
        else:
            # Apagado
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
