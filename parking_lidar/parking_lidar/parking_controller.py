#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3Stamped
import math

# --- TUS CONSTANTES ORIGINALES ---
REVERSE_SPEED = -0.15
FORWARD_SPEED = 0.5

KP_YAW = 0.5
KP_LAT = 30.0
MAX_ANG = 1.0

STOP_DIST = 0.20
ALIGN_TOL = 0.04
CENTER_TOL = 0.15

FINAL_DIST = 0.12
FINAL_TOL  = 0.005
FINAL_SPEED = -0.05

YAW_REF = math.pi / 2

# --- NUEVA CONSTANTE PARA EL RESCATE ---
# Si quieres que se eche para adelante "antes" (mas tiempo), sube este valor a 0.20 o 0.25
RESCUE_DIST = 0.15 

class ParkingController(Node):
    def __init__(self):
        super().__init__('parking_controller_cmdvel')

        self.state = 'BUSCAR'
        self.parking_side = 'RIGHT'   # RIGHT / LEFT
        self.yaw = 0.0

        self.ul = self.ur = self.ub = 1.0

        self.create_subscription(String, '/parking/state', self.state_cb, 10)
        self.create_subscription(Imu, '/imu', self.imu_cb, 10)
        self.create_subscription(LaserScan, '/ultrasonic/rear_left', self.ul_cb, 10)
        self.create_subscription(LaserScan, '/ultrasonic/rear_right', self.ur_cb, 10)
        self.create_subscription(LaserScan, '/ultrasonic/rear_center', self.ub_cb, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.qcar_pub = self.create_publisher(Vector3Stamped, '/qcar/user_command', 10)
        self.state_pub = self.create_publisher(String, '/parking/state_feedback', 10)

        self.timer = self.create_timer(0.05, self.loop)

        self.get_logger().info('Parking Controller HIBRIDO iniciado (Con Rescate)')

    def state_cb(self, msg):
        if ':' in msg.data:
            self.state, self.parking_side = msg.data.split(':')
        else:
            self.state = msg.data

    def imu_cb(self, msg):
        q = msg.orientation
        self.yaw = math.atan2(
            2*(q.w*q.z + q.x*q.y),
            1 - 2*(q.y*q.y + q.z*q.z)
        )

    def get_range(self, msg):
        vals = [r for r in msg.ranges if r > 0.05 and not math.isinf(r)]
        return sum(vals)/len(vals) if vals else 5.0

    def ul_cb(self, msg): self.ul = self.get_range(msg)
    def ur_cb(self, msg): self.ur = self.get_range(msg)
    def ub_cb(self, msg): self.ub = self.get_range(msg)

    def loop(self):
        cmd = Twist()

        if self.state == 'BUSCAR':
            cmd.linear.x = FORWARD_SPEED

        elif self.state == 'ESPERAR':
            cmd.linear.x = 0.0

        elif self.state == 'ENTRAR_DIAGONAL':
            cmd.linear.x = -0.2
            # MANTENEMOS TU LOGICA EXACTA DE GIRO INICIAL
            cmd.angular.z = 1.0 if self.parking_side == 'RIGHT' else -1.0

            self.get_logger().info(
                f'ENTRAR_DIAGONAL | side:{self.parking_side}'
            )

        elif self.state == 'ENDEREZAR':
            
            # --- INICIO DE LOGICA DE RESCATE ---
            do_rescue = False
            rescue_steer = 0.0

            # Si voy a la IZQ (LEFT) y el sensor DERECHO (ur) esta muy cerca:
            if self.parking_side == 'LEFT' and self.ur < RESCUE_DIST:
                do_rescue = True
                # Avanzo y giro ruedas a la derecha (-0.5) para abrir la cola a la izq
                rescue_steer = -0.5
                self.get_logger().error(f'RESCATE: Muy pegado a la derecha (ur={self.ur:.2f})')

            # Si voy a la DER (RIGHT) y el sensor IZQUIERDO (ul) esta muy cerca:
            elif self.parking_side == 'RIGHT' and self.ul < RESCUE_DIST:
                do_rescue = True
                # Avanzo y giro ruedas a la izquierda (0.5) para abrir la cola a la der
                rescue_steer = 0.5
                self.get_logger().error(f'RESCATE: Muy pegado a la izquierda (ul={self.ul:.2f})')

            if do_rescue:
                # Accion de rescate
                cmd.linear.x = 0.15  # Velocidad suave Hacia Adelante
                cmd.angular.z = rescue_steer
            
            else:
                # --- TU LOGICA ORIGINAL (PID) SI NO HAY PELIGRO ---
                yaw_err = math.atan2(
                    math.sin(YAW_REF - self.yaw),
                    math.cos(YAW_REF - self.yaw)
                )

                lat_err = (
                    self.ul - self.ur
                    if self.parking_side == 'RIGHT'
                    else self.ur - self.ul
                )

                ang = KP_YAW * yaw_err + KP_LAT * lat_err

                if self.parking_side == 'LEFT':
                    ang = -ang

                ang = max(-MAX_ANG, min(MAX_ANG, ang))

                cmd.linear.x = -0.2
                cmd.angular.z = ang

                self.get_logger().warn(
                    f'ENDEREZAR | side:{self.parking_side} '
                    f'yaw:{self.yaw:.3f} err:{yaw_err:.3f} '
                    f'lat:{lat_err:.3f} ul:{self.ul:.3f} ur:{self.ur:.3f}'
                )

                if abs(yaw_err) < ALIGN_TOL and abs(lat_err) < CENTER_TOL:
                    self.state_pub.publish(String(data='ENDEREZADO'))
            # --- FIN LOGICA RESCATE / PID ---

        elif self.state == 'ESTACIONANDO':
            cmd.linear.x = REVERSE_SPEED
            self.get_logger().warn(f'ESTACIONANDO | ub:{self.ub:.3f}')

            if self.ub <= STOP_DIST:
                self.state_pub.publish(String(data='DIST_OK'))

        elif self.state == 'STOP':
            if self.ub > FINAL_DIST + FINAL_TOL:
                cmd.linear.x = FINAL_SPEED
            else:
                cmd.linear.x = 0.0

        # --- PUBLICACION FINAL (Igual que tu codigo base) ---
        
        self.cmd_pub.publish(cmd)

        cmd_real = Vector3Stamped()
        cmd_real.header.stamp = self.get_clock().now().to_msg()
        cmd_real.vector.x = float(cmd.linear.x)
        
        steer = float(cmd.angular.z)
        
        # Clamp fisico del QCar
        if steer > 0.5: steer = 0.5
        if steer < -0.5: steer = -0.5
        
        cmd_real.vector.y = steer

        self.qcar_pub.publish(cmd_real)

def main():
    rclpy.init()
    node = ParkingController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()