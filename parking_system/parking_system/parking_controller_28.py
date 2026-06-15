#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Vector3Stamped, PoseStamped
from std_msgs.msg import String, Float32

import math
import serial
import time


# ─────────────────────────────────────────────
# CONFIGURACION DE HARDWARE
# ─────────────────────────────────────────────
SERIAL_PORT = '/dev/ttyUSB1'
BAUD_RATE   = 115200

# ─────────────────────────────────────────────
# LIMITES DE SEGURIDAD HARDWARE PWM
# ─────────────────────────────────────────────
PWM_THROTTLE_NEUTRAL = 2457
PWM_THROTTLE_FWD_MAX = 2800
PWM_THROTTLE_REV_MAX = 2150

PWM_STEER_CENTER     = 2542
PWM_STEER_LEFT_MAX   = 1669
PWM_STEER_RIGHT_MAX  = 3276

# ─────────────────────────────────────────────
# VELOCIDADES PWM
# ─────────────────────────────────────────────
SEARCH_SPEED  = 2682
PARKING_SPEED = 2235

ORIENT_SPEED           = 2670
ALEJARSE_SPEED         = 2695
ALEJARSE_SPEED_LEFT    = 2695
FORWARD_CORRECT_SPEED  = 2690
BACKWARD_CORRECT_SPEED = 2250
AJUSTE_ADELANTE_SPEED  = 2680
STOP_FASE_SPEED        = 2600

# ─────────────────────────────────────────────
# DIRECCIONES PWM
# ─────────────────────────────────────────────
STEER_ALEJARSE_DER    = 3250
STEER_ALEJARSE_IZQ    = 1950
STEER_DIAGONAL_DER    = 3250
STEER_DIAGONAL_IZQ    = 2000
STEER_CORRECT_DER     = 3200
STEER_CORRECT_IZQ     = 2000

# ─────────────────────────────────────────────
# CONTROL
# ─────────────────────────────────────────────
KP_ORIENT   = 9.0
KD_ORIENT   = 3.0
KP_YAW      = 3.5
KP_LAT      = 4520.0
MAX_ANG_PID = 300

ALIGN_TOL  = 4.5
CENTER_TOL = 0.10

# ─────────────────────────────────────────────
# PARADAS / DISTANCIAS
# ─────────────────────────────────────────────
IR_STOP_VALUE  = 0
REAR_STOP_DIST = 0.13

LATERAL_DANGER_DIST    = 0.13
REAR_DANGER_DIST       = 0.13
FORWARD_CORRECT_TIME   = 0.66
BACKWARD_CORRECT_TIME  = 0.77
AJUSTE_ADELANTE_TIME   = 0.1

# ─────────────────────────────────────────────
# ZED / IMU FUSION
# ─────────────────────────────────────────────
USE_ZED_YAW_AS_PRIMARY = True
USE_YAW_FUSION         = True
ZED_YAW_WEIGHT         = 0.7
EXT_YAW_WEIGHT         = 0.3

# Si tu tópico /imu/euler aún sale con signo opuesto respecto al marco ROS:
# pon -1.0. Si ya viene corregido, deja 1.0.
EXTERNAL_IMU_YAW_SIGN = 1.0

# Tiempo máximo para considerar fresca la ZED
ZED_TIMEOUT_SEC = 0.30

# Tiempo máximo para considerar fresca la IMU externa
IMU_TIMEOUT_SEC = 0.30


def wrap_angle_deg(angle):
    return (angle + 180.0) % 360.0 - 180.0


def angle_diff_deg(target, current):
    return wrap_angle_deg(target - current)


def blend_angles_deg(a_deg, b_deg, wa=0.5, wb=0.5):
    """
    Promedio robusto de ángulos en grados.
    """
    a_rad = math.radians(a_deg)
    b_rad = math.radians(b_deg)

    x = wa * math.cos(a_rad) + wb * math.cos(b_rad)
    y = wa * math.sin(a_rad) + wb * math.sin(b_rad)

    if abs(x) < 1e-9 and abs(y) < 1e-9:
        return wrap_angle_deg(a_deg)

    return wrap_angle_deg(math.degrees(math.atan2(y, x)))


class ParkingControllerTraxxas(Node):
    def __init__(self):
        super().__init__('parking_controller_traxxas')

        # ─────────────────────────────────────
        # ESTADO GENERAL
        # ─────────────────────────────────────
        self.state        = 'ORIENTANDOSE'
        self.parking_side = 'RIGHT'
        self.prev_state   = None

        self.lane_angle   = 90.0
        self.vision_lista = False
        self.camara_notificada = False

        self.angulo_deseado_der = 90.0
        self.angulo_deseado_izq = 87.0
        self.angulo_deseado_actual = None
        self.prev_error_angulo = None

        self.freno_emergencia = False
        self.freno_stop_dist  = False
        self.freno_final_dist = False
        self.correcting_until_time = 0.0

        self.reversa_armada      = False
        self.esc_transicion_fase = 0
        self.esc_tiempo_fase     = 0.0

        self.ul       = 1.0
        self.ub       = 1.0
        self.ur       = 1.0
        self.ir_state = -1

        self.lidar_rf = False
        self.lidar_lf = False
        self.ir_activo = False

        self.ajuste_adelante_done = False
        self.ajuste_adelante_time = 0.0

        self.stop_fase      = 0
        self.stop_fase_time = 0.0

        # ─────────────────────────────────────
        # YAW / POSE ROBUSTA
        # ─────────────────────────────────────
        self.yaw_initial_ext = None
        self.yaw_ext = 0.0
        self.ext_imu_stamp = 0.0

        self.zed_yaw = None
        self.zed_yaw_stamp = 0.0
        self.zed_tracking_ok = False

        self.yaw = 0.0            # yaw final usado por el controlador
        self.yaw_source = "NONE"

        self.yaw_target = None
        self.yaw_buscar = 0.0

        # pose ZED
        self.zed_pose_available = False
        self.zed_x = 0.0
        self.zed_y = 0.0
        self.zed_z = 0.0
        self.zed_pose_stamp = 0.0

        # marcas de pose al entrar a estados
        self.state_entry_pose = None
        self.state_entry_yaw  = None

        # ─────────────────────────────────────
        # ARDUINO
        # ─────────────────────────────────────
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
            self.get_logger().info(f'--- CONECTADO A ARDUINO ({SERIAL_PORT}) ---')
            time.sleep(2)
            self.ser.reset_input_buffer()
        except Exception as e:
            self.get_logger().error(f'ERROR ARDUINO: {e}')
            self.ser = None

        # ─────────────────────────────────────
        # SUSCRIPCIONES
        # ─────────────────────────────────────
        self.create_subscription(String, '/parking/state', self.state_cb, 10)
        self.create_subscription(String, '/parking/perception', self.perception_cb, 10)

        # IMU externa original
        self.create_subscription(Vector3Stamped, '/imu/euler', self.imu_cb, 10)

        # Ángulo visual
        self.create_subscription(Float32, '/qcar/lane_angle_ema', self.angle_cb, 10)

        # ZED
        self.create_subscription(Float32, '/zed/yaw_deg', self.zed_yaw_cb, 10)
        self.create_subscription(PoseStamped, '/zed/pose', self.zed_pose_cb, 10)
        self.create_subscription(String, '/zed/tracking_state', self.zed_tracking_cb, 10)

        # ─────────────────────────────────────
        # PUBLICADORES
        # ─────────────────────────────────────
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.direction_pwm_pub = self.create_publisher(String, 'direction_servo', qos_profile)
        self.throttle_pwm_pub  = self.create_publisher(String, 'throttle_motor', qos_profile)
        self.state_pub         = self.create_publisher(String, '/parking/state_feedback', 10)

        # debug
        self.yaw_debug_pub = self.create_publisher(Float32, '/parking/yaw_used', 10)
        self.pose_dist_pub = self.create_publisher(Float32, '/parking/state_distance', 10)

        self.timer = self.create_timer(0.05, self.loop)

    # ─────────────────────────────────────────
    # CALLBACKS
    # ─────────────────────────────────────────
    def angle_cb(self, msg):
        self.lane_angle = msg.data * -1.0
        self.vision_lista = True

    def state_cb(self, msg):
        if ':' in msg.data:
            nuevo_state, self.parking_side = msg.data.split(':')
        else:
            nuevo_state = msg.data

        if nuevo_state == 'STOP' and self.state != 'STOP':
            self.freno_stop_dist = False

        if nuevo_state != self.state:
            self._on_state_change(nuevo_state)

        if self.state != 'ORIENTANDOSE':
            self.state = nuevo_state

    def perception_cb(self, msg):
        self.lidar_rf = 'RF:False' in msg.data
        self.lidar_lf = 'LF:False' in msg.data

    def imu_cb(self, msg):
        # Asumimos que msg.vector.z ya es yaw en grados.
        # Si aún viene con signo opuesto al marco ROS, ajusta EXTERNAL_IMU_YAW_SIGN.
        raw_yaw = EXTERNAL_IMU_YAW_SIGN * msg.vector.z

        if self.yaw_initial_ext is None:
            self.yaw_initial_ext = raw_yaw

        self.yaw_ext = wrap_angle_deg(raw_yaw - self.yaw_initial_ext)
        self.ext_imu_stamp = time.time()

    def zed_yaw_cb(self, msg):
        self.zed_yaw = wrap_angle_deg(msg.data)
        self.zed_yaw_stamp = time.time()

    def zed_pose_cb(self, msg):
        self.zed_x = msg.pose.position.x
        self.zed_y = msg.pose.position.y
        self.zed_z = msg.pose.position.z
        self.zed_pose_available = True
        self.zed_pose_stamp = time.time()

    def zed_tracking_cb(self, msg):
        self.zed_tracking_ok = (msg.data == 'OK')

    # ─────────────────────────────────────────
    # UTILIDADES DE POSE / YAW
    # ─────────────────────────────────────────
    def _on_state_change(self, new_state):
        self.state = new_state
        self.state_entry_yaw = self.yaw if self._yaw_is_available() else None

        if self.zed_pose_available:
            self.state_entry_pose = (self.zed_x, self.zed_y, self.zed_z)
        else:
            self.state_entry_pose = None

        self.get_logger().info(
            f'Cambio de estado -> {new_state} | yaw={self.yaw:.2f} | pose={self.state_entry_pose}'
        )

    def _yaw_is_available(self):
        return self.yaw_source != "NONE"

    def _zed_yaw_fresh(self):
        return (time.time() - self.zed_yaw_stamp) < ZED_TIMEOUT_SEC

    def _ext_yaw_fresh(self):
        return (time.time() - self.ext_imu_stamp) < IMU_TIMEOUT_SEC

    def update_yaw_estimate(self):
        zed_ok = self.zed_tracking_ok and self.zed_yaw is not None and self._zed_yaw_fresh()
        ext_ok = self._ext_yaw_fresh()

        if zed_ok and ext_ok and USE_YAW_FUSION:
            self.yaw = blend_angles_deg(
                self.zed_yaw,
                self.yaw_ext,
                wa=ZED_YAW_WEIGHT,
                wb=EXT_YAW_WEIGHT
            )
            self.yaw_source = "FUSED"

        elif zed_ok and USE_ZED_YAW_AS_PRIMARY:
            self.yaw = self.zed_yaw
            self.yaw_source = "ZED"

        elif ext_ok:
            self.yaw = self.yaw_ext
            self.yaw_source = "EXT_IMU"

        elif zed_ok:
            self.yaw = self.zed_yaw
            self.yaw_source = "ZED"

        else:
            self.yaw_source = "NONE"

        if self.yaw_source != "NONE":
            msg = Float32()
            msg.data = float(self.yaw)
            self.yaw_debug_pub.publish(msg)

    def get_state_distance(self):
        if not self.zed_pose_available or self.state_entry_pose is None:
            return None

        dx = self.zed_x - self.state_entry_pose[0]
        dy = self.zed_y - self.state_entry_pose[1]
        dz = self.zed_z - self.state_entry_pose[2]
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)

        msg = Float32()
        msg.data = float(dist)
        self.pose_dist_pub.publish(msg)
        return dist

    def filter_range(self, value, prev):
        if value > 5.0:
            return prev
        if value <= 0.05:
            return 0.05
        return 0.3 * prev + 0.7 * value

    def read_ultrasonics_fast(self):
        if self.ser and self.ser.in_waiting > 0:
            try:
                raw_data = self.ser.read_all().decode('utf-8', errors='ignore')
                lines = raw_data.split('\n')

                for line in reversed(lines):
                    if ',' in line and len(line.split(',')) == 4:
                        parts = line.strip().split(',')

                        try:
                            new_ul = float(parts[0]) / 100.0
                            new_ub = float(parts[1]) / 100.0
                            new_ur = float(parts[2]) / 100.0
                            new_ir = int(parts[3])

                            self.ul = self.filter_range(new_ul, self.ul)
                            self.ur = self.filter_range(new_ur, self.ur)
                            self.ir_state = new_ir

                            if 0.05 < new_ub < 2.0:
                                self.ub = new_ub
                            return
                        except ValueError:
                            continue
            except Exception:
                pass

    def clamp_throttle(self, value):
        return max(PWM_THROTTLE_REV_MAX, min(int(value), PWM_THROTTLE_FWD_MAX))

    def clamp_steering(self, value):
        return max(PWM_STEER_LEFT_MAX, min(int(value), PWM_STEER_RIGHT_MAX))

    def actualizar_ir_activo(self):
        if not self.ir_activo:
            if self.state == 'ESTACIONANDO' and self.lidar_rf and self.lidar_lf:
                self.ir_activo = True

    # ─────────────────────────────────────────
    # LOOP PRINCIPAL
    # ─────────────────────────────────────────
    def loop(self):
        now = time.time()

        self.read_ultrasonics_fast()
        self.actualizar_ir_activo()
        self.update_yaw_estimate()

        # Esperar cámara de carril
        if not self.vision_lista:
            self._publicar(PWM_THROTTLE_NEUTRAL, PWM_STEER_CENTER)
            return

        if not self.camara_notificada:
            self.get_logger().info('¡CÁMARA ACTIVADA! Arrancando todo...')
            self.camara_notificada = True

        # Si no tenemos ningún yaw, mejor no mover
        if not self._yaw_is_available():
            self.get_logger().warn('Sin yaw válido (ni ZED ni IMU externa). Manteniendo neutro.')
            self._publicar(PWM_THROTTLE_NEUTRAL, PWM_STEER_CENTER)
            return

        if self.freno_emergencia or self.freno_stop_dist or self.freno_final_dist:
            self._publicar(PWM_THROTTLE_NEUTRAL, PWM_STEER_CENTER)
            return

        # Si acabamos de entrar a ENDEREZAR, fijamos target desde yaw actual
        if self.state == 'ENDEREZAR' and self.prev_state != 'ENDEREZAR':
            if self.parking_side == 'RIGHT':
                self.yaw_target = wrap_angle_deg(self.yaw_buscar - 90.0)
            else:
                self.yaw_target = wrap_angle_deg(self.yaw_buscar + 90.0)

        self.prev_state = self.state
        speed_cmd = PWM_THROTTLE_NEUTRAL
        steer_cmd = PWM_STEER_CENTER

        # Distancia recorrida en el estado actual
        state_distance = self.get_state_distance()

        # ─────────────────────────────────────
        # MAQUINA DE ESTADOS
        # ─────────────────────────────────────
        if self.state == 'ORIENTANDOSE':
            speed_cmd = ORIENT_SPEED

            if not self.vision_lista:
                steer_cmd = PWM_STEER_CENTER
            else:
                if self.angulo_deseado_actual is None:
                    if self.lane_angle > self.angulo_deseado_izq:
                        self.angulo_deseado_actual = self.angulo_deseado_izq
                    else:
                        self.angulo_deseado_actual = self.angulo_deseado_der

                error_angulo = self.angulo_deseado_actual - self.lane_angle

                if self.prev_error_angulo is None:
                    self.prev_error_angulo = error_angulo

                d_error = error_angulo - self.prev_error_angulo
                self.prev_error_angulo = error_angulo

                offset = (error_angulo * KP_ORIENT) + (d_error * KD_ORIENT)
                offset = max(-MAX_ANG_PID, min(offset, MAX_ANG_PID))
                steer_cmd = PWM_STEER_CENTER + offset

                if abs(error_angulo) < 1.5:
                    steer_cmd = PWM_STEER_CENTER
                    self.state = 'BUSCAR_CAJA'
                    self._on_state_change('BUSCAR_CAJA')
                    self.state_pub.publish(String(data='BUSCAR_CAJA'))

        elif self.state == 'BUSCAR_CAJA':
            speed_cmd = SEARCH_SPEED
            steer_cmd = PWM_STEER_CENTER

        elif self.state == 'BUSCAR':
            self.yaw_buscar = self.yaw
            speed_cmd = SEARCH_SPEED
            steer_cmd = PWM_STEER_CENTER

        elif self.state == 'ESPERAR':
            speed_cmd = PWM_THROTTLE_NEUTRAL
            steer_cmd = PWM_STEER_CENTER

        elif self.state == 'ALEJARSE':
            if self.parking_side == 'RIGHT':
                speed_cmd = ALEJARSE_SPEED
                steer_cmd = STEER_ALEJARSE_IZQ
            else:
                speed_cmd = ALEJARSE_SPEED_LEFT
                steer_cmd = STEER_ALEJARSE_DER

        elif self.state == 'ENTRAR_DIAGONAL':
            speed_cmd = PARKING_SPEED
            steer_cmd = STEER_DIAGONAL_DER if self.parking_side == 'RIGHT' else STEER_DIAGONAL_IZQ

        elif self.state == 'ENDEREZAR':
            speed_cmd = PARKING_SPEED
            yaw_target = self.yaw_target if self.yaw_target is not None else self.yaw

            yaw_err = angle_diff_deg(yaw_target, self.yaw)
            lat_err = (self.ul - self.ur) * 0.4 if self.parking_side == 'LEFT' else (self.ul - self.ur)

            offset = (KP_YAW * yaw_err) - (KP_LAT * lat_err)
            offset = max(-MAX_ANG_PID, min(offset, MAX_ANG_PID))
            steer_cmd = PWM_STEER_CENTER - offset

            if abs(yaw_err) < ALIGN_TOL:
                self.state_pub.publish(String(data='ENDEREZADO'))

        elif self.state == 'ESTACIONANDO':
            if not self.ajuste_adelante_done:
                if self.ajuste_adelante_time == 0.0:
                    self.ajuste_adelante_time = time.time()

                if time.time() - self.ajuste_adelante_time < AJUSTE_ADELANTE_TIME:
                    speed_cmd = AJUSTE_ADELANTE_SPEED
                    steer_cmd = PWM_STEER_CENTER
                else:
                    self.ajuste_adelante_done = True
            else:
                speed_cmd = PARKING_SPEED
                steer_cmd = PWM_STEER_CENTER

                condicion_ir = (self.ir_activo and self.ir_state == IR_STOP_VALUE)
                condicion_ub = (self.ub <= REAR_STOP_DIST)

                # Redundancia extra con pose:
                # si no avanzó nada en este estado, no la usamos para frenar;
                # si avanzó, al menos la publicamos y puedes tunearla después.
                if condicion_ir or condicion_ub:
                    self.freno_stop_dist = True
                    self.state_pub.publish(String(data='DIST_OK'))

        elif self.state == 'STOP':
            if self.stop_fase == 2:
                self.freno_final_dist = True
            elif self.stop_fase == 0:
                if self.stop_fase_time == 0.0:
                    self.stop_fase_time = time.time()

                if time.time() - self.stop_fase_time < 0.45:
                    speed_cmd = STOP_FASE_SPEED
                    steer_cmd = PWM_STEER_CENTER
                else:
                    self.stop_fase = 2

        # ─────────────────────────────────────
        # SISTEMA DE SUPERVIVENCIA
        # ─────────────────────────────────────
        if self.state in ['ENTRAR_DIAGONAL', 'ENDEREZAR']:
            if now > self.correcting_until_time:
                peligro_izq   = (self.parking_side == 'RIGHT' and self.ul <= LATERAL_DANGER_DIST)
                peligro_der   = (self.parking_side == 'LEFT'  and self.ur <= LATERAL_DANGER_DIST)
                peligro_atras = (self.ub <= REAR_DANGER_DIST)

                if peligro_izq or peligro_der or peligro_atras:
                    self.correcting_until_time = now + FORWARD_CORRECT_TIME + BACKWARD_CORRECT_TIME

            if now < self.correcting_until_time:
                tiempo_restante = self.correcting_until_time - now

                if tiempo_restante > BACKWARD_CORRECT_TIME:
                    speed_cmd = FORWARD_CORRECT_SPEED
                    steer_cmd = STEER_CORRECT_IZQ if self.parking_side == 'RIGHT' else STEER_CORRECT_DER
                else:
                    speed_cmd = BACKWARD_CORRECT_SPEED
                    steer_cmd = STEER_CORRECT_DER if self.parking_side == 'RIGHT' else STEER_CORRECT_IZQ

        # ─────────────────────────────────────
        # FRENO DE EMERGENCIA
        # ─────────────────────────────────────
        cond_emerg_ir = (self.ir_activo and self.ir_state == IR_STOP_VALUE)
        cond_emerg_ub = (self.ub <= REAR_STOP_DIST)

        if speed_cmd < PWM_THROTTLE_NEUTRAL and (cond_emerg_ir or cond_emerg_ub) and self.state not in ['ESTACIONANDO', 'STOP', 'ENDEREZAR']:
            self.freno_emergencia = True
            speed_cmd = PWM_THROTTLE_NEUTRAL
            steer_cmd = PWM_STEER_CENTER

        # debug cada cierto tiempo implícitamente por logger
        if state_distance is not None:
            self.get_logger().debug(
                f'state={self.state} | yaw={self.yaw:.2f} ({self.yaw_source}) | dist={state_distance:.3f} m'
            )

        self._publicar(speed_cmd, steer_cmd)

    # ─────────────────────────────────────────
    # PUBLICAR PWM
    # ─────────────────────────────────────────
    def _publicar(self, speed_cmd, steer_cmd):
        now = time.time()

        final_speed = self.clamp_throttle(speed_cmd)
        final_steer = self.clamp_steering(steer_cmd)

        # transición ESC para reversa
        if speed_cmd >= PWM_THROTTLE_NEUTRAL:
            self.reversa_armada = False
            self.esc_transicion_fase = 0

        if speed_cmd < PWM_THROTTLE_NEUTRAL and not self.reversa_armada and self.esc_transicion_fase == 0:
            self.esc_transicion_fase = 1
            self.esc_tiempo_fase = now

        if self.esc_transicion_fase == 1:
            final_speed = PWM_THROTTLE_REV_MAX
            if (now - self.esc_tiempo_fase) > 0.2:
                self.esc_transicion_fase = 2
                self.esc_tiempo_fase = now

        elif self.esc_transicion_fase == 2:
            final_speed = PWM_THROTTLE_NEUTRAL
            if (now - self.esc_tiempo_fase) > 0.3:
                self.esc_transicion_fase = 0
                self.reversa_armada = True

        msg_throttle = String()
        msg_throttle.data = str(final_speed)
        self.throttle_pwm_pub.publish(msg_throttle)

        msg_steering = String()
        msg_steering.data = str(final_steer)
        self.direction_pwm_pub.publish(msg_steering)


def main():
    rclpy.init()
    node = ParkingControllerTraxxas()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.throttle_pwm_pub.publish(String(data=str(PWM_THROTTLE_NEUTRAL)))
        node.direction_pwm_pub.publish(String(data=str(PWM_STEER_CENTER)))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()