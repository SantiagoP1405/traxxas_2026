#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from geometry_msgs.msg import Vector3Stamped
import math
import serial
import time

# --- CONFIGURACION DE HARDWARE ---
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE   = 115200

# --- LIMITES DE SEGURIDAD FÍSICA QCAR ---
MAX_SPEED_LIMIT = 0.10
MAX_STEER_LIMIT = 0.5

# --- CALIBRACIÓN MECÁNICA (TRIM) ---
STEERING_OFFSET = -0.0812

# --- VELOCIDADES ---
SEARCH_SPEED  =  0.085
PARKING_SPEED = -0.06

# --- MANIOBRA DE ALEJAMIENTO (estado ALEJARSE) ---
ALEJARSE_SPEED      = 0.072
ALEJARSE_SPEED_LEFT = 0.072   # más lento para compensar el giro más cerrado
ALEJARSE_STEER      = 0.30

# --- CONSTANTES PID ENDEREZAR ---
KP_YAW      = 0.4
KP_LAT      = 15.0
MAX_ANG_PID = 0.30

ALIGN_TOL  = 0.08
CENTER_TOL = 0.10

# --- SENSOR INFRARROJO DE BANQUETA ---
IR_STOP_VALUE = 0
FINAL_SPEED   = -0.04

# --- MANIOBRA DE CORRECCIÓN LATERAL Y TRASERA (supervivencia con timer estilo V2) ---
LATERAL_DANGER_DIST   = 0.15
REAR_DANGER_DIST      = 0.15
FORWARD_CORRECT_TIME  = 0.65
FORWARD_CORRECT_SPEED = 0.08
FORWARD_CORRECT_STEER = 0.35

# --- PARÁMETROS DE ENTRADA DIAGONAL ---
DIAGONAL_STEER = 0.42

# --- AJUSTE ADELANTE EN ESTACIONANDO ---
AJUSTE_ADELANTE_TIME = 0.1   # segundos que avanza antes de entrar al espacio


class ParkingController4(Node):
    def __init__(self):
        super().__init__('parking_controller4_qcar')

        self.state        = 'BUSCAR'
        self.parking_side = 'RIGHT'
        self.yaw          = 0.0
        self.yaw_initial  = None

        self.prev_state = None
        self.yaw_target = None
        self.yaw_buscar = 0.0

        # --- FLAGS DE FRENO PERMANENTE ---
        self.freno_emergencia = False
        self.freno_stop_dist  = False
        self.freno_final_dist = False

        # --- FLAG PARA LA MANIOBRA DE CORRECCIÓN (Estilo V2) ---
        self.correcting_until_time = 0.0

        # Sensores (en METROS) y estado del infrarrojo
        self.ul       = 1.0
        self.ub       = 1.0
        self.ur       = 1.0
        self.ir_state = -1

        # --- PERCEPCIÓN LIDAR (para validar IR en ESTACIONANDO) ---
        self.lidar_rf = False  # True = lado derecho ocupado (caja presente)
        self.lidar_lf = False  # True = lado izquierdo ocupado (caja presente)

        # #LIDARMOD distancias laterales del lidar
        self.lidar_right_dist = 10.0
        self.lidar_left_dist  = 10.0

        # Una vez activado, el IR se checa solo sin depender del lidar
        self.ir_activo = False

        # Flag para ajuste adelante en ESTACIONANDO
        self.ajuste_adelante_done = False
        self.ajuste_adelante_time = 0.0

        # Flags para maniobra final en STOP
        self.stop_fase        = 0      # 0=esperando, 1=avanzando, 2=frenado
        self.stop_fase_time   = 0.0

        # --- CONEXION ARDUINO ---
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
            self.get_logger().info(f'--- CONECTADO A ARDUINO ({SERIAL_PORT}) ---')
            time.sleep(2)
            self.ser.reset_input_buffer()
        except Exception as e:
            self.get_logger().error(f'ERROR ARDUINO: {e}')
            self.ser = None

        # --- SUSCRIPCIONES ---
        self.create_subscription(String, '/parking/state',      self.state_cb,     10)
        self.create_subscription(String, '/parking/perception', self.perception_cb, 10)
        self.create_subscription(Imu,    '/imu/data',           self.imu_cb,        10)

        # --- PUBLICADORES ---
        self.cmd_pub   = self.create_publisher(Vector3Stamped, '/qcar/user_command',      10)
        self.state_pub = self.create_publisher(String,         '/parking/state_feedback', 10)

        self.timer = self.create_timer(0.05, self.loop)
        self.get_logger().info('Parking Controller 4 QCar iniciado (Supervivencia + Ignorar IR por Estado)')

    # ------------------------------------------------------------------
    def state_cb(self, msg):
        if ':' in msg.data:
            nuevo_state, self.parking_side = msg.data.split(':')
        else:
            nuevo_state = msg.data
        # Al entrar a STOP, liberar el freno de ESTACIONANDO para poder moverse
        if nuevo_state == 'STOP' and self.state != 'STOP':
            self.freno_stop_dist = False
            self.get_logger().warn('Entrando a STOP — freno_stop_dist liberado')
        self.state = nuevo_state

    # ------------------------------------------------------------------
    def perception_cb(self, msg):
        # Guardamos si el lidar ve cajas a los lados (RF0 = caja derecha, LF0 = caja izquierda)
        self.lidar_rf = 'RF0' in msg.data
        self.lidar_lf = 'LF0' in msg.data
        # #LIDARMOD parsear distancias laterales reales
        try:
            for token in msg.data.split():
                if token.startswith('RD'):
                    self.lidar_right_dist = float(token[2:])
                elif token.startswith('LD'):
                    self.lidar_left_dist = float(token[2:])
        except Exception:
            pass

    # ------------------------------------------------------------------
    def imu_cb(self, msg):
        q       = msg.orientation
        raw_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        if self.yaw_initial is None:
            self.yaw_initial = raw_yaw
            self.get_logger().info(f'YAW inicial capturado: {math.degrees(raw_yaw):.2f}°')
        self.yaw = raw_yaw - self.yaw_initial

    # ------------------------------------------------------------------
    def filter_range(self, value, prev):
        if value > 5.0:
            return prev
        if value <= 0.05:
            return 0.05
        return 0.3 * prev + 0.7 * value

    # ------------------------------------------------------------------
    def read_ultrasonics_fast(self):
        if self.ser and self.ser.in_waiting > 0:
            try:
                raw_data = self.ser.read_all().decode('utf-8', errors='ignore')
                lines    = raw_data.split('\n')
                for line in reversed(lines):
                    if ',' in line and len(line.split(',')) == 4:
                        parts = line.strip().split(',')
                        try:
                            new_ul = float(parts[0]) / 100.0
                            new_ub = float(parts[1]) / 100.0
                            new_ur = float(parts[2]) / 100.0
                            new_ir = int(parts[3])

                            self.ul       = self.filter_range(new_ul, self.ul)
                            self.ur       = self.filter_range(new_ur, self.ur)
                            self.ir_state = new_ir

                            if 0.05 < new_ub < 2.0:
                                self.ub = new_ub

                            return
                        except ValueError:
                            continue
            except Exception:
                pass

    # ------------------------------------------------------------------
    def clamp(self, value, limit):
        return max(-limit, min(value, limit))

    # ------------------------------------------------------------------
    def actualizar_ir_activo(self):
        # Se activa una sola vez cuando estamos en ESTACIONANDO
        # y el lidar confirma cajas a ambos lados.
        # Una vez activo, ya no se apaga — el IR se checa solo.
        if not self.ir_activo:
            if self.state == 'ESTACIONANDO' and self.lidar_rf and self.lidar_lf:
                self.ir_activo = True
                self.get_logger().warn('IR activado — lidar confirmó cajas a ambos lados')

    # ------------------------------------------------------------------
    def loop(self):
        now = time.time()
        self.read_ultrasonics_fast()

        self.actualizar_ir_activo()

        # --- FRENOS PERMANENTES ---
        if self.freno_emergencia or self.freno_stop_dist or self.freno_final_dist:
            if self.freno_emergencia:
                self.get_logger().error('Vehículo bloqueado: FRENO EMERGENCIA (barrera)')
            elif self.freno_stop_dist:
                self.get_logger().warn('Vehículo bloqueado: ESTACIONADO (barrera)')
            elif self.freno_final_dist:
                self.get_logger().warn('Vehículo bloqueado: FINAL (barrera)')
            self._publicar(0.0, 0.0)
            return

        # Capturar yaw_target al entrar a ENDEREZAR
        if self.state == 'ENDEREZAR' and self.prev_state != 'ENDEREZAR':
            if self.parking_side == 'RIGHT':
                self.yaw_target = self.yaw_buscar - (math.pi / 2)
            else:
                self.yaw_target = self.yaw_buscar + (math.pi / 2)
            self.get_logger().warn(
                f'# debug yaw | yaw_buscar={math.degrees(self.yaw_buscar):.2f}° '
                f'yaw_actual={math.degrees(self.yaw):.2f}° '
                f'yaw_target={math.degrees(self.yaw_target):.2f}°'
            )

        self.prev_state = self.state

        speed_cmd = 0.0
        steer_cmd = 0.0

        # ==============================================================
        # MAQUINA DE ESTADOS
        # ==============================================================

        if self.state == 'BUSCAR':
            self.yaw_buscar = self.yaw
            speed_cmd = SEARCH_SPEED
            steer_cmd = 0.0

        elif self.state == 'ESPERAR':
            speed_cmd = 0.0
            steer_cmd = 0.0

        elif self.state == 'ALEJARSE':
            if self.parking_side == 'RIGHT':
                speed_cmd = ALEJARSE_SPEED
                steer_cmd = -ALEJARSE_STEER
            else:
                speed_cmd = ALEJARSE_SPEED_LEFT
                steer_cmd = ALEJARSE_STEER

        elif self.state == 'ENTRAR_DIAGONAL':
            speed_cmd = PARKING_SPEED
            steer_cmd = DIAGONAL_STEER if self.parking_side == 'RIGHT' else -DIAGONAL_STEER

            self.get_logger().info(
                f'DIAG | ul={self.ul:.2f} ur={self.ur:.2f} steer_fijo={steer_cmd:.3f}'
            )

        elif self.state == 'ENDEREZAR':
            speed_cmd = PARKING_SPEED

            yaw_target = self.yaw_target if self.yaw_target else self.yaw
            yaw_err    = math.atan2(
                math.sin(yaw_target - self.yaw),
                math.cos(yaw_target - self.yaw)
            )
            #lat_err   = self.ul - self.ur
            if self.parking_side == 'LEFT':
                lat_err = (self.ul - self.ur) * 0.4  # menos agresivo del lado izquierdo
            else:
                lat_err = self.ul - self.ur

            ang       = KP_YAW * yaw_err - KP_LAT * lat_err
            steer_cmd = self.clamp(ang, MAX_ANG_PID)

            self.get_logger().info(
                f'ENDEREZAR | ul={self.ul:.2f} ur={self.ur:.2f} steer={steer_cmd:.2f}'
            )

            if abs(yaw_err) < ALIGN_TOL: #and abs(lat_err) < CENTER_TOL:
                self.state_pub.publish(String(data='ENDEREZADO'))

        elif self.state == 'ESTACIONANDO':

            # Pequeño ajuste hacia adelante antes de entrar al espacio
            if not self.ajuste_adelante_done:
                if self.ajuste_adelante_time == 0.0:
                    self.ajuste_adelante_time = time.time()
                if time.time() - self.ajuste_adelante_time < AJUSTE_ADELANTE_TIME:
                    speed_cmd = 0.05   # avanza recto
                    steer_cmd = 0.0
                else:
                    self.ajuste_adelante_done = True
            else:
                # Ya hizo el ajuste, ahora solo le hace caso al IR
                speed_cmd = PARKING_SPEED
                if self.ir_activo and self.ir_state == IR_STOP_VALUE:
                    self.get_logger().warn(
                        f'Línea de fondo detectada (lidar_rf={self.lidar_rf} lidar_lf={self.lidar_lf}) — freno permanente'
                    )
                    self.freno_stop_dist = True
                    self.state_pub.publish(String(data='DIST_OK'))

        elif self.state == 'STOP':
            self.get_logger().info(f'STOP | fase={self.stop_fase} ir={self.ir_state} fase_time={self.stop_fase_time:.2f}')
            if self.stop_fase == 2:
                # Fase final: frenado permanente
                self.freno_final_dist = True
            elif self.stop_fase == 0:
                # Arranca inmediatamente hacia adelante
                if self.stop_fase_time == 0.0:
                    self.stop_fase_time = time.time()
                    self.get_logger().warn('STOP — avanzando 0.3s hacia adelante')
                if time.time() - self.stop_fase_time < 0.45:
                    speed_cmd = 0.07
                    steer_cmd = 0.0
                else:
                    self.stop_fase = 2

        # ==============================================================
        # SISTEMA DE SUPERVIVENCIA LATERAL Y TRASERO (Estilo V2 con Timer)
        # ==============================================================
        if self.state in ['ENTRAR_DIAGONAL', 'ENDEREZAR']:

            if now > self.correcting_until_time:
                peligro_izq   = (self.parking_side == 'RIGHT' and self.ul <= LATERAL_DANGER_DIST)
                peligro_der   = (self.parking_side == 'LEFT'  and self.ur <= LATERAL_DANGER_DIST)
                peligro_atras = (self.ub <= REAR_DANGER_DIST)

                if peligro_izq or peligro_der or peligro_atras:
                    if peligro_atras:
                        lado = "Atrás"
                        dist = self.ub
                    else:
                        lado = "Izq" if peligro_izq else "Der"
                        dist = self.ul if peligro_izq else self.ur

                    self.get_logger().warn(
                        f'¡Peligro {lado} ({dist:.2f}m)! Iniciando corrección de {FORWARD_CORRECT_TIME}s...'
                    )
                    self.correcting_until_time = now + FORWARD_CORRECT_TIME

            if now < self.correcting_until_time:
                speed_cmd = FORWARD_CORRECT_SPEED
                steer_cmd = -FORWARD_CORRECT_STEER if self.parking_side == 'RIGHT' else FORWARD_CORRECT_STEER

        # ==============================================================
        # FRENO DE EMERGENCIA TRASERO (infrarrojo)
        # Solo activo en ESTACIONANDO cuando lidar confirma cajas a ambos lados
        # ==============================================================
        if speed_cmd < 0.0 and self.ir_activo and self.ir_state == IR_STOP_VALUE and self.state not in ['ESTACIONANDO', 'STOP']:
            self.get_logger().error('¡FRENO EMERGENCIA PERMANENTE! Barrera tocada')
            self.freno_emergencia = True
            speed_cmd = 0.0
            steer_cmd = 0.0

        self._publicar(speed_cmd, steer_cmd)

    # ------------------------------------------------------------------
    def _publicar(self, speed_cmd, steer_cmd):
        final_speed    = self.clamp(speed_cmd, MAX_SPEED_LIMIT)
        logical_steer  = self.clamp(-steer_cmd, MAX_STEER_LIMIT)
        hardware_steer = self.clamp(logical_steer + STEERING_OFFSET, MAX_STEER_LIMIT)

        msg = Vector3Stamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'command_input'
        msg.vector.x        = float(final_speed)
        msg.vector.y        = float(hardware_steer)
        msg.vector.z        = 0.0

        self.cmd_pub.publish(msg)


# ----------------------------------------------------------------------
def main():
    rclpy.init()
    node = ParkingController4()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Vector3Stamped()
        stop.header.stamp = node.get_clock().now().to_msg()
        node.cmd_pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
