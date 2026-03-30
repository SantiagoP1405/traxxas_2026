#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String, Float32
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

# --- PARÁMETROS DE ORIENTACIÓN INICIAL ---
ORIENT_SPEED     = 0.048   
ORIENT_STEER_VAL = -0.25    

# --- MANIOBRA DE ALEJAMIENTO (estado ALEJARSE) ---
ALEJARSE_SPEED      = 0.072
ALEJARSE_SPEED_LEFT = 0.072   
ALEJARSE_STEER      = 0.30

# --- CONSTANTES PID ENDEREZAR ---
KP_YAW      = 0.45
KP_LAT      = 15.0
MAX_ANG_PID = 0.30

ALIGN_TOL  = 0.08
CENTER_TOL = 0.10

# --- SENSOR INFRARROJO DE BANQUETA ---
IR_STOP_VALUE = 0
FINAL_SPEED   = -0.04

# --- MANIOBRA DE CORRECCIÓN LATERAL Y TRASERA (NUEVA SECUENCIA 2 PASOS) ---
LATERAL_DANGER_DIST    = 0.15
REAR_DANGER_DIST       = 0.13
FORWARD_CORRECT_TIME   = 0.70  # Fase 1: Tiempo yendo al frente
BACKWARD_CORRECT_TIME  = 0.37  # Fase 2: Tiempo yendo en reversa
FORWARD_CORRECT_SPEED  = 0.07
BACKWARD_CORRECT_SPEED = -0.07 # Reversa
FORWARD_CORRECT_STEER  = 0.35

# --- PARÁMETROS DE ENTRADA DIAGONAL ---
DIAGONAL_STEER = 0.42

# --- AJUSTE ADELANTE EN ESTACIONANDO ---
AJUSTE_ADELANTE_TIME = 0.1   


class ParkingController5(Node):
    def __init__(self):
        super().__init__('parking_controller5_qcar')

        self.state        = 'ORIENTANDOSE'
        self.parking_side = 'RIGHT'
        self.yaw          = 0.0
        self.yaw_initial  = None

        self.prev_state = None
        self.yaw_target = None
        self.yaw_buscar = 0.0
        
        self.lane_angle = 90.0  
        self.vision_lista = False   # <--- Candado para esperar a la cámara

        # --- NUEVOS PARÁMETROS DE ÁNGULOS DESEADOS ---
        self.angulo_deseado_der = 84.0
        self.angulo_deseado_izq = 75.0
        self.angulo_deseado_actual = None # Aquí guardaremos la decisión final

        # --- FLAGS DE FRENO PERMANENTE ---
        self.freno_emergencia = False
        self.freno_stop_dist  = False
        self.freno_final_dist = False

        self.correcting_until_time = 0.0

        # Sensores 
        self.ul       = 1.0
        self.ub       = 1.0
        self.ur       = 1.0
        self.ir_state = -1

        self.lidar_rf = False  
        self.lidar_lf = False  

        self.lidar_right_dist = 10.0
        self.lidar_left_dist  = 10.0

        self.ir_activo = False

        self.ajuste_adelante_done = False
        self.ajuste_adelante_time = 0.0

        self.stop_fase        = 0      
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
        self.create_subscription(String,  '/parking/state',         self.state_cb,     10)
        self.create_subscription(String,  '/parking/perception',    self.perception_cb, 10)
        self.create_subscription(Imu,     '/imu/data',              self.imu_cb,       10)
        self.create_subscription(Float32, '/qcar/lane_angle_ema',   self.angle_cb,      10)

        # --- PUBLICADORES ---
        self.cmd_pub   = self.create_publisher(Vector3Stamped, '/qcar/user_command',      10)
        self.state_pub = self.create_publisher(String,         '/parking/state_feedback', 10)

        self.timer = self.create_timer(0.05, self.loop)
        self.get_logger().info('Parking Controller iniciado: Lógica limpia V5 con log de ángulo')

    def angle_cb(self, msg):
        self.lane_angle = msg.data * -1.0
        self.vision_lista = True    # <--- Ya llegó el primer ángulo real

    def state_cb(self, msg):
        if ':' in msg.data:
            nuevo_state, self.parking_side = msg.data.split(':')
        else:
            nuevo_state = msg.data
            
        if nuevo_state == 'STOP' and self.state != 'STOP':
            self.freno_stop_dist = False
            self.get_logger().warn('Entrando a STOP — freno_stop_dist liberado')
            
        if self.state != 'ORIENTANDOSE':
            self.state = nuevo_state

    def perception_cb(self, msg):
        self.lidar_rf = 'RF0' in msg.data
        self.lidar_lf = 'LF0' in msg.data
        try:
            for token in msg.data.split():
                if token.startswith('RD'):
                    self.lidar_right_dist = float(token[2:])
                elif token.startswith('LD'):
                    self.lidar_left_dist = float(token[2:])
        except Exception:
            pass

    def imu_cb(self, msg):
        q       = msg.orientation
        raw_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        if self.yaw_initial is None:
            self.yaw_initial = raw_yaw
        self.yaw = raw_yaw - self.yaw_initial

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

    def clamp(self, value, limit):
        return max(-limit, min(value, limit))

    def actualizar_ir_activo(self):
        if not self.ir_activo:
            if self.state == 'ESTACIONANDO' and self.lidar_rf and self.lidar_lf:
                self.ir_activo = True
                self.get_logger().warn('IR activado — lidar confirmó cajas')

    def loop(self):
        now = time.time()
        self.read_ultrasonics_fast()
        self.actualizar_ir_activo()

        if self.freno_emergencia or self.freno_stop_dist or self.freno_final_dist:
            self._publicar(0.0, 0.0)
            return

        if self.state == 'ENDEREZAR' and self.prev_state != 'ENDEREZAR':
            if self.parking_side == 'RIGHT':
                self.yaw_target = self.yaw_buscar - (math.pi / 2)
            else:
                self.yaw_target = self.yaw_buscar + (math.pi / 2)

        self.prev_state = self.state

        speed_cmd = 0.0
        steer_cmd = 0.0

        # ==============================================================
        # MAQUINA DE ESTADOS
        # ==============================================================

        if self.state == 'ORIENTANDOSE':
            speed_cmd = ORIENT_SPEED

            if not self.vision_lista:
                steer_cmd = 0.0
            else:
                if self.angulo_deseado_actual is None:
                    if self.lane_angle > 90.0:
                        self.angulo_deseado_actual = self.angulo_deseado_der
                        self.get_logger().info(f'--- DECISIÓN DE RUTA: Ángulo inicial {self.lane_angle:.1f}° > 90. Target seteado en: {self.angulo_deseado_actual}° ---')
                    else:
                        self.angulo_deseado_actual = self.angulo_deseado_izq
                        self.get_logger().info(f'--- DECISIÓN DE RUTA: Ángulo inicial {self.lane_angle:.1f}° <= 90. Target seteado en: {self.angulo_deseado_actual}° ---')

                error_angulo = self.angulo_deseado_actual - self.lane_angle
                KP_ORIENT = 0.015 
                
                steer_cmd = error_angulo * KP_ORIENT
                steer_cmd = self.clamp(steer_cmd, MAX_STEER_LIMIT)

                if abs(error_angulo) < 3.0:
                    steer_cmd = 0.0
                    self.get_logger().warn(f'¡Alineado en carril! Ángulo final: {self.lane_angle:.2f}°. Cambiando a BUSCAR_CAJA...')
                    self.state = 'BUSCAR_CAJA'
                    self.state_pub.publish(String(data='BUSCAR_CAJA'))

        elif self.state == 'BUSCAR_CAJA':
            speed_cmd = SEARCH_SPEED
            steer_cmd = 0.0

        elif self.state == 'BUSCAR':
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

        elif self.state == 'ENDEREZAR':
            speed_cmd = PARKING_SPEED

            yaw_target = self.yaw_target if self.yaw_target else self.yaw
            yaw_err    = math.atan2(math.sin(yaw_target - self.yaw), math.cos(yaw_target - self.yaw))
            
            if self.parking_side == 'LEFT':
                lat_err = (self.ul - self.ur) * 0.4
            else:
                lat_err = self.ul - self.ur

            ang       = KP_YAW * yaw_err - KP_LAT * lat_err
            steer_cmd = self.clamp(ang, MAX_ANG_PID)

            if abs(yaw_err) < ALIGN_TOL: 
                self.state_pub.publish(String(data='ENDEREZADO'))

        elif self.state == 'ESTACIONANDO':
            if not self.ajuste_adelante_done:
                if self.ajuste_adelante_time == 0.0:
                    self.ajuste_adelante_time = time.time()
                if time.time() - self.ajuste_adelante_time < AJUSTE_ADELANTE_TIME:
                    speed_cmd = 0.05   
                    steer_cmd = 0.0
                else:
                    self.ajuste_adelante_done = True
            else:
                speed_cmd = PARKING_SPEED
                if self.ir_activo and self.ir_state == IR_STOP_VALUE:
                    self.freno_stop_dist = True
                    self.state_pub.publish(String(data='DIST_OK'))

        elif self.state == 'STOP':
            if self.stop_fase == 2:
                self.freno_final_dist = True
            elif self.stop_fase == 0:
                if self.stop_fase_time == 0.0:
                    self.stop_fase_time = time.time()
                if time.time() - self.stop_fase_time < 0.45:
                    speed_cmd = 0.07
                    steer_cmd = 0.0
                else:
                    self.stop_fase = 2

        # ==============================================================
        # SISTEMA DE SUPERVIVENCIA LATERAL Y TRASERO (SECUENCIA 2 PASOS)
        # ==============================================================
        if self.state in ['ENTRAR_DIAGONAL', 'ENDEREZAR']:
            # 1. Si detecta peligro, asigna el tiempo de TODO el combo (Adelante + Atras)
            if now > self.correcting_until_time:
                peligro_izq   = (self.parking_side == 'RIGHT' and self.ul <= LATERAL_DANGER_DIST)
                peligro_der   = (self.parking_side == 'LEFT'  and self.ur <= LATERAL_DANGER_DIST)
                peligro_atras = (self.ub <= REAR_DANGER_DIST)

                if peligro_izq or peligro_der or peligro_atras:
                    tiempo_total = FORWARD_CORRECT_TIME + BACKWARD_CORRECT_TIME
                    self.correcting_until_time = now + tiempo_total
                    self.get_logger().warn('¡PELIGRO! Iniciando supervivencia: Adelante y luego Atrás')

            # 2. Si estamos dentro de la maniobra de supervivencia, pisamos los comandos
            if now < self.correcting_until_time:
                tiempo_restante = self.correcting_until_time - now
                
                # FASE 1: Ir hacia adelante (Si aún falta tiempo para pasar a la fase 2)
                if tiempo_restante > BACKWARD_CORRECT_TIME:
                    speed_cmd = FORWARD_CORRECT_SPEED
                    steer_cmd = -FORWARD_CORRECT_STEER if self.parking_side == 'RIGHT' else FORWARD_CORRECT_STEER
                
                # FASE 2: Ir hacia atrás con volante invertido
                else:
                    speed_cmd = BACKWARD_CORRECT_SPEED
                    steer_cmd = FORWARD_CORRECT_STEER if self.parking_side == 'RIGHT' else -FORWARD_CORRECT_STEER

        if speed_cmd < 0.0 and self.ir_activo and self.ir_state == IR_STOP_VALUE and self.state not in ['ESTACIONANDO', 'STOP']:
            self.freno_emergencia = True
            speed_cmd = 0.0
            steer_cmd = 0.0

        self._publicar(speed_cmd, steer_cmd)

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

def main():
    rclpy.init()
    node = ParkingController5()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Vector3Stamped()
        stop.header.stamp = node.get_clock().now().to_msg()
        stop.vector.x = 0.0
        stop.vector.y = STEERING_OFFSET 
        stop.vector.z = 0.0
        node.cmd_pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
