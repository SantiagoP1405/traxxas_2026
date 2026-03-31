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
BAUD_RATE = 115200

# --- LIMITES DE SEGURIDAD FÍSICA QCAR ---
MAX_SPEED_LIMIT = 0.10  
MAX_STEER_LIMIT = 0.5   

# --- CALIBRACIÓN MECÁNICA (TRIM) ---
STEERING_OFFSET = -0.0812

# --- VELOCIDADES (LENTO PERO SEGURO) ---
SEARCH_SPEED  = 0.085
PARKING_SPEED = -0.070

# --- CONSTANTES DE TU SIMULACION ---
KP_YAW = 0.4 
KP_LAT = 15.0 
MAX_ANG_PID = 0.30 

ALIGN_TOL = 0.08
CENTER_TOL = 0.10

# --- NUEVO PARÁMETRO DE BANQUETA INFRARROJA ---
# Cambia esto a 1 si tu sensor funciona al revés (1=Blanco, 0=Negro)
IR_STOP_VALUE = 0 
FINAL_SPEED = -0.05

# --- CORRECCIÓN DIAGONAL (SUAVE) ---
DIAG_CORRECTION_THRESH = 0.05
DIAG_CORRECTION_GAIN   = 0.08

# --- MANIOBRA DE CORRECCIÓN HACIA ADELANTE ---
LATERAL_DANGER_DIST = 0.15 
FORWARD_CORRECT_TIME = 0.65 
FORWARD_CORRECT_SPEED = 0.08 
FORWARD_CORRECT_STEER = 0.35 

class ParkingController3(Node):
    def __init__(self):
        super().__init__('parking_controller3_qcar')

        self.state = 'BUSCAR'
        self.parking_side = 'RIGHT'   
        self.yaw = 0.0
        self.yaw_initial = None
        
        self.prev_state = None 
        self.yaw_target = None

        self.yaw_buscar = 0.0

        # --- FLAGS DE FRENO PERMANENTE ---
        self.freno_emergencia = False
        self.freno_stop_dist  = False
        self.freno_final_dist = False

        self.correcting_until_time = 0.0

        # Sensores (en METROS) y estado del Infrarrojo
        self.ul = 1.0
        self.ub = 1.0
        self.ur = 1.0
        self.ir_state = -1 # Estado inicial del infrarrojo

        # --- CONEXION ARDUINO ---
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
            self.get_logger().info(f'--- CONECTADO A ARDUINO ({SERIAL_PORT}) ---')
            time.sleep(2)
            self.ser.reset_input_buffer()
        except Exception as e:
            self.get_logger().error(f'ERROR ARDUINO: {e}')
            self.ser = None

        # --- SUSCRIPCIONES QCAR ---
        self.create_subscription(String, '/parking/state', self.state_cb, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_cb, 10)

        # --- PUBLICADORES QCAR ---
        self.cmd_pub = self.create_publisher(Vector3Stamped, '/qcar/user_command', 10)
        self.state_pub = self.create_publisher(String, '/parking/state_feedback', 10)

        self.timer = self.create_timer(0.05, self.loop)
        self.get_logger().info('Parking Controller 3 QCar Iniciado (Freno por Infrarrojo)')

    def state_cb(self, msg):
        if ':' in msg.data:
            self.state, self.parking_side = msg.data.split(':')
        else:
            self.state = msg.data

    def imu_cb(self, msg):
        q = msg.orientation
        raw_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        if self.yaw_initial is None:
            self.yaw_initial = raw_yaw
            self.get_logger().info(f'YAW inicial capturado: {math.degrees(raw_yaw):.2f}°')
        self.yaw = raw_yaw - self.yaw_initial

    def filter_range(self, value, prev):
        if value <= 0.05 or value > 5.0:
            return prev
        return 0.3 * prev + 0.7 * value

    def read_ultrasonics_fast(self):
        if self.ser and self.ser.in_waiting > 0:
            try:
                raw_data = self.ser.read_all().decode('utf-8', errors='ignore')
                lines = raw_data.split('\n')
                for line in reversed(lines):
                    # AHORA ESPERAMOS 4 VALORES
                    if ',' in line and len(line.split(',')) == 4:
                        parts = line.strip().split(',')
                        try:
                            new_ul = float(parts[0]) / 100.0
                            new_ub = float(parts[1]) / 100.0
                            new_ur = float(parts[2]) / 100.0
                            new_ir = int(parts[3]) # Leemos el infrarrojo

                            self.ul = self.filter_range(new_ul, self.ul)
                            self.ur = self.filter_range(new_ur, self.ur)
                            
                            if 0.05 < new_ub < 2.0:
                                self.ub = new_ub
                                
                            self.ir_state = new_ir # Guardamos estado IR

                            return
                        except ValueError:
                            continue
            except Exception:
                pass

    def clamp(self, value, limit):
        return max(-limit, min(value, limit))

    def loop(self):
        now = time.time()
        self.read_ultrasonics_fast()

        # --- SI CUALQUIER FRENO PERMANENTE ESTÁ ACTIVO, PARAR TODO ---
        if self.freno_emergencia or self.freno_stop_dist or self.freno_final_dist:
            if self.freno_emergencia:
                self.get_logger().error('Vehículo bloqueado: FRENO EMERGENCIA (Banqueta detectada)')
            elif self.freno_stop_dist:
                self.get_logger().warn('Vehículo bloqueado: ESTACIONADO (Banqueta detectada)')
            elif self.freno_final_dist:
                self.get_logger().warn('Vehículo bloqueado: FINAL (Banqueta detectada)')

            self._publicar(0.0, 0.0)
            return

        if self.state == 'ENDEREZAR' and self.prev_state != 'ENDEREZAR':
            self.yaw_target = self.yaw_buscar - (math.pi / 2)
            self.get_logger().warn(
                f'# debug yaw | yaw_buscar={math.degrees(self.yaw_buscar):.2f}° '
                f'yaw_actual={math.degrees(self.yaw):.2f}° '
                f'yaw_target={math.degrees(self.yaw_target):.2f}°'
            )

        self.prev_state = self.state

        speed_cmd = 0.0
        steer_cmd = 0.0

        if self.state == 'BUSCAR':
            self.yaw_buscar = self.yaw
            speed_cmd = SEARCH_SPEED
            steer_cmd = 0.0

        elif self.state == 'ESPERAR':
            speed_cmd = 0.0
            steer_cmd = 0.0

        elif self.state == 'ENTRAR_DIAGONAL':
            speed_cmd = PARKING_SPEED
            base_steer = 0.38 if self.parking_side == 'RIGHT' else -0.38

            lat_diff = self.ul - self.ur
            if lat_diff > DIAG_CORRECTION_THRESH:
                correction = +DIAG_CORRECTION_GAIN 
            elif lat_diff < -DIAG_CORRECTION_THRESH:
                correction = -DIAG_CORRECTION_GAIN 
            else:
                correction = 0.0

            steer_cmd = self.clamp(base_steer + correction, MAX_STEER_LIMIT)

        elif self.state == 'ENDEREZAR':
            speed_cmd = PARKING_SPEED

            yaw_target = self.yaw_target if self.yaw_target else self.yaw
            yaw_err = math.atan2(math.sin(yaw_target - self.yaw), math.cos(yaw_target - self.yaw))
            lat_err = self.ul - self.ur

            ang = KP_YAW * yaw_err - KP_LAT * lat_err
            steer_cmd = self.clamp(ang, MAX_ANG_PID)

            if abs(yaw_err) < ALIGN_TOL and abs(lat_err) < CENTER_TOL:
                self.state_pub.publish(String(data='ENDEREZADO'))

        elif self.state == 'ESTACIONANDO':
            speed_cmd = PARKING_SPEED
            
            # --- FRENO POR INFRARROJO EN VEZ DE ULTRASONICO ---
            if self.ir_state == IR_STOP_VALUE:
                self.get_logger().warn('Línea de banqueta detectada — freno permanente')
                self.freno_stop_dist = True
                self.state_pub.publish(String(data='DIST_OK'))

        elif self.state == 'STOP':
            # --- AJUSTE FINAL POR INFRARROJO ---
            if self.ir_state != IR_STOP_VALUE:
                speed_cmd = FINAL_SPEED
                steer_cmd = 0.0
            else:
                self.get_logger().warn('Línea de banqueta (FINAL) detectada — freno permanente')
                self.freno_final_dist = True

        # ==========================================================
        # SISTEMA DE SUPERVIVENCIA LATERAL (Adelante + Giro)
        # ==========================================================
        if self.state in ['ENTRAR_DIAGONAL', 'ENDEREZAR'] and speed_cmd < 0.0:
            
            if now > self.correcting_until_time:
                if self.parking_side == 'RIGHT' and self.ul <= LATERAL_DANGER_DIST:
                    self.get_logger().warn(f'¡Peligro Izq ({self.ul:.2f}m)! Avanzando y girando IZQ...')
                    self.correcting_until_time = now + FORWARD_CORRECT_TIME

                elif self.parking_side == 'LEFT' and self.ur <= LATERAL_DANGER_DIST:
                    self.get_logger().warn(f'¡Peligro Der ({self.ur:.2f}m)! Avanzando y girando DER...')
                    self.correcting_until_time = now + FORWARD_CORRECT_TIME

        if now < self.correcting_until_time:
            speed_cmd = FORWARD_CORRECT_SPEED
            if self.parking_side == 'RIGHT':
                steer_cmd = -FORWARD_CORRECT_STEER
            else:
                steer_cmd = FORWARD_CORRECT_STEER

        # ==========================================================
        # FRENO DE EMERGENCIA TRASERO (AHORA CON INFRARROJO)
        # ==========================================================
        if speed_cmd < 0.0 and self.ir_state == IR_STOP_VALUE:
            self.get_logger().error('¡FRENO EMERGENCIA PERMANENTE! Banqueta tocada')
            self.freno_emergencia = True
            speed_cmd = 0.0
            steer_cmd = 0.0

        self._publicar(speed_cmd, steer_cmd)

    def _publicar(self, speed_cmd, steer_cmd):
        final_speed = self.clamp(speed_cmd, MAX_SPEED_LIMIT)
        logical_steer = self.clamp(-steer_cmd, MAX_STEER_LIMIT)
        hardware_steer = self.clamp(logical_steer + STEERING_OFFSET, MAX_STEER_LIMIT)

        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'command_input'
        msg.vector.x = float(final_speed)
        msg.vector.y = float(hardware_steer)
        msg.vector.z = 0.0

        self.cmd_pub.publish(msg)

def main():
    rclpy.init()
    node = ParkingController3()
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