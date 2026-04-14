#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Vector3Stamped 
from std_msgs.msg import String, Float32
import math
import serial
import time

# --- CONFIGURACION DE HARDWARE ---
SERIAL_PORT = '/dev/ttyUSB1'  
BAUD_RATE   = 115200

# --- LIMITES DE SEGURIDAD HARDWARE PWM ---
PWM_THROTTLE_NEUTRAL = 2457
PWM_THROTTLE_FWD_MAX = 2800  
PWM_THROTTLE_REV_MAX = 2150  

# ¡CENTRO FÍSICO AJUSTADO!
PWM_STEER_CENTER     = 2525  
PWM_STEER_LEFT_MAX   = 1669
PWM_STEER_RIGHT_MAX  = 3276

# --- VELOCIDADES PWM ---
SEARCH_SPEED  = 2685    
PARKING_SPEED = 2240    

ORIENT_SPEED           = 2680   
ALEJARSE_SPEED         = 2695   
ALEJARSE_SPEED_LEFT    = 2695   
FORWARD_CORRECT_SPEED  = 2690   
BACKWARD_CORRECT_SPEED = 2250   
AJUSTE_ADELANTE_SPEED  = 2680   
STOP_FASE_SPEED        = 2600   

# --- DIRECCIONES PWM LÓGICAS ---
STEER_ALEJARSE_DER    = 3250   
STEER_ALEJARSE_IZQ    = 1950   
STEER_DIAGONAL_DER    = 3250   
STEER_DIAGONAL_IZQ    = 2000   
STEER_CORRECT_DER     = 3200   
STEER_CORRECT_IZQ     = 2000   

# --- CONSTANTES PID ---
KP_ORIENT   = 20.0   # Fuerza de corrección a la cámara
KD_ORIENT   = 12.0   # Amortiguación para evitar oscilaciones
KP_YAW      = 3.5    
KP_LAT      = 4520.0 
MAX_ANG_PID = 355    

ALIGN_TOL  = 4.5     
CENTER_TOL = 0.10

# --- SENSORES ---
IR_STOP_VALUE  = 0
REAR_STOP_DIST = 0.13  
LATERAL_DANGER_DIST    = 0.15
REAR_DANGER_DIST       = 0.15
FORWARD_CORRECT_TIME   = 1.0  
BACKWARD_CORRECT_TIME  = 0.77  
AJUSTE_ADELANTE_TIME   = 0.1   

class ParkingControllerTraxxas(Node):
    def __init__(self):
        super().__init__('parking_controller_traxxas')

        self.state        = 'ORIENTANDOSE'
        self.parking_side = 'RIGHT'
        self.yaw          = 0.0
        self.yaw_initial  = None
        self.prev_state   = None
        self.yaw_target   = None
        self.yaw_buscar   = 0.0
        
        self.lane_angle   = 90.0  
        self.vision_lista = False
        self.camara_notificada = False

        self.angulo_deseado_der = 95.0  
        self.angulo_deseado_izq = 92.0  
        self.angulo_deseado_actual = None
        self.prev_error_angulo = 0.0      

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
        self.stop_fase        = 0      
        self.stop_fase_time   = 0.0

        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
            self.get_logger().info(f'--- CONECTADO A ARDUINO ({SERIAL_PORT}) ---')
            time.sleep(2)
            self.ser.reset_input_buffer()
        except Exception as e:
            self.get_logger().error(f'ERROR ARDUINO: {e}')
            self.ser = None

        self.create_subscription(String,  '/parking/state',         self.state_cb,     10)
        self.create_subscription(String,  '/parking/perception',    self.perception_cb, 10)
        self.create_subscription(Vector3Stamped, '/imu/euler',      self.imu_cb,       10) 
        self.create_subscription(Float32, '/qcar/lane_angle_ema',   self.angle_cb,     10)

        qos_profile = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.direction_pwm_pub = self.create_publisher(String, 'direction_servo', qos_profile)
        self.throttle_pwm_pub  = self.create_publisher(String, 'throttle_motor', qos_profile)
        self.state_pub         = self.create_publisher(String, '/parking/state_feedback', 10)

        self.timer = self.create_timer(0.05, self.loop)

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
        if self.state != 'ORIENTANDOSE':
            self.state = nuevo_state

    def perception_cb(self, msg):
        self.lidar_rf = 'RF:False' in msg.data
        self.lidar_lf = 'LF:False' in msg.data

    def imu_cb(self, msg):
        raw_yaw = msg.vector.z 
        if self.yaw_initial is None:
            self.yaw_initial = raw_yaw
        self.yaw = raw_yaw - self.yaw_initial
        self.yaw = (self.yaw + 180) % 360 - 180

    def filter_range(self, value, prev):
        if value > 5.0: return prev
        if value <= 0.05: return 0.05
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
                            self.ul = self.filter_range(float(parts[0]) / 100.0, self.ul)
                            ub_val = float(parts[1]) / 100.0
                            self.ur = self.filter_range(float(parts[2]) / 100.0, self.ur)
                            self.ir_state = int(parts[3])
                            if 0.05 < ub_val < 2.0: self.ub = ub_val
                            return
                        except ValueError: continue
            except Exception: pass

    def clamp_throttle(self, value):
        return max(PWM_THROTTLE_REV_MAX, min(int(value), PWM_THROTTLE_FWD_MAX))

    def clamp_steering(self, value):
        return max(PWM_STEER_LEFT_MAX, min(int(value), PWM_STEER_RIGHT_MAX))

    def actualizar_ir_activo(self):
        if not self.ir_activo:
            if self.state == 'ESTACIONANDO' and self.lidar_rf and self.lidar_lf:
                self.ir_activo = True

    def loop(self):
        now = time.time()
        self.read_ultrasonics_fast()
        self.actualizar_ir_activo()

        if not self.vision_lista:
            self._publicar(PWM_THROTTLE_NEUTRAL, PWM_STEER_CENTER)
            return
            
        if not self.camara_notificada:
            self.get_logger().info('¡CÁMARA ACTIVADA! Arrancando...')
            self.camara_notificada = True

        if self.freno_emergencia or self.freno_stop_dist or self.freno_final_dist:
            self._publicar(PWM_THROTTLE_NEUTRAL, PWM_STEER_CENTER)
            return

        if self.state == 'ENDEREZAR' and self.prev_state != 'ENDEREZAR':
            if self.parking_side == 'RIGHT': self.yaw_target = self.yaw_buscar - 90.0
            else: self.yaw_target = self.yaw_buscar + 90.0

        self.prev_state = self.state
        speed_cmd = PWM_THROTTLE_NEUTRAL
        steer_cmd = PWM_STEER_CENTER

        # ==============================================================
        # MAQUINA DE ESTADOS - SEGUIMIENTO DE CARRIL CONTINUO CON PD
        # ==============================================================
        if self.state in ['ORIENTANDOSE', 'BUSCAR_CAJA', 'BUSCAR']:
            speed_cmd = ORIENT_SPEED if self.state == 'ORIENTANDOSE' else SEARCH_SPEED
            if self.state == 'BUSCAR': self.yaw_buscar = self.yaw

            if self.angulo_deseado_actual is None:
                if self.lane_angle > self.angulo_deseado_izq:
                    self.angulo_deseado_actual = self.angulo_deseado_izq
                else:
                    self.angulo_deseado_actual = self.angulo_deseado_der

            error_angulo = self.angulo_deseado_actual - self.lane_angle
            derivada = error_angulo - self.prev_error_angulo
            offset = (error_angulo * KP_ORIENT) + (derivada * KD_ORIENT)
            self.prev_error_angulo = error_angulo

            offset = max(-MAX_ANG_PID, min(offset, MAX_ANG_PID))
            steer_cmd = int(PWM_STEER_CENTER + offset)

            if self.state == 'ORIENTANDOSE' and abs(error_angulo) < 1.5:
                self.state = 'BUSCAR_CAJA'
                self.state_pub.publish(String(data='BUSCAR_CAJA'))

        # ==============================================================
        # MANIOBRAS DE ESTACIONAMIENTO
        # ==============================================================
        elif self.state == 'ESPERAR':
            speed_cmd = PWM_THROTTLE_NEUTRAL
            steer_cmd = PWM_STEER_CENTER

        elif self.state == 'ALEJARSE':
            speed_cmd = ALEJARSE_SPEED
            steer_cmd = STEER_ALEJARSE_IZQ if self.parking_side == 'RIGHT' else STEER_ALEJARSE_DER

        elif self.state == 'ENTRAR_DIAGONAL':
            speed_cmd = PARKING_SPEED
            steer_cmd = STEER_DIAGONAL_DER if self.parking_side == 'RIGHT' else STEER_DIAGONAL_IZQ

        elif self.state == 'ENDEREZAR':
            speed_cmd = PARKING_SPEED
            yaw_target = self.yaw_target if self.yaw_target else self.yaw
            yaw_err = (yaw_target - self.yaw + 180) % 360 - 180
            lat_err = (self.ul - self.ur) * 0.4 if self.parking_side == 'LEFT' else self.ul - self.ur
            offset = (KP_YAW * yaw_err) - (KP_LAT * lat_err)
            offset = max(-MAX_ANG_PID, min(offset, MAX_ANG_PID))
            steer_cmd = PWM_STEER_CENTER - offset
            if abs(yaw_err) < ALIGN_TOL: self.state_pub.publish(String(data='ENDEREZADO'))

        elif self.state == 'ESTACIONANDO':
            if not self.ajuste_adelante_done:
                if self.ajuste_adelante_time == 0.0: self.ajuste_adelante_time = time.time()
                if time.time() - self.ajuste_adelante_time < AJUSTE_ADELANTE_TIME:
                    speed_cmd = AJUSTE_ADELANTE_SPEED
                else: self.ajuste_adelante_done = True
            else:
                speed_cmd = PARKING_SPEED
                if (self.ir_activo and self.ir_state == IR_STOP_VALUE) or (self.ub <= REAR_STOP_DIST):
                    self.freno_stop_dist = True
                    self.state_pub.publish(String(data='DIST_OK'))

        elif self.state == 'STOP':
            if self.stop_fase == 2: self.freno_final_dist = True
            elif self.stop_fase == 0:
                if self.stop_fase_time == 0.0: self.stop_fase_time = time.time()
                if time.time() - self.stop_fase_time < 0.45: speed_cmd = STOP_FASE_SPEED
                else: self.stop_fase = 2

        # ==============================================================
        # SISTEMA DE SUPERVIVENCIA Y FRENO DE EMERGENCIA (¡Restaurado!)
        # ==============================================================
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

        # Freno crítico si va en reversa hacia un obstáculo
        cond_emerg_ir = (self.ir_activo and self.ir_state == IR_STOP_VALUE)
        cond_emerg_ub = (self.ub <= REAR_STOP_DIST)
        if speed_cmd < PWM_THROTTLE_NEUTRAL and (cond_emerg_ir or cond_emerg_ub) and self.state not in ['ESTACIONANDO', 'STOP', 'ENDEREZAR']:
            self.get_logger().warn('¡FRENO DE EMERGENCIA!')
            self.freno_emergencia = True
            speed_cmd = PWM_THROTTLE_NEUTRAL
            steer_cmd = PWM_STEER_CENTER

        # PUBLICACIÓN FINAL
        self._publicar(speed_cmd, steer_cmd)

    def _publicar(self, speed_cmd, steer_cmd):
        now = time.time()
        final_speed = self.clamp_throttle(speed_cmd)
        final_steer = self.clamp_steering(steer_cmd)

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

        self.throttle_pwm_pub.publish(String(data=str(final_speed)))
        self.direction_pwm_pub.publish(String(data=str(final_steer)))

def main():
    rclpy.init()
    node = ParkingControllerTraxxas()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.throttle_pwm_pub.publish(String(data=str(PWM_THROTTLE_NEUTRAL)))
        node.direction_pwm_pub.publish(String(data=str(PWM_STEER_CENTER)))
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__': main()