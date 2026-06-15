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
PWM_STEER_CENTER     = 2542 
PWM_STEER_LEFT_MAX   = 1669 
PWM_STEER_RIGHT_MAX  = 3276 

# --- VELOCIDADES PWM (> 2457 Adelante, < 2457 Reversa) --- 
SEARCH_SPEED  = 2680    #2682 
PARKING_SPEED = 2235     

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

# --- CONSTANTES PID ENDEREZAR --- 
KP_ORIENT   = 6.5  
KD_ORIENT   = 3.0   # <-- NUEVO: Freno derivativo para centrar llantas suavemente 15.0 
KP_YAW      = 3.5     
KP_LAT      = 4520.0  
MAX_ANG_PID = 300    # Mantenemos la fuerza extra que pediste 

ALIGN_TOL  = 4.5      
CENTER_TOL = 0.10 

# --- SENSORES DE PARADA TRASERA --- 
IR_STOP_VALUE  = 0 
REAR_STOP_DIST = 0.13   

LATERAL_DANGER_DIST    = 0.15 
REAR_DANGER_DIST       = 0.15 
FORWARD_CORRECT_TIME   = 0.66   
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
        self.camara_notificada = False # <-- Agregado para avisar 1 sola vez 

        self.angulo_deseado_der = 95.0   
        self.angulo_deseado_izq = 87.0   
        self.angulo_deseado_actual = None 
        self.prev_error_angulo = None  # <-- NUEVO: Guarda el error anterior para el derivativo 

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
        self.lidar_right_dist = 10.0 
        self.lidar_left_dist  = 10.0 
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
        self.create_subscription(Float32, '/qcar/lane_angle_ema',   self.angle_cb,     10) # Cambia a _raw si sigue con lag 

        qos_profile = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10) 
        self.direction_pwm_pub = self.create_publisher(String, 'direction_servo', qos_profile) 
        self.throttle_pwm_pub  = self.create_publisher(String, 'throttle_motor', qos_profile) 
        self.state_pub         = self.create_publisher(String, '/parking/state_feedback', 10) 
        
        # --- NUEVO: Publicador de Luces ---
        self.led_power_pub     = self.create_publisher(String, '/led_power', 10)
        self.last_led_msg      = ""
        # ----------------------------------

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
                            new_ul = float(parts[0]) / 100.0 
                            new_ub = float(parts[1]) / 100.0 
                            new_ur = float(parts[2]) / 100.0 
                            new_ir = int(parts[3]) 

                            self.ul       = self.filter_range(new_ul, self.ul) 
                            self.ur       = self.filter_range(new_ur, self.ur) 
                            self.ir_state = new_ir 
                            if 0.05 < new_ub < 2.0: self.ub = new_ub 
                            return 
                        except ValueError: 
                            continue 
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

        # --- NUEVO: ESPERAR CÁMARA --- 
        if not self.vision_lista: 
            self._publicar(PWM_THROTTLE_NEUTRAL, PWM_STEER_CENTER) 
            return 
            
        if not self.camara_notificada: 
            self.get_logger().info('¡CÁMARA ACTIVADA! Arrancando todo...') 
            self.camara_notificada = True 
        # ----------------------------- 

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
        # MAQUINA DE ESTADOS - REVERTIDA (SOLO ORIENTA AL INICIO) 
        # ============================================================== 
        
        if self.state == 'ORIENTANDOSE': 
            speed_cmd = ORIENT_SPEED 

            if not self.vision_lista: 
                steer_cmd = PWM_STEER_CENTER 
            else: 
                    if self.angulo_deseado_actual is None: 
                        # Si el ángulo es mayor que tu límite máximo (ej. > 100) 
                        if self.lane_angle > self.angulo_deseado_izq: 
                            self.angulo_deseado_actual = self.angulo_deseado_izq 
                        # Si el ángulo es menor que tu límite mínimo (ej. < 87) 
                        else: 
                            self.angulo_deseado_actual = self.angulo_deseado_der 

                    error_angulo = self.angulo_deseado_actual - self.lane_angle 
                    
                    # --- NUEVO: CÁLCULO DEL FRENO DERIVATIVO --- 
                    if self.prev_error_angulo is None: 
                        self.prev_error_angulo = error_angulo 
                        
                    d_error = error_angulo - self.prev_error_angulo 
                    self.prev_error_angulo = error_angulo 

                    # Ahora sumamos el empuje (KP) más el freno de la velocidad (KD) 
                    offset = (error_angulo * KP_ORIENT) + (d_error * KD_ORIENT) 
                    # ------------------------------------------- 

                    offset = max(-MAX_ANG_PID, min(offset, MAX_ANG_PID)) 
                    steer_cmd = PWM_STEER_CENTER + offset 

                    if abs(error_angulo) < 1.5: 
                        steer_cmd = PWM_STEER_CENTER 
                        self.state = 'BUSCAR_CAJA' 
                        self.state_pub.publish(String(data='BUSCAR_CAJA')) 

        elif self.state == 'BUSCAR_CAJA': 
            speed_cmd = SEARCH_SPEED 
            steer_cmd = PWM_STEER_CENTER 

        elif self.state == 'BUSCAR': 
            self.yaw_buscar = self.yaw 
            speed_cmd = SEARCH_SPEED 
            steer_cmd = PWM_STEER_CENTER 

        # ============================================================== 
        # MANIOBRAS DE ESTACIONAMIENTO (Resto igual) 
        # ============================================================== 
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
            yaw_target = self.yaw_target if self.yaw_target else self.yaw 
            yaw_err = (yaw_target - self.yaw + 180) % 360 - 180 
            lat_err = (self.ul - self.ur) * 0.4 if self.parking_side == 'LEFT' else self.ul - self.ur 
            
            offset = (KP_YAW * yaw_err) - (KP_LAT * lat_err) 
            offset = max(-MAX_ANG_PID, min(offset, MAX_ANG_PID)) 
            steer_cmd = PWM_STEER_CENTER - offset 

            if abs(yaw_err) < ALIGN_TOL:  
                self.state_pub.publish(String(data='ENDEREZADO')) 

        elif self.state == 'ESTACIONANDO': 
            if not self.ajuste_adelante_done: 
                if self.ajuste_adelante_time == 0.0: self.ajuste_adelante_time = time.time() 
                if time.time() - self.ajuste_adelante_time < AJUSTE_ADELANTE_TIME: 
                    speed_cmd = AJUSTE_ADELANTE_SPEED    
                    steer_cmd = PWM_STEER_CENTER 
                else: self.ajuste_adelante_done = True 
            else: 
                speed_cmd = PARKING_SPEED 
                condicion_ir = (self.ir_activo and self.ir_state == IR_STOP_VALUE) 
                condicion_ub = (self.ub <= REAR_STOP_DIST) 
                if condicion_ir or condicion_ub: 
                    self.freno_stop_dist = True 
                    self.state_pub.publish(String(data='DIST_OK')) 

        elif self.state == 'STOP': 
            if self.stop_fase == 2: self.freno_final_dist = True 
            elif self.stop_fase == 0: 
                if self.stop_fase_time == 0.0: self.stop_fase_time = time.time() 
                if time.time() - self.stop_fase_time < 0.45: 
                    speed_cmd = STOP_FASE_SPEED    
                    steer_cmd = PWM_STEER_CENTER 
                else: self.stop_fase = 2 

        # SISTEMA DE SUPERVIVENCIA 
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

        cond_emerg_ir = (self.ir_activo and self.ir_state == IR_STOP_VALUE) 
        cond_emerg_ub = (self.ub <= REAR_STOP_DIST) 

        if speed_cmd < PWM_THROTTLE_NEUTRAL and (cond_emerg_ir or cond_emerg_ub) and self.state not in ['ESTACIONANDO', 'STOP', 'ENDEREZAR']: 
            self.freno_emergencia = True 
            speed_cmd = PWM_THROTTLE_NEUTRAL 
            steer_cmd = PWM_STEER_CENTER 

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

        # --- LÓGICA PARA PUBLICAR LED_POWER ('P', 'S' y 'F') ---
        current_led_msg = ""
        if final_speed < PWM_THROTTLE_NEUTRAL:
            current_led_msg = "P" # Reversa
        elif final_speed == PWM_THROTTLE_NEUTRAL:
            current_led_msg = "S" # Detenido
        else:
            current_led_msg = "F" # Adelante

        # Solo publicamos si el estado cambió respecto a la vez anterior
        if current_led_msg != self.last_led_msg:
            self.led_power_pub.publish(String(data=current_led_msg))
            
        self.last_led_msg = current_led_msg
        # ---------------------------------------------------

        msg_throttle = String() 
        msg_throttle.data = str(final_speed) 
        self.throttle_pwm_pub.publish(msg_throttle) 

        msg_steering = String() 
        msg_steering.data = str(final_steer) 
        self.direction_pwm_pub.publish(msg_steering) 

def main(): 
    rclpy.init() 
    node = ParkingControllerTraxxas() 
    try: rclpy.spin(node) 
    except KeyboardInterrupt: pass 
    finally: 
        # --- NUEVO: Publicar 'S' de Stop cuando el nodo se apague por seguridad ---
        node.led_power_pub.publish(String(data="S"))
        # --------------------------------------------------------------------------
        node.throttle_pwm_pub.publish(String(data=str(PWM_THROTTLE_NEUTRAL))) 
        node.direction_pwm_pub.publish(String(data=str(PWM_STEER_CENTER))) 
        node.destroy_node() 
        rclpy.shutdown() 

if __name__ == '__main__': 
    main()