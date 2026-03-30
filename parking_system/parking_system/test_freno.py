#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
import serial
import time

# --- CONFIGURACION ---
SERIAL_PORT = '/dev/ttyUSB0' 
BAUD_RATE = 115200

TEST_SPEED = -0.08         # Reversa despacito
STEERING_OFFSET = -0.0812  # Para que vaya derechito hacia atrás
EMERGENCY_STOP_DIST = 0.08 # 8 cm (0.08 metros)

class TestFreno(Node):
    def __init__(self):
        super().__init__('test_freno_node')

        self.ul = 5.0
        self.ub = 5.0 
        self.ur = 5.0

        # --- MEMORIA DEL FRENO ---
        # Esta variable recordará si ya frenamos una vez
        self.freno_activado = False

        # --- CONEXION ARDUINO ---
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
            self.get_logger().info(f' Conectado al Arduino en {SERIAL_PORT}')
            time.sleep(2)
            self.ser.reset_input_buffer()
        except Exception as e:
            self.get_logger().error(f' ERROR ARDUINO: {e}')
            self.ser = None

        # --- PUBLICADOR QCAR ---
        self.cmd_pub = self.create_publisher(Vector3Stamped, '/qcar/user_command', 10)
        
        self.timer = self.create_timer(0.05, self.loop)
        self.get_logger().info(' Test de reversa iniciado. Si pones la mano, ¡se detendrá PARA SIEMPRE!')

    def read_ultrasonics_fast(self):
        if self.ser and self.ser.in_waiting > 0:
            try:
                raw_data = self.ser.read_all().decode('utf-8', errors='ignore')
                lines = raw_data.split('\n')
                
                for line in reversed(lines):
                    if ',' in line and len(line.split(',')) == 3:
                        parts = line.strip().split(',')
                        try:
                            def clean(val):
                                v = float(val) / 100.0  
                                if v >= 4.0: 
                                    return 5.0
                                return v 
                            
                            self.ul = clean(parts[0])
                            self.ub = clean(parts[1]) 
                            self.ur = clean(parts[2])
                            return
                        except ValueError:
                            continue
            except Exception:
                pass

    def loop(self):
        self.read_ultrasonics_fast()
        
        # 1. Checar si ya estamos "bloqueados" permanentemente
        if self.freno_activado:
            speed_cmd = 0.0
            # Ya no imprime nada más que este mensaje
            self.get_logger().warn(' Vehículo bloqueado permanentemente por freno de emergencia.')
        
        # 2. Si no estamos bloqueados, seguimos la lógica normal
        else:
            speed_cmd = TEST_SPEED
            
            # Si detectamos el obstáculo, ACTIVAMOS EL BLOQUEO
            if self.ub <= EMERGENCY_STOP_DIST:
                self.get_logger().error(f'¡FRENO DE EMERGENCIA! Objeto a {self.ub:.2f} m. Apagando motores permanentemente.')
                self.freno_activado = True  # <--- AQUI ESTA LA MAGIA
                speed_cmd = 0.0
            else:
                self.get_logger().info(f'Retrocediendo... Distancia Centro: {self.ub:.2f} m')

        # 3. Mandar comandos al motor
        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'command_input'
        
        msg.vector.x = float(speed_cmd)
        msg.vector.y = float(STEERING_OFFSET)
        msg.vector.z = 0.0

        self.cmd_pub.publish(msg)

def main():
    rclpy.init()
    node = TestFreno()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Vector3Stamped()
        stop.header.stamp = node.get_clock().now().to_msg()
        stop.vector.x = 0.0
        stop.vector.y = float(STEERING_OFFSET)
        node.cmd_pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
