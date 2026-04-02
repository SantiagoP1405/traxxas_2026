#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
import sys, select, termios, tty # Librerias para leer teclado en Linux

# --- CONFIGURACION PARA LEER TECLAS SIN ENTER ---
settings = termios.tcgetattr(sys.stdin)

def getKey():
    # Esta funcion cambia la terminal a modo "crudo" para leer 1 tecla
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1) # Espera 0.1s
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class MotorTester(Node):
    def __init__(self):
        super().__init__('motor_tester')
        self.pub = self.create_publisher(Vector3Stamped, '/qcar/user_command', 10)
        self.get_logger().info('--- TESTER CONTINUO ---')

    def send_command(self, speed, steer):
        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'command_input'
        msg.vector.x = float(speed)
        msg.vector.y = float(steer)
        msg.vector.z = 0.0 
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = MotorTester()

    print("\n===========================================")
    print("   CONTROL DE VELOCIDAD CONSTANTE")
    print("===========================================")
    
    try:
        # 1. PEDIMOS LOS VALORES UNA VEZ AL INICIO
        v_input = input("1. Ingresa Velocidad constante (-0.2 a 0.2): ")
        s_input = input("2. Ingresa Giro constante (-0.5 a 0.5): ")
        
        target_speed = float(v_input)
        target_steer = float(s_input)

        print(f"\n--> MOVIENDO A: Vel={target_speed}, Giro={target_steer}")
        print("--> PRESIONA LA TECLA 'a' PARA DETENER EL COCHE <--\n")

        # 2. BUCLE INFINITO HASTA QUE SE APRIETE 'a'
        while True:
            # Enviamos el comando al coche
            node.send_command(target_speed, target_steer)
            
            # Revisamos si se apretó alguna tecla (sin bloquear el programa)
            key = getKey()
            
            if key == 'a':
                print("\n¡TECLA 'A' DETECTADA! FRENANDO...")
                break
            
            # Ctrl+C tambien funciona por si acaso
            if key == '\x03': 
                break

            # Mantenemos vivo a ROS
            rclpy.spin_once(node, timeout_sec=0.01)

    except Exception as e:
        print(e)

    finally:
        # 3. RUTINA DE SEGURIDAD (Siempre se ejecuta al final)
        print("Enviando comando de parada (0,0)...")
        # Enviamos varias veces el 0 para asegurar que frene
        for _ in range(10):
            node.send_command(0.0, 0.0)
            rclpy.spin_once(node, timeout_sec=0.01)

        # Restauramos la terminal a la normalidad
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()