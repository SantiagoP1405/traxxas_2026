#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import time

class SensorTester(Node):
    def __init__(self):
        super().__init__('sensor_tester')
        
        # --- CONFIGURACION ---
        self.serial_port = '/dev/ttyUSB0' 
        self.baud_rate = 115200

        self.get_logger().info(f'Intentando conectar a {self.serial_port}...')

        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info('¡CONEXION EXITOSA con Arduino!')
        except Exception as e:
            self.get_logger().error(f'Error conectando: {e}')
            self.ser = None

        self.timer = self.create_timer(0.01, self.read_serial)

    def read_serial(self):
        if self.ser and self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                datos = line.split(',')

                # AHORA ESPERAMOS 4 DATOS
                if len(datos) == 4:
                    izq = datos[0]
                    cen = datos[1]
                    der = datos[2]
                    ir  = datos[3]

                    print(f"Izq: {izq}cm | Cen: {cen}cm | Der: {der}cm | IR: {ir}")
                else:
                    self.get_logger().warn(f'Dato incompleto: {line}')

            except Exception as e:
                self.get_logger().warn(f'Error procesando linea: {e}')

def main():
    rclpy.init()
    node = SensorTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()