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
            self.get_logger().info('¡CONEXION EXITOSA con la ESP32!')
        except Exception as e:
            self.get_logger().error(f'Error conectando: {e}')
            self.ser = None

        # --- VARIABLES PARA GUARDAR ESTADO ---
        # Guardamos la batería para imprimirla junto con los sensores si queremos
        self.battery_total = "N/A"
        self.battery_percent = "N/A"
        self.battery_status = "N/A"

        self.timer = self.create_timer(0.01, self.read_serial)

    def read_serial(self):
        if self.ser and self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                datos = line.split(',')

                # CASO 1: Mensaje de Sensores (4 datos)
                if len(datos) == 4:
                    izq = datos[0]
                    cen = datos[1]
                    der = datos[2]
                    ir  = datos[3]

                    # Imprimimos los sensores y le pegamos la última lectura de batería conocida
                    print(f"[SENSORES] Izq:{izq}cm | Cen:{cen}cm | Der:{der}cm | IR:{ir} || Bat:{self.battery_total}V ({self.battery_percent}%) [{self.battery_status}]")

                # CASO 2: Mensaje de Batería (7 datos)
                elif len(datos) == 7:
                    # cell1, cell2, cell3, total, porcentaje, warningStr, diff
                    # Solo guardamos los que nos interesan para mostrarlos luego
                    self.battery_total = datos[3]
                    self.battery_percent = datos[4]
                    self.battery_status = datos[5]
                    
                    # Opcional: imprimir un mensaje dedicado solo cuando llega la batería
                    # print(f"--- [INFO BATERÍA] Total: {self.battery_total}V | Estado: {self.battery_status} ---")

                else:
                    # Solo avisa si llega algo totalmente diferente (ruido serial, etc.)
                    self.get_logger().debug(f'Línea ignorada (longitud {len(datos)}): {line}')

            except UnicodeDecodeError:
                # A veces al conectar el puerto los primeros bytes son basura
                pass
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