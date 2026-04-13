#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import time
from sensor_msgs.msg import BatteryState

class BatteryMonitorNode(Node):
    def __init__(self):
        super().__init__('battery_monitor_node')

        try:
            # Reducimos el timeout para que no bloquee el nodo
            self.ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=0.05)
            time.sleep(2)
            self.ser.reset_input_buffer()
            self.get_logger().info("✅ Serial abierto en /dev/ttyUSB1")
        except Exception as e:
            self.get_logger().error(f"❌ No se pudo abrir serial: {e}")
            exit(1)

        self.pub = self.create_publisher(BatteryState, '/battery_status', 10)
        
        # Leemos el serial súper rápido (20Hz) para vaciar el buffer
        # ya que los ultrasónicos mandan datos a 30Hz.
        self.timer = self.create_timer(0.05, self.read_serial)

    def read_serial(self):
        try:
            # Leer todas las líneas que estén encoladas en el buffer
            while self.ser.in_waiting > 0:
                raw = self.ser.readline().decode('utf-8', errors='ignore').strip()

                if not raw:
                    continue

                parts = raw.split(',')

                # Si el mensaje tiene 4 partes, es de los sensores (L, C, R, IR)
                # Lo ignoramos silenciosamente en este nodo.
                if len(parts) == 4:
                    continue
                
                # Si tiene 7 partes, es la telemetría de la batería
                elif len(parts) == 7:
                    c1, c2, c3 = map(float, parts[0:3])
                    pack = float(parts[3])
                    percent = float(parts[4]) / 100.0
                    state = parts[5]
                    # parts[6] es la diferencia (diff), la omitimos en el mensaje ROS

                    msg = BatteryState()
                    msg.voltage = pack
                    msg.cell_voltage = [c1, c2, c3]
                    msg.percentage = percent
                    msg.current = float('nan') # No tenemos sensor de corriente
                    msg.present = True
                    msg.power_supply_health = self.get_health_status(state)

                    self.pub.publish(msg)

                    self.get_logger().info(f"Batería: {pack:.2f}V | {percent*100:.1f}% | Celdas: [{c1:.2f}, {c2:.2f}, {c3:.2f}]")
                
                else:
                    # Solo nos avisa si llega basura o un formato raro
                    self.get_logger().warn(f"Formato no reconocido: {raw}")

        except Exception as e:
            self.get_logger().error(f"Error de lectura serial: {e}")

    def get_health_status(self, state_str):
        # Mapea el string del ESP32 a constantes de ROS 2 BatteryState
        if state_str == "OK":
            return BatteryState.POWER_SUPPLY_HEALTH_GOOD
        elif state_str == "LOW" or state_str == "DANGER":
            return BatteryState.POWER_SUPPLY_HEALTH_DEAD
        return BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN

def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
