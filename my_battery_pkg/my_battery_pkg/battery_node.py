#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import time
from sensor_msgs.msg import BatteryState


class BatteryNode(Node):
    def __init__(self):
        super().__init__('battery_node')

        try:
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            time.sleep(2)
            self.ser.reset_input_buffer()
            self.get_logger().info("Serial abierto en /dev/ttyACM0")
        except Exception as e:
            self.get_logger().error(f"No se pudo abrir serial: {e}")
            exit(1)

        self.pub = self.create_publisher(BatteryState, '/battery_status', 10)
        self.timer = self.create_timer(1.0, self.read_serial)

    def read_serial(self):
        try:
            raw = self.ser.readline().decode(errors='ignore').strip()

            if not raw:
                return

            parts = raw.split(',')

            if len(parts) != 7:
                self.get_logger().warn(f"Formato inválido: {raw}")
                return

            c1, c2, c3 = map(float, parts[0:3])
            pack = float(parts[3])
            percent = float(parts[4]) / 100.0
            state = parts[5]
            diff = float(parts[6])

            msg = BatteryState()
            msg.voltage = pack
            msg.cell_voltage = [c1, c2, c3]
            msg.percentage = percent
            msg.current = float('nan')
            msg.present = True

            self.pub.publish(msg)

            self.get_logger().info(
                f"C1={c1:.2f} C2={c2:.2f} C3={c3:.2f} "
                f"Pack={pack:.2f} {percent*100:.1f}% {state}"
            )

        except Exception as e:
            self.get_logger().error(f"Serial error: {e}")


def main():
    rclpy.init()
    node = BatteryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

