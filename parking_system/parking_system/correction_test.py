#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
import serial
import time

# --- CONFIGURACION ---
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200

# --- VELOCIDAD DE PRUEBA ---
TEST_SPEED = -0.05  # reversa despacito

# --- CALIBRACIÓN ---
STEERING_OFFSET = -0.0812

# --- CORRECCIÓN LATERAL ---
DIAG_CORRECTION_THRESH = 0.05
DIAG_CORRECTION_GAIN   = 0.08

# --- BASE STEER con margen para corregir en ambos lados ---
BASE_STEER = 0.0

MAX_STEER_LIMIT = 0.5


class TestCorreccionLateral(Node):
    def __init__(self):
        super().__init__('test_correccion_lateral')

        self.ul = 5.0
        self.ur = 5.0
        self.ub = 5.0

        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
            self.get_logger().info(f'Conectado al Arduino en {SERIAL_PORT}')
            time.sleep(2)
            self.ser.reset_input_buffer()
        except Exception as e:
            self.get_logger().error(f'ERROR ARDUINO: {e}')
            self.ser = None

        self.cmd_pub = self.create_publisher(Vector3Stamped, '/qcar/user_command', 10)
        self.timer = self.create_timer(0.05, self.loop)
        self.get_logger().info('Test Corrección Lateral iniciado — pon la mano en ul o ur y observa el steer')

    def filter_range(self, value, prev):
        if value <= 0.05 or value > 5.0:
            return prev
        return 0.3 * prev + 0.7 * value

    def read_ultrasonics(self):
        if self.ser and self.ser.in_waiting > 0:
            try:
                raw_data = self.ser.read_all().decode('utf-8', errors='ignore')
                lines = raw_data.split('\n')
                for line in reversed(lines):
                    if ',' in line and len(line.split(',')) == 3:
                        parts = line.strip().split(',')
                        try:
                            new_ul = float(parts[0]) / 100.0
                            new_ub = float(parts[1]) / 100.0
                            new_ur = float(parts[2]) / 100.0

                            self.ul = self.filter_range(new_ul, self.ul)
                            self.ur = self.filter_range(new_ur, self.ur)
                            if 0.05 < new_ub < 2.0:
                                self.ub = new_ub
                            return
                        except ValueError:
                            continue
            except Exception:
                pass

    def clamp(self, value, limit):
        return max(-limit, min(value, limit))

    def loop(self):
        self.read_ultrasonics()

        lat_diff = self.ul - self.ur

        if lat_diff > DIAG_CORRECTION_THRESH:
            correction = -DIAG_CORRECTION_GAIN
            lado = 'ul MAS CERCA → corrigiendo a la derecha'
        elif lat_diff < -DIAG_CORRECTION_THRESH:
            correction = +DIAG_CORRECTION_GAIN
            lado = 'ur MAS CERCA → corrigiendo a la izquierda'
        else:
            correction = 0.0
            lado = 'centrado'

        steer_cmd = self.clamp(BASE_STEER + correction, MAX_STEER_LIMIT)

        self.get_logger().info(
            f'ul={self.ul:.2f}m  ur={self.ur:.2f}m  '
            f'diff={lat_diff:.3f}  corr={correction:.3f}  '
            f'steer={steer_cmd:.3f} | {lado}'
        )

        # 1. Dirección lógica
        logical_steer = self.clamp(-steer_cmd, MAX_STEER_LIMIT)

        # 2. Aplicar offset mecánico
        hardware_steer = logical_steer + STEERING_OFFSET

        # 3. Limitar de nuevo por si la suma supera el límite
        hardware_steer = self.clamp(hardware_steer, MAX_STEER_LIMIT)

        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'command_input'
        msg.vector.x = float(TEST_SPEED)
        msg.vector.y = float(hardware_steer)
        msg.vector.z = 0.0

        self.cmd_pub.publish(msg)


def main():
    rclpy.init()
    node = TestCorreccionLateral()
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
