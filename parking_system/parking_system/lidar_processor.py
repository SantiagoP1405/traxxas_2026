#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import math

FRONT_STOP_DIST = 0.20  #0.20
RIGHT_FREE_DIST = 0.30 #0.57
LEFT_FREE_DIST  = 0.30 #0.57
REAR_FREE_DIST  = 0.20  #0.20

class LidarProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')
        # Tópico de lectura (asegúrate de que coincida con el que publica tu LiDAR)
        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.pub = self.create_publisher(
            String,
            '/parking/perception',
            10
        )
        self.get_logger().info('LiDAR Processor iniciado con calibración en grados (0-360)')

    def scan_callback(self, msg):
        front, right, left, rear = [], [], [], []
        
        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r) or r < 0.05:
                continue
            
            # 1. Obtenemos el ángulo original en radianes
            angle_rad = msg.angle_min + i * msg.angle_increment
            
            # 2. MAGIA: Lo pasamos a grados y forzamos que esté entre 0 y 359.99
            angle_deg = math.degrees(angle_rad) % 360
            
            # 3. Asignamos según los grados 
            # (Ajusta estos rangos si tu LiDAR está montado chueco)
            if 85 < angle_deg < 95:          # Front (90 grados)
                left.append(r)
            elif 170 < angle_deg < 190:      # Left  (180 grados)
                rear.append(r)
            elif 265 < angle_deg < 275:      # Rear  (270 grados)
                right.append(r)
            elif angle_deg > 350 or angle_deg < 10: # Right (0 / 360 grados)
                front.append(r)

        # Evaluamos si está libre superando la distancia mínima
        front_clear = min(front, default=10.0) > FRONT_STOP_DIST
        right_free  = min(right, default=10.0) > RIGHT_FREE_DIST
        left_free   = min(left,  default=10.0) > LEFT_FREE_DIST
        rear_clear  = min(rear,  default=10.0) > REAR_FREE_DIST

        # Distancias mínimas reales de cada lado
        right_dist  = round(min(right, default=10.0), 3)
        left_dist   = round(min(left,  default=10.0), 3)

        self.get_logger().info(
            f'LIDAR | FC:{front_clear} RF:{right_free} LF:{left_free} RC:{rear_clear}'
        )

        msg_out = String()
        msg_out.data = (
            f'FC{int(front_clear)} '
            f'RF{int(right_free)} '
            f'LF{int(left_free)} '
            f'RC{int(rear_clear)} '
            f'RD{right_dist} '  
            f'LD{left_dist}'    
        )
        self.pub.publish(msg_out)

def main():
    rclpy.init()
    node = LidarProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()