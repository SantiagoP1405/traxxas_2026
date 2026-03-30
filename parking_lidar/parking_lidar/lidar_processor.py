#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import math

FRONT_STOP_DIST = 0.20
RIGHT_FREE_DIST = 0.50
LEFT_FREE_DIST  = 0.50
REAR_FREE_DIST  = 0.20


class LidarProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')

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

        self.get_logger().info('LiDAR Processor iniciado')

    def scan_callback(self, msg):
        front, right, left, rear = [], [], [], []

        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r) or r < 0.05:
                continue

            angle = msg.angle_min + i * msg.angle_increment

            if -0.17 < angle < 0.17:
                front.append(r)
            elif -1.60 < angle < -1.30:
                right.append(r)
            elif 0.37 < angle < 1.07:
                left.append(r)
            elif abs(abs(angle) - math.pi) < 0.26:
                rear.append(r)

        front_clear = min(front, default=10.0) > FRONT_STOP_DIST
        right_free  = min(right, default=10.0) > RIGHT_FREE_DIST
        left_free   = min(left,  default=10.0) > LEFT_FREE_DIST
        rear_clear  = min(rear,  default=10.0) > REAR_FREE_DIST

        self.get_logger().info(
            f'LIDAR | FC:{front_clear} RF:{right_free} LF:{left_free} RC:{rear_clear}'
        )

        msg_out = String()
        msg_out.data = (
            f'FC{int(front_clear)} '
            f'RF{int(right_free)} '
            f'LF{int(left_free)} '
            f'RC{int(rear_clear)}'
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