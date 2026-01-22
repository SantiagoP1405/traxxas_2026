#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import String
from ackermann_msgs.msg import AckermannDriveStamped
import math

# Ganancias
KP_YAW = 1.2
KP_LAT = 0.8

MAX_STEER = 0.35      # rad (~20°)
REVERSE_SPEED = -0.05

STOP_DIST = 0.15
ALIGN_TOL = 0.03
CENTER_TOL = 0.03


class ParkingController(Node):
    def __init__(self):
        super().__init__('parking_controller')

        self.state = 'BUSCAR'
        self.yaw = 0.0
        self.yaw_ref = None

        self.ul = 1.0
        self.ur = 1.0
        self.ub = 1.0

        self.create_subscription(String, '/parking/state', self.state_cb, 10)
        self.create_subscription(Imu, '/imu', self.imu_cb, 10)
        self.create_subscription(Range, '/ultra/rear_left', self.ul_cb, 10)
        self.create_subscription(Range, '/ultra/rear_right', self.ur_cb, 10)
        self.create_subscription(Range, '/ultra/rear', self.ub_cb, 10)

        self.cmd_pub = self.create_publisher(
            AckermannDriveStamped,
            '/cmd_ackermann',
            10
        )

        self.state_pub = self.create_publisher(
            String,
            '/parking/state_feedback',
            10
        )

        self.timer = self.create_timer(0.05, self.loop)

        self.get_logger().info('Parking Controller ACKERMANN iniciado')

    # callbacks

    def state_cb(self, msg):
        self.state = msg.data

        if self.state == 'ENDEREZAR' and self.yaw_ref is None:
            self.yaw_ref = self.yaw
            self.get_logger().info(
                f'Yaw referencia guardado: {self.yaw_ref:.3f} rad'
            )

        if self.state != 'ENDEREZAR':
            self.yaw_ref = None

    def imu_cb(self, msg):
        q = msg.orientation
        siny = 2 * (q.w*q.z + q.x*q.y)
        cosy = 1 - 2 * (q.y*q.y + q.z*q.z)
        self.yaw = math.atan2(siny, cosy)

    def ul_cb(self, msg): self.ul = msg.range
    def ur_cb(self, msg): self.ur = msg.range
    def ub_cb(self, msg): self.ub = msg.range

    # loop control

    def loop(self):
        cmd = AckermannDriveStamped()
        cmd.drive.speed = 0.0
        cmd.drive.steering_angle = 0.0

        # entrar en diagonal
        if self.state == 'ENTRAR_DIAGONAL':
            cmd.drive.speed = REVERSE_SPEED
            cmd.drive.steering_angle = 0.30

            self.get_logger().info(
                'ENTRAR_DIAGONAL | reverse + steering fijo'
            )

        # enderezar 
        elif self.state == 'ENDEREZAR' and self.yaw_ref is not None:
            yaw_err = self.yaw_ref - self.yaw
            lat_err = self.ul - self.ur

            steer = KP_YAW*yaw_err + KP_LAT*lat_err
            steer = max(-MAX_STEER, min(MAX_STEER, steer))

            cmd.drive.speed = REVERSE_SPEED
            cmd.drive.steering_angle = steer

            self.get_logger().info(
                f'ENDEREZAR | yaw_err:{yaw_err:.3f} '
                f'lat_err:{lat_err:.3f} '
                f'steer:{steer:.3f} '
                f'UL:{self.ul:.2f} UR:{self.ur:.2f} UB:{self.ub:.2f}'
            )

            if abs(yaw_err) < ALIGN_TOL and abs(lat_err) < CENTER_TOL:
                self.get_logger().info('ENDEREZADO ✔')
                msg = String()
                msg.data = 'ENDEREZADO'
                self.state_pub.publish(msg)

        # stop por ultrasonido
        if self.ub < STOP_DIST:
            cmd.drive.speed = 0.0
            cmd.drive.steering_angle = 0.0
            self.get_logger().info('STOP POR ULTRASONIDO')

        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = ParkingController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()