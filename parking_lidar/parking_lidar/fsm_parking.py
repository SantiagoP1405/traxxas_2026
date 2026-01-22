#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from enum import Enum
import time

class State(Enum):
    BUSCAR = 'BUSCAR'
    ENTRAR_DIAGONAL = 'ENTRAR_DIAGONAL'
    ENDEREZAR = 'ENDEREZAR'
    STOP = 'STOP'

class ParkingFSM(Node):
    def __init__(self):
        super().__init__('parking_fsm')

        self.state = State.BUSCAR
        self.enter_start_time = None

        self.create_subscription(
            String,
            '/parking/perception',
            self.perception_cb,
            10
        )
        
        self.create_subscription(
            String,
            '/parking/state_feedback',
            self.feedback_cb,
            10
        )

        self.pub = self.create_publisher(
            String,
            '/parking/state',
            10
        )

        # Timer para estados dependientes del tiempo
        self.timer = self.create_timer(0.05, self.timer_cb)

        self.get_logger().info(f'FSM iniciada en {self.state.value}')
        self.publish_state()

    def perception_cb(self, msg):
        data = msg.data

        fc = 'FC1' in data
        rf = 'RF1' in data
        lf = 'LF1' in data

        prev = self.state

        if self.state == State.BUSCAR:
            if fc and rf and lf:
                self.state = State.ENTRAR_DIAGONAL
                self.enter_start_time = time.time()
                self.get_logger().info('ENTRAR_DIAGONAL: inicio temporizador')

        if self.state != prev:
            self.get_logger().info(f'FSM -> {self.state.value}')
            self.publish_state()
    def feedback_cb(self, msg):
        if msg.data == 'ENDEREZADO' and self.state == State.ENDEREZAR:
            self.state = State.STOP
            self.get_logger().info('FSM: ENDEREZADO recibido → STOP')
            self.publish_state()

    def timer_cb(self):
        # lógica basada en tiempo (NO sensores)
        if self.state == State.ENTRAR_DIAGONAL and self.enter_start_time is not None:
            elapsed = time.time() - self.enter_start_time

            if elapsed > 1.5:
                self.state = State.ENDEREZAR
                self.enter_start_time = None
                self.get_logger().info('ENTRAR_DIAGONAL terminado → ENDEREZAR')
                self.publish_state()

    def publish_state(self):
        msg = String()
        msg.data = self.state.value
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = ParkingFSM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()