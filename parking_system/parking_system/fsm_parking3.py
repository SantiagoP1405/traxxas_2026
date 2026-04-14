#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from enum import Enum
import time

DETECT_EXTRA_TIME = 0.0 
CONFIRM_TIME      = 0.17    
WAIT_TIME         = 1.0
ALEJARSE_TIME     = 0.2 #0.77   
ENTER_TIME        = 3.0   
PARK_WAIT_TIME    = 1.0

class State(Enum):
    ORIENTANDOSE     = 'ORIENTANDOSE'
    BUSCAR_CAJA      = 'BUSCAR_CAJA'       # NUEVO ESTADO: Avanza hasta tener una caja a lado
    BUSCAR           = 'BUSCAR'            # ESTADO ORIGINAL: Busca el hueco
    ESPERAR          = 'ESPERAR'
    ALEJARSE         = 'ALEJARSE'          
    ENTRAR_DIAGONAL  = 'ENTRAR_DIAGONAL'
    ENDEREZAR        = 'ENDEREZAR'
    ESTACIONANDO     = 'ESTACIONANDO'
    STOP             = 'STOP'

class ParkingFSM3(Node):
    def __init__(self):
        super().__init__('parking_fsm3')

        self.box_side = None
        self.state = State.ORIENTANDOSE # Inicia alineándose al carril

        self.detect_start_time   = None
        self.confirm_start_time  = None
        self.wait_start_time     = None
        self.alejarse_start_time = None    
        self.enter_start_time    = None
        self.park_wait_start     = None

        self.create_subscription(String, '/parking/perception',     self.perception_cb, 10)
        self.create_subscription(String, '/parking/state_feedback', self.feedback_cb,   10)

        self.pub   = self.create_publisher(String, '/parking/state', 10)
        self.timer = self.create_timer(0.05, self.timer_cb)

        self.get_logger().info(f'FSM3 iniciado en {self.state.value}')
        self.publish_state()

    def perception_cb(self, msg):
        # Lógica para BUSCAR_CAJA: Solo nos importa encontrar la primera caja
        if self.state == State.BUSCAR_CAJA:
            rf = 'RF1' in msg.data
            lf = 'LF1' in msg.data

            if not rf and lf:
                self.box_side = 'RIGHT'
                self.get_logger().info('Caja detectada a la DERECHA. Cambiando a BUSCAR hueco.')
                self.state = State.BUSCAR
                self.publish_state()
            elif not lf and rf:
                self.box_side = 'LEFT'
                self.get_logger().info('Caja detectada a la IZQUIERDA. Cambiando a BUSCAR hueco.')
                self.state = State.BUSCAR
                self.publish_state()
            return

        # Lógica original para BUSCAR (buscar el hueco)
        if self.state != State.BUSCAR:
            return

        fc = 'FC1' in msg.data
        rf = 'RF1' in msg.data
        lf = 'LF1' in msg.data

        if fc and rf and lf:
            if self.confirm_start_time is None:
                self.confirm_start_time = time.time()
                self.get_logger().info('Posible espacio, confirmando...')

            if self.detect_start_time is None:
                if time.time() - self.confirm_start_time >= CONFIRM_TIME:
                    self.detect_start_time = time.time()
                    self.get_logger().info('Espacio CONFIRMADO')
        else:
            if self.confirm_start_time is not None:
                self.get_logger().info('Espacio perdido, reseteando confirmación')
            self.confirm_start_time = None

    def feedback_cb(self, msg):
        if msg.data == 'BUSCAR_CAJA' and self.state == State.ORIENTANDOSE:
            self.state = State.BUSCAR_CAJA
            self.get_logger().info('ORIENTANDOSE -> BUSCAR_CAJA')
            self.publish_state()

        elif msg.data == 'ENDEREZADO' and self.state == State.ENDEREZAR:
            self.state           = State.ESTACIONANDO
            self.park_wait_start = time.time()
            self.get_logger().info('ENDEREZAR -> ESTACIONANDO')
            self.publish_state()

        elif msg.data == 'DIST_OK' and self.state == State.ESTACIONANDO:
            self.state = State.STOP
            self.get_logger().info('ESTACIONANDO -> STOP')
            self.publish_state()

    def timer_cb(self):
        now = time.time()

        if self.state == State.BUSCAR and self.detect_start_time:
            if now - self.detect_start_time > DETECT_EXTRA_TIME:
                self.state             = State.ESPERAR
                self.wait_start_time   = now
                self.detect_start_time = None
                self.get_logger().info('BUSCAR -> ESPERAR')
                self.publish_state()

        elif self.state == State.ESPERAR and self.wait_start_time:
            if now - self.wait_start_time > WAIT_TIME:
                self.state               = State.ALEJARSE
                self.alejarse_start_time = now
                self.wait_start_time     = None
                self.get_logger().info('ESPERAR -> ALEJARSE')
                self.publish_state()

        elif self.state == State.ALEJARSE and self.alejarse_start_time:
            if now - self.alejarse_start_time > ALEJARSE_TIME:
                self.state               = State.ENTRAR_DIAGONAL
                self.enter_start_time    = now
                self.alejarse_start_time = None
                self.get_logger().info('ALEJARSE -> ENTRAR_DIAGONAL')
                self.publish_state()

        elif self.state == State.ENTRAR_DIAGONAL and self.enter_start_time:
            if now - self.enter_start_time > ENTER_TIME:
                self.state            = State.ENDEREZAR
                self.enter_start_time = None
                self.get_logger().info('ENTRAR_DIAGONAL -> ENDEREZAR')
                self.publish_state()

        elif self.state == State.ESTACIONANDO and self.park_wait_start:
            if now - self.park_wait_start > PARK_WAIT_TIME:
                self.park_wait_start = None

    def parking_side(self):
        if self.box_side == 'LEFT':
            return 'LEFT'
        elif self.box_side == 'RIGHT':
            return 'RIGHT'
        return 'RIGHT'  

    def publish_state(self):
        side     = self.parking_side()
        msg      = String()
        msg.data = f'{self.state.value}:{side}'
        self.get_logger().info(f'FSM3 publica -> {msg.data}')
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = ParkingFSM3()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
