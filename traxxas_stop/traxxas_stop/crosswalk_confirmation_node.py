#!/usr/bin/env python3
"""
=======================================================================
 CrosswalkConfirmation  -  traxxas_stop
-----------------------------------------------------------------------
 Filtra la deteccion raw del paso peatonal con un sistema de score
 identico al de stop_confirmation_node, para evitar falsos positivos.

 La logica de score es deliberadamente identica a stop_confirmation:
   - Cada tick con evidencia valida: score += score_gain
   - Cada tick sin evidencia:        score -= score_decay
   - confirmed = True cuando score >= score_trigger

 Diferencia vs stop_confirmation:
   - No usa "strong_area" (el paso peatonal no tiene area equivalente)
   - Usa stripe_count como metrica de calidad en lugar de area
   - La distancia maxima puede ser mas grande porque el paso peatonal
     se detecta antes de llegar a el (el carro lo ve en el suelo
     mientras se acerca)

 TOPICOS SUSCRITOS
   /traxxas/crosswalk/detected_raw   Bool
   /traxxas/crosswalk/distance_m     Float32
   /traxxas/crosswalk/stripe_count   Float32

 TOPICOS PUBLICADOS
   /traxxas/crosswalk/confirmed      Bool
   /traxxas/crosswalk/score          Float32
=======================================================================
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32


class CrosswalkConfirmation(Node):

    def __init__(self):
        super().__init__('crosswalk_confirmation')

        self.declare_parameter('score_gain',    2)
        self.declare_parameter('score_decay',   1)
        self.declare_parameter('score_max',     10)
        self.declare_parameter('score_trigger', 5)
        self.declare_parameter('min_stripes_for_confirmation', 3.0)
        self.declare_parameter('max_distance_for_confirmation', 1.20)

        self.score_gain    = int(self.get_parameter('score_gain').value)
        self.score_decay   = int(self.get_parameter('score_decay').value)
        self.score_max     = int(self.get_parameter('score_max').value)
        self.score_trigger = int(self.get_parameter('score_trigger').value)
        self.min_stripes   = float(
            self.get_parameter('min_stripes_for_confirmation').value)
        self.max_distance  = float(
            self.get_parameter('max_distance_for_confirmation').value)

        self.last_raw      = False
        self.last_stripes  = 0.0
        self.last_distance = -1.0
        self.score         = 0

        self.create_subscription(
            Bool,    '/traxxas/crosswalk/detected_raw',
            self.raw_callback,      10)
        self.create_subscription(
            Float32, '/traxxas/crosswalk/stripe_count',
            self.stripes_callback,  10)
        self.create_subscription(
            Float32, '/traxxas/crosswalk/distance_m',
            self.distance_callback, 10)

        self.confirmed_pub = self.create_publisher(
            Bool,    '/traxxas/crosswalk/confirmed', 10)
        self.score_pub     = self.create_publisher(
            Float32, '/traxxas/crosswalk/score',     10)

        self.timer = self.create_timer(0.1, self.update_logic)

        self.get_logger().info('CrosswalkConfirmation iniciado')

    def raw_callback(self, msg):
        self.last_raw = bool(msg.data)

    def stripes_callback(self, msg):
        self.last_stripes = float(msg.data)

    def distance_callback(self, msg):
        self.last_distance = float(msg.data)

    def update_logic(self):
        valid_stripes  = self.last_stripes >= self.min_stripes
        valid_distance = (self.last_distance > 0.0 and
                          self.last_distance <= self.max_distance)

        evidence = self.last_raw and valid_stripes and valid_distance

        if evidence:
            self.score = min(self.score_max, self.score + self.score_gain)
        else:
            self.score = max(0, self.score - self.score_decay)

        confirmed = self.score >= self.score_trigger

        msg_conf = Bool();    msg_conf.data = confirmed
        msg_score = Float32(); msg_score.data = float(self.score)
        self.confirmed_pub.publish(msg_conf)
        self.score_pub.publish(msg_score)


def main(args=None):
    rclpy.init(args=args)
    node = CrosswalkConfirmation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()