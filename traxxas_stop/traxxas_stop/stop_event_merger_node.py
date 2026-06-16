#!/usr/bin/env python3
"""
=======================================================================
 StopEventMerger  -  traxxas_stop
 -----------------------------------------------------------------------
 Fusiona los eventos de detencion de DOS fuentes en un solo topico
 que consume el action server:

   /traxxas/stop_sign/confirmed  ──┐
                                   ├──> /traxxas/stop_event/confirmed
   /traxxas/crosswalk/confirmed  ──┘    /traxxas/stop_event/distance_m
                                        /traxxas/stop_event/source

 Logica de fusion (OR simple, sin prioridad):
   confirmed = stop_sign_confirmed OR crosswalk_confirmed

   distance_m:
     - Si solo una fuente confirma  -> usa esa distancia
     - Si ambas confirman           -> usa la menor (mas cercana)
     - Si ninguna confirma          -> -1.0

   source: "stop_sign" | "crosswalk" | "both" | "none"
     Util para debug y monitoreo.

 El action server solo necesita suscribirse a /traxxas/stop_event/*
 y funciona igual para señal de alto y paso peatonal.

 TOPICOS SUSCRITOS
   /traxxas/stop_sign/confirmed   Bool
   /traxxas/stop_sign/distance_m  Float32
   /traxxas/crosswalk/confirmed   Bool
   /traxxas/crosswalk/distance_m  Float32

 TOPICOS PUBLICADOS
   /traxxas/stop_event/confirmed  Bool
   /traxxas/stop_event/distance_m Float32
   /traxxas/stop_event/source     String  (para debug)
=======================================================================
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String


class StopEventMerger(Node):

    def __init__(self):
        super().__init__('stop_event_merger')

        # Estado de cada fuente
        self._sign_confirmed  = False
        self._sign_distance   = -1.0
        self._cross_confirmed = False
        self._cross_distance  = -1.0

        # Suscripciones
        self.create_subscription(
            Bool,    '/traxxas/stop_sign/confirmed',
            self._cb_sign_conf,  10)
        self.create_subscription(
            Float32, '/traxxas/stop_sign/distance_m',
            self._cb_sign_dist,  10)
        self.create_subscription(
            Bool,    '/traxxas/crosswalk/confirmed',
            self._cb_cross_conf, 10)
        self.create_subscription(
            Float32, '/traxxas/crosswalk/distance_m',
            self._cb_cross_dist, 10)

        # Publicadores
        self._pub_confirmed = self.create_publisher(
            Bool,    '/traxxas/stop_event/confirmed',  10)
        self._pub_distance  = self.create_publisher(
            Float32, '/traxxas/stop_event/distance_m', 10)
        self._pub_source    = self.create_publisher(
            String,  '/traxxas/stop_event/source',     10)

        # Timer 10 Hz (mismo ritmo que confirmation nodes)
        self.create_timer(0.1, self._publish_merged)

        self.get_logger().info('StopEventMerger iniciado')
        self.get_logger().info('  Fusiona: stop_sign + crosswalk -> stop_event')

    # -- Callbacks --------------------------------------------------------

    def _cb_sign_conf(self,  msg): self._sign_confirmed  = bool(msg.data)
    def _cb_sign_dist(self,  msg): self._sign_distance   = float(msg.data)
    def _cb_cross_conf(self, msg): self._cross_confirmed = bool(msg.data)
    def _cb_cross_dist(self, msg): self._cross_distance  = float(msg.data)

    # -- Logica de fusion -------------------------------------------------

    def _publish_merged(self):
        confirmed = self._sign_confirmed or self._cross_confirmed

        # Elegir distancia
        if self._sign_confirmed and self._cross_confirmed:
            # Ambas activas: usar la menor distancia valida
            dists = [d for d in [self._sign_distance, self._cross_distance]
                     if d > 0.0]
            distance = min(dists) if dists else -1.0
            source   = 'both'
        elif self._sign_confirmed:
            distance = self._sign_distance
            source   = 'stop_sign'
        elif self._cross_confirmed:
            distance = self._cross_distance
            source   = 'crosswalk'
        else:
            distance = -1.0
            source   = 'none'

        # Publicar
        msg_conf = Bool();    msg_conf.data = confirmed
        msg_dist = Float32(); msg_dist.data = float(distance)
        msg_src  = String();  msg_src.data  = source

        self._pub_confirmed.publish(msg_conf)
        self._pub_distance.publish(msg_dist)
        self._pub_source.publish(msg_src)


def main(args=None):
    rclpy.init(args=args)
    node = StopEventMerger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()