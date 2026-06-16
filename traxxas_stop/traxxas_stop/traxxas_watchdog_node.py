#!/usr/bin/env python3
"""
=======================================================================
 Traxxas Watchdog Safety Node
-----------------------------------------------------------------------
 Monitorea /throttle_motor y /direction_servo.
 Si deja de recibir comandos reales durante más de `timeout` segundos,
 inyecta la secuencia de parada segura:
   throttle → th_min  (pulso de freno)
   throttle → th_center (neutro)
   direction → dir_center (recto)

 Para distinguir mensajes propios de los del controlador se usa un
 flag interno _watchdog_publishing, evitando el bucle infinito de
 reset que ocurriría si se re-suscribiera al mismo topic.
=======================================================================
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TraxxasWatchdog(Node):
    """Safety watchdog que garantiza parada si los comandos se detienen."""

    def __init__(self):
        super().__init__('traxxas_watchdog')

        # ── Parámetros ───────────────────────────────────────────────
        self.declare_parameter('timeout',         0.3)    # s sin comandos → parada
        self.declare_parameter('throttle_center', 2457)   # neutro
        self.declare_parameter('throttle_min',    1638)   # pulso de freno
        self.declare_parameter('dir_center',      2642)   # dirección recta

        self.timeout      = float(self.get_parameter('timeout').value)
        self.th_center    = int(self.get_parameter('throttle_center').value)
        self.th_min       = int(self.get_parameter('throttle_min').value)
        self.dir_center   = int(self.get_parameter('dir_center').value)

        # ── Estado interno ───────────────────────────────────────────
        self.last_real_cmd_time  = self.get_clock().now()
        self._watchdog_publishing = False   # flag para no resetear timer propio
        self._stopped             = False   # evita publicar la secuencia en loop

        # ── Suscripciones (escucha comandos del controlador) ─────────
        # Basta monitorear uno de los dos topics; si el controlador
        # publica uno, publica el otro también en el mismo ciclo.
        self.create_subscription(
            String, '/throttle_motor',
            self.throttle_callback, 10
        )

        # ── Publicadores ─────────────────────────────────────────────
        self.pub_throttle  = self.create_publisher(String, '/throttle_motor',  10)
        self.pub_direction = self.create_publisher(String, '/direction_servo',  10)

        # ── Timer de vigilancia (20 Hz) ──────────────────────────────
        self.timer = self.create_timer(0.05, self.watchdog_cb)

        self.get_logger().info(
            f'TraxxasWatchdog iniciado | timeout={self.timeout:.2f} s | '
            f'th_center={self.th_center} | th_min={self.th_min} | '
            f'dir_center={self.dir_center}'
        )

    # ── Callback ─────────────────────────────────────────────────────

    def throttle_callback(self, msg: String):
        """
        Recibe mensajes de /throttle_motor.
        Ignora los que publicó este mismo nodo (_watchdog_publishing=True).
        """
        if not self._watchdog_publishing:
            self.last_real_cmd_time = self.get_clock().now()
            self._stopped = False   # el controlador volvió → listo para parar de nuevo si hace falta

    # ── Lógica del watchdog ───────────────────────────────────────────

    def watchdog_cb(self):
        now = self.get_clock().now()
        dt  = (now.nanoseconds - self.last_real_cmd_time.nanoseconds) * 1e-9

        if dt <= self.timeout:
            return   # todo bien, el controlador sigue vivo

        if self._stopped:
            return   # ya se envió la secuencia, no repetir en bucle

        # ── Timeout superado: secuencia de parada segura ──────────────
        self.get_logger().warn(
            f'Sin comandos durante {dt:.2f} s (timeout={self.timeout:.2f} s) '
            f'→ PARADA DE SEGURIDAD'
        )

        self._watchdog_publishing = True
        try:
            self._publish(self.pub_throttle,  self.th_min)      # pulso freno
            self._publish(self.pub_direction, self.dir_center)  # dirección recta
            self._publish(self.pub_throttle,  self.th_center)   # neutro
        finally:
            self._watchdog_publishing = False

        self._stopped = True
        self.get_logger().info(
            f'Parada segura aplicada | '
            f'throttle={self.th_min}→{self.th_center} | dir={self.dir_center}'
        )

    def _publish(self, publisher, value: int):
        msg = String()
        msg.data = str(value)
        publisher.publish(msg)


# ── MAIN ──────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = TraxxasWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Parada garantizada al cerrar el nodo
        try:
            node._watchdog_publishing = True
            node._publish(node.pub_throttle,  node.th_min)
            node._publish(node.pub_direction, node.dir_center)
            node._publish(node.pub_throttle,  node.th_center)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()