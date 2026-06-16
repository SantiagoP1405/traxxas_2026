#!/usr/bin/env python3
"""
=======================================================================
 StopRoutineActionServer  -  traxxas_stop
-----------------------------------------------------------------------
Action server que gestiona N paradas completas ante señales de ALTO
 y/o pasos peatonales (crosswalk) para el Traxxas físico.

 Coordinacion con pp_node
 ------------------------
   stop_active = False  ->  pp_node controla /throttle_motor
   stop_active = True   ->  ESTE SERVER manda los valores de throttle (no lo controla) /throttle_motor

 Retroalimentacion del encoder
   Suscrito a /wheel/twist (TwistStamped, linear.x en m/s).
   - BRAKING  : StopManeuver/BrakeRamp extiende la rampa hasta
                que |v| < 0.03 m/s (encoder). Evita declarar
                "detenido" si el vehiculo aun se mueve.
   - STOPPED  : solo temporal (5 s por defecto).
   - RESUMING : StopManeuver/ResumeRamp espera v > 0.10 m/s tras
                la rampa de tiempo. Evita pasar a COOLDOWN si el
                vehiculo no arranco realmente.

    Latch de punto ciego en APPROACHING:
      Si la cámara pierde la señal estando en APPROACHING (señal salió
      del encuadre al acercarse), se fuerza BRAKING a ciegas en lugar
      de volver a SEARCHING. Evita que el vehículo pase de largo.

 FSM de estados
 ───────────────
   SEARCHING   ← pp_node controla throttle
   APPROACHING ← este server toma control (reduce throttle por distancia)
   BRAKING     ← StopManeuver BRAKING  (rampa + encoder confirm)
   STOPPED     ← StopManeuver STOPPED  (espera N s)
   RESUMING    ← StopManeuver RESUMING (rampa + encoder confirm)
   COOLDOWN    ← cede throttle, espera que señal desaparezca
 
 Tópicos suscritos
 ──────────────────
   /traxxas/stop_event/confirmed  Bool
   /traxxas/stop_event/distance_m Float32
   /traxxas/stop_event/source     String  (para log)
   /wheel/twist                   TwistStamped  (encoder)
 
 Tópicos publicados
 ───────────────────
   /traxxas/stop_throttle         Float32  (0.0–1.0, normalizado)
   /traxxas/stop_sign/active      Bool
   /traxxas/mission/done          Bool
=======================================================================
"""

import time
import rclpy
 
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool, Float32, String
from traxxas_stop_interfaces.action import StopRoutine
from traxxas_stop.stop_maneuvers import StopManeuver
 
 
class StopRoutineActionServer(Node):
 
    ST_SEARCHING   = 'SEARCHING'
    ST_APPROACHING = 'APPROACHING'
    ST_BRAKING     = 'BRAKING'
    ST_STOPPED     = 'STOPPED'
    ST_RESUMING    = 'RESUMING'
    ST_COOLDOWN    = 'COOLDOWN'
 
    LOOP_HZ        = 20
    LOOP_DT        = 1.0 / LOOP_HZ
    COOLDOWN_MIN_S = 2.5
 
    def __init__(self):
        super().__init__('stop_routine_action_server')
 
        self._cb_group = ReentrantCallbackGroup()
 
        # ── Suscriptores ─────────────────────────────────────────────
        # Fuente unificada: stop_event_merger fusiona stop_sign + crosswalk
        self.create_subscription(
            Bool,    '/traxxas/stop_event/confirmed',
            self._cb_confirmed, 10, callback_group=self._cb_group)
        self.create_subscription(
            Float32, '/traxxas/stop_event/distance_m',
            self._cb_distance, 10, callback_group=self._cb_group)
        self.create_subscription(
            String,  '/traxxas/stop_event/source',
            self._cb_source, 10, callback_group=self._cb_group)
 
        # Encoder del Traxxas (sin cambio)
        self.create_subscription(
            TwistStamped, '/wheel/twist',
            self._cb_twist, 10, callback_group=self._cb_group)
 
        # ── Publicadores ─────────────────────────────────────────────
        # Throttle normalizado → pp_node lo convierte a PWM
        self._pub_throttle = self.create_publisher(
            Float32, '/traxxas/stop_throttle', 10)
        self._pub_active   = self.create_publisher(
            Bool,    '/traxxas/stop_sign/active', 10)
        self._pub_done     = self.create_publisher(
            Bool,    '/traxxas/mission/done', 10)
 
        # ── Estado compartido ─────────────────────────────────────────
        self._confirmed  : bool  = False
        self._distance_m : float = -1.0
        self._source     : str   = 'none'
        self._velocity   : float = 0.0   # m/s del encoder
 
        # ── Action Server ─────────────────────────────────────────────
        self._action_server = ActionServer(
            self, StopRoutine, 'stop_routine',
            execute_callback=self._execute_cb,
            callback_group=self._cb_group,
        )
 
        self.get_logger().info('=' * 58)
        self.get_logger().info(' STOP ROUTINE ACTION SERVER (Traxxas) iniciado')
        self.get_logger().info('  Fuente: /traxxas/stop_event/* (stop_sign + crosswalk)')
        self.get_logger().info('  Encoder: /wheel/twist (TwistStamped)')
        self.get_logger().info('  Throttle: /traxxas/stop_throttle (normalizado)')
        self.get_logger().info('=' * 58)
 
    # ── Callbacks ─────────────────────────────────────────────────────
 
    def _cb_confirmed(self, msg: Bool):
        self._confirmed = bool(msg.data)
 
    def _cb_distance(self, msg: Float32):
        self._distance_m = float(msg.data)
 
    def _cb_source(self, msg: String):
        self._source = str(msg.data)
 
    def _cb_twist(self, msg: TwistStamped):
        self._velocity = float(msg.twist.linear.x)
 
    # ── Helpers de publicación ────────────────────────────────────────
 
    def _pub_throttle_norm(self, norm: float):
        """
        Publica throttle normalizado (0.0–1.0) en /traxxas/stop_throttle.
        pp_node lo mapea a PWM via PWMController.throttle_lerp().
        0.0 = brake_pwm (freno/neutro)
        1.0 = cruise_pwm (velocidad de crucero)
        """
        msg = Float32()
        msg.data = float(max(0.0, min(1.0, norm)))
        self._pub_throttle.publish(msg)
 
    def _set_active(self, active: bool):
        msg = Bool(); msg.data = active
        self._pub_active.publish(msg)
 
    def _set_done(self, done: bool):
        msg = Bool(); msg.data = done
        self._pub_done.publish(msg)
 
    def _send_feedback(self, goal_handle, state, stops_done,
                       throttle_norm, elapsed_wait=0.0):
        fb = StopRoutine.Feedback()
        fb.state              = state
        fb.stops_completed    = int(stops_done)
        fb.current_pwm        = int(throttle_norm * 10000)  # escala legacy
        fb.distance_to_sign_m = float(self._distance_m)
        fb.elapsed_wait_s     = float(elapsed_wait)
        goal_handle.publish_feedback(fb)
 
    # ── Execute callback — máquina de estados ─────────────────────────
 
    def _execute_cb(self, goal_handle):
        g = goal_handle.request
        total_stops  = int(g.total_stops)
        cruise_pwm   = float(g.cruise_pwm)   # puede ser legacy (2570) o normalizado (0.04)
        brake_pwm    = float(g.brake_pwm)
        prepare_dist = float(g.prepare_dist_m)
        stop_dist    = float(g.stop_dist_m)
        wait_time    = float(g.wait_time_s)
        brake_dur    = float(g.brake_duration_s)
        resume_dur   = float(g.resume_duration_s)
 
        # Compatibilidad: si el cliente manda PWM legacy (>1.0) usamos norm=1.0
        # El mapeo real lo hace pp_node con PWMController
        if cruise_pwm > 1.0:
            self.get_logger().warn(
                f'cruise_pwm={cruise_pwm:.0f} parece valor PWM legacy. '
                f'Usando throttle_norm=1.0 (cruise) como referencia.')
            cruise_norm = 1.0
        else:
            cruise_norm = cruise_pwm   # ya es normalizado
 
        self.get_logger().info(
            f'[ACTION] Goal: {total_stops} paradas | '
            f'cruise_norm={cruise_norm:.2f} | '
            f'prepare={prepare_dist:.2f} m  stop={stop_dist:.2f} m | '
            f'wait={wait_time:.1f} s | fuente={self._source}'
        )
 
        stops_completed  = 0
        state            = self.ST_SEARCHING
        current_norm     = cruise_norm
        maneuver         = None
        cooldown_start   = 0.0
 
        self._set_active(False)
        self._set_done(False)
 
        while rclpy.ok():
 
            # Cancelación
            if goal_handle.is_cancel_requested:
                self.get_logger().warn('[ACTION] Cancelado por el cliente.')
                self._set_active(False)
                goal_handle.canceled()
                return self._make_result(False, stops_completed, 'Cancelado')
 
            dist = self._distance_m
            v    = self._velocity
            now  = time.time()
 
            # ── SEARCHING ────────────────────────────────────────────
            if state == self.ST_SEARCHING:
                self._set_active(False)
                current_norm = cruise_norm
 
                if self._confirmed and 0.0 < dist <= prepare_dist:
                    self.get_logger().info(
                        f'[ACTION] Evento [{self._source}] a {dist:.2f} m → APPROACHING')
                    state = self.ST_APPROACHING
                    self._set_active(True)
 
            # ── APPROACHING ──────────────────────────────────────────
            elif state == self.ST_APPROACHING:
                self._set_active(True)
 
                if dist > 0.0:
                    if dist >= prepare_dist:
                        current_norm = cruise_norm
                    elif dist <= stop_dist:
                        current_norm = 0.0
                    else:
                        t = (dist - stop_dist) / max(prepare_dist - stop_dist, 1e-6)
                        # mínimo de aproximación: 60% del crucero
                        min_approach = cruise_norm * 0.6
                        current_norm = min_approach + t * (cruise_norm - min_approach)
 
                    self._pub_throttle_norm(current_norm)
 
                    if dist <= stop_dist:
                        self.get_logger().info(
                            f'[ACTION] dist={dist:.3f} m ≤ {stop_dist:.2f} m → BRAKING')
                        state = self.ST_BRAKING
                        maneuver = StopManeuver(
                            start_pwm  = int(current_norm * 1000),  # escala interna
                            brake_pwm  = 0,
                            cruise_pwm = int(cruise_norm * 1000),
                            brake_dur  = brake_dur,
                            wait_s     = wait_time,
                            resume_dur = resume_dur,
                        )
                        maneuver.start()
                else:
                    # Sin distancia válida: avanzar lento
                    current_norm = cruise_norm * 0.6
                    self._pub_throttle_norm(current_norm)
 
                # ── Latch de punto ciego ───────────────────────────
                # Si la cámara pierde la señal en APPROACHING, es porque
                # el vehículo ya está muy cerca (señal fuera del encuadre).
                # Forzar BRAKING en lugar de volver a SEARCHING.
                if not self._confirmed:
                    self.get_logger().warn(
                        f'[ACTION] Señal [{self._source}] perdida en punto ciego '
                        f'→ Forzando BRAKING a ciegas')
                    state = self.ST_BRAKING
                    maneuver = StopManeuver(
                        start_pwm  = int(current_norm * 1000),
                        brake_pwm  = 0,
                        cruise_pwm = int(cruise_norm * 1000),
                        brake_dur  = brake_dur,
                        wait_s     = wait_time,
                        resume_dur = resume_dur,
                    )
                    maneuver.start()
 
            # ── BRAKING / STOPPED / RESUMING ─────────────────────────
            elif state in (self.ST_BRAKING, self.ST_STOPPED, self.ST_RESUMING):
                # tick devuelve (pwm_int, phase_str) — convertir a norm
                raw_pwm, maneuver_phase = maneuver.tick(v)
                state = maneuver_phase
 
                # Convertir PWM interno a norm (0.0–1.0)
                cruise_int = int(cruise_norm * 1000)
                current_norm = raw_pwm / max(cruise_int, 1)
                current_norm = max(0.0, min(1.0, current_norm))
 
                if state in (self.ST_BRAKING, self.ST_STOPPED, self.ST_RESUMING):
                    self._pub_throttle_norm(current_norm)
 
                    if state == self.ST_STOPPED:
                        self.get_logger().info(
                            f'[ACTION] Detenido. '
                            f'Esperando {wait_time:.1f}s... '
                            f'({maneuver.wait_elapsed:.1f}s)',
                            throttle_duration_sec=1.0
                        )
                    elif state == self.ST_RESUMING:
                        self.get_logger().info(
                            '[ACTION] Detenido confirmado (encoder). Arrancando...',
                            throttle_duration_sec=1.0
                        )
 
                if maneuver.done:
                    stops_completed += 1
                    self.get_logger().info(
                        f'[ACTION] Maniobra completa. '
                        f'Paradas: {stops_completed}/{total_stops} | v={v:.3f} m/s'
                    )
                    state          = self.ST_COOLDOWN
                    cooldown_start = now
 
            # ── COOLDOWN ─────────────────────────────────────────────
            elif state == self.ST_COOLDOWN:
                self._set_active(False)
                current_norm = cruise_norm
 
                elapsed_cd = now - cooldown_start
                sign_gone  = (not self._confirmed) or (dist < 0.0)
 
                if elapsed_cd >= self.COOLDOWN_MIN_S and sign_gone:
                    if stops_completed >= total_stops:
                        self._set_done(True)
                        self.get_logger().info(
                            f'[ACTION] Misión completa: {stops_completed} paradas.')
                        goal_handle.succeed()
                        return self._make_result(
                            True, stops_completed,
                            f'Misión completada: {stops_completed} eventos procesados.')
                    else:
                        self.get_logger().info(
                            f'[ACTION] Cooldown OK → SEARCHING '
                            f'(faltan {total_stops - stops_completed} paradas)')
                        state = self.ST_SEARCHING
 
            # ── Feedback y sleep ──────────────────────────────────────
            self._send_feedback(
                goal_handle, state, stops_completed, current_norm,
                elapsed_wait=maneuver.wait_elapsed if maneuver else 0.0,
            )
            time.sleep(self.LOOP_DT)
 
        goal_handle.abort()
        return self._make_result(False, stops_completed, 'rclpy shutdown durante ejecución')
 
    @staticmethod
    def _make_result(success: bool, stops: int, msg: str) -> StopRoutine.Result:
        r = StopRoutine.Result()
        r.success         = success
        r.stops_completed = stops
        r.message         = msg
        return r
 
 
def main(args=None):
    rclpy.init(args=args)
    node = StopRoutineActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Asegurar que pp_node retome el throttle normal al cerrar
        try:
            norm_msg = Float32(); norm_msg.data = 1.0
            node._pub_throttle.publish(norm_msg)
            active_msg = Bool(); active_msg.data = False
            node._pub_active.publish(active_msg)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()