#!/usr/bin/env python3
"""
=======================================================================
 Pure Pursuit Controller  -  Traxxas fisico con ROS2
 Controlador de seguimiento de carril con Pure Pursuit.
 Se integra con StopRoutineActionServer via /traxxas/stop_sign/active.

 Coordinacion con el action server
 ----------------------------------
   stop_active = False  →  pp_node publica throttle (crucero) + steering
   stop_active = True   →  pp_node publica steering con el throttle
                           que el action server indique via /traxxas/stop_throttle
                           (garantiza que el watchdog no dispare por silencio)

 Logica de throttle
 ------------------
   Usa PWMController para publicar el PWM correcto en cada caso:
     - Normal (stop_active=False): publica th_max en cada tick
     - Cedido (stop_active=True): no publica los valores que le indica el server

 Topicos suscritos
 -----------------
   /pose_traxxas                  Vector3Stamped  (x, y, theta)
   /traxxas/stop_sign/active      Bool
   /traxxas/stop_throttle         Float32  (0.0=freno, 1.0=crucero)

 Topicos publicados
 ------------------
   /direction_servo               String (uint16 PWM) - SIEMPRE
   /throttle_motor                String (uint16 PWM) - solo si !stop_active
=======================================================================
"""

import math
import csv
import numpy as np
import matplotlib.pyplot as plt
import rclpy

from pathlib import Path
from datetime import datetime
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Bool, Float32, String
from traxxas_stop.pwm_controller import PWMController


# ======================================================================
#  ESTADO DEL VEHICULO
# ======================================================================

class PPState:
    def __init__(self):
        self.xr    = 0.0
        self.yr    = 0.0
        self.theta = 0.0
        self.v     = 0.0

    def calc_distance(self, px, py):
        return math.hypot(self.xr - px, self.yr - py)

    def update(self, x, y, theta, v):
        self.xr    = x
        self.yr    = y
        self.theta = theta
        self.v     = v


# ======================================================================
#  TRAYECTORIA + INDICE OBJETIVO (loop infinito)
# ======================================================================

class TargetCourse:
    def __init__(self, path_points, k_gain, base_lookahead):
        self.cx  = [p[0] for p in path_points]
        self.cy  = [p[1] for p in path_points]
        self.k   = k_gain
        self.Lfc = base_lookahead
        self.old_nearest_point_index = None

    def search_target_index(self, state: PPState):
        N = len(self.cx)
        if self.old_nearest_point_index is None:
            dists = [math.hypot(state.xr - cx_i, state.yr - cy_i)
                     for cx_i, cy_i in zip(self.cx, self.cy)]
            ind = int(np.argmin(dists))
            self.old_nearest_point_index = ind
        else:
            ind    = self.old_nearest_point_index
            d_this = state.calc_distance(self.cx[ind], self.cy[ind])
            for _ in range(N):
                next_ind = (ind + 1) % N
                d_next   = state.calc_distance(self.cx[next_ind], self.cy[next_ind])
                if d_this < d_next:
                    break
                ind    = next_ind
                d_this = d_next
            self.old_nearest_point_index = ind

        Lf         = self.k * max(state.v, 0.0) + self.Lfc
        target_ind = ind
        for _ in range(N):
            if state.calc_distance(self.cx[target_ind], self.cy[target_ind]) >= Lf:
                break
            target_ind = (target_ind + 1) % N

        return target_ind, Lf


# ======================================================================
#  NODO PRINCIPAL
# ======================================================================

class PurePursuitTraxxas(Node):

    def __init__(self):
        super().__init__('pure_pursuit_traxxas')

        # -- Parametros ------------------------------------------------
        self.declare_parameter('path_csv',       '')
        self.declare_parameter('circle_radius',  0.8)
        self.declare_parameter('circle_points',  300)
        self.declare_parameter('lookahead',      0.15)
        self.declare_parameter('k_gain',         0.5)
        self.declare_parameter('v_ref',          0.5)
        self.declare_parameter('wheelbase',      0.324)

        self.declare_parameter('throttle_center', 2457)
        self.declare_parameter('throttle_max',    2680)
        self.declare_parameter('throttle_min',    1638)

        self.declare_parameter('dir_center',    2642)
        self.declare_parameter('dir_max_right', 3276)
        self.declare_parameter('dir_min_left',  1669)
        self.declare_parameter('max_steer_rad', 0.349)  # 20 grados en radianes

        self.lookahead = self.get_parameter('lookahead').value
        self.k_gain    = self.get_parameter('k_gain').value
        self.v_ref     = self.get_parameter('v_ref').value
        self.L         = self.get_parameter('wheelbase').value

        th_center = int(self.get_parameter('throttle_center').value)
        th_max    = int(self.get_parameter('throttle_max').value)
        th_min    = int(self.get_parameter('throttle_min').value)

        dir_center    = int(self.get_parameter('dir_center').value)
        dir_max       = int(self.get_parameter('dir_max_right').value)
        dir_min       = int(self.get_parameter('dir_min_left').value)
        max_steer_rad = float(self.get_parameter('max_steer_rad').value)

        # -- PWMController (logica de PWM centralizada) ----------------
        self.pwm = PWMController(
            brake_pwm    = th_center,
            cruise_pwm   = th_max,
            max_pwm      = th_max,
            dir_center   = dir_center,
            dir_max      = dir_max,
            dir_min      = dir_min,
            max_steer_rad= max_steer_rad,
        )
        self.th_center = th_center   # para uso en main (shutdown seguro)

        # -- Estado ----------------------------------------------------
        self.state           = PPState()
        self.current_pose    = None
        self.target_ind      = 0
        self.last_logged_ind = None
        self.stop_active     = False

        # Throttle normalizado recibido del action server (0.0–1.0)
        # 0.0 = freno (brake_pwm), 1.0 = crucero (cruise_pwm)
        self.stop_throttle_norm = 0.0

        # -- Logs para analisis post-ejecucion --------------------------------
        self.generated_trajectory = []
        self.log_t   = []
        self.log_yaw = []
        self.log_v   = []
        self.start_time = self.get_clock().now()

        # -- Path ------------------------------------------------------
        path_csv  = self.get_parameter('path_csv').value.strip()
        self.path = self.load_path(path_csv) if path_csv else self.generate_circle_path()

        if not self.path:
            self.get_logger().error('Path vacio. Abortando.')
            return

        self.target_course = TargetCourse(self.path, self.k_gain, self.lookahead)
        self.get_logger().info(f'Path listo: {len(self.path)} puntos')

        # -- Publicadores ----------------------------------------------
        self.pub_throttle  = self.create_publisher(String, '/throttle_motor',  10)
        self.pub_direction = self.create_publisher(String, '/direction_servo',  10)

        # -- Suscripciones ---------------------------------------------
        self.create_subscription(
            Vector3Stamped, '/pose_traxxas',
            self.pose_callback, 10)
        self.create_subscription(
            Bool, '/traxxas/stop_sign/active',
            self.stop_active_callback, 10)
        # Throttle normalizado del action server durante la maniobra
        self.create_subscription(
            Float32, '/traxxas/stop_throttle',
            self.stop_throttle_callback, 10)

        # -- Valores iniciales (neutro) --------------------------------
        self._pub_throttle_val(self.pwm.throttle_brake())
        self._pub_direction_val(self.pwm.steer_center())

        # -- Timer 50 Hz -----------------------------------------------
        self.timer = self.create_timer(0.02, self.control_loop)
        self.get_logger().info(
            f'Pure Pursuit iniciado | '
            f'brake={self.pwm.brake_pwm} cruise/max={self.pwm.cruise_pwm} | '
            f'dir [{self.pwm.dir_min}~{self.pwm.dir_center}~{self.pwm.dir_max}]'
        )

    # -- Callbacks -----------------------------------------------------

    def pose_callback(self, msg: Vector3Stamped):
        self.current_pose = msg

    def stop_active_callback(self, msg: Bool):
        prev = self.stop_active
        self.stop_active = bool(msg.data)
        if self.stop_active != prev:
            self.get_logger().warn(
                'stop_active → ' + (
                    'ACTIVO (action server controla throttle)'
                    if self.stop_active else
                    'INACTIVO (throttle retomado por pp_node)')
            )
 
    def stop_throttle_callback(self, msg: Float32):
        """
        Recibe throttle normalizado del action server (0.0–1.0).
        0.0 = brake_pwm, 1.0 = cruise_pwm.
        """
        self.stop_throttle_norm = float(msg.data)

    # -- Bucle de control ----------------------------------------------

    def control_loop(self):
        if self.current_pose is None:
            return
 
        x   = self.current_pose.vector.x
        y   = self.current_pose.vector.y
        yaw = self.current_pose.vector.z
        self.state.update(x, y, yaw, self.v_ref)
 
        # Dirección: Pure Pursuit SIEMPRE
        delta = self.compute_pure_pursuit_delta()
        for pwm_val in self.pwm.steer_to_pwm(delta):
            self._pub_direction_val(pwm_val)
 
        if self.stop_active:
            # Publicar el throttle que el action server indicó
            # (el watchdog sigue recibiendo comandos → no dispara)
            throttle_pwm = self.pwm.throttle_lerp(self.stop_throttle_norm)
            self._pub_throttle_val(throttle_pwm)
        else:
            # Crucero normal
            self._pub_throttle_val(self.pwm.throttle_max_val())
 
            # Log solo cuando pp_node tiene el control
            now = self.get_clock().now()
            t = (now.nanoseconds - self.start_time.nanoseconds) * 1e-9
            self.generated_trajectory.append((x, y))
            self.log_t.append(t)
            self.log_yaw.append(yaw)
            self.log_v.append(self.v_ref)
 
    #  -- Pure Pursuit ----------------------------------------------
 
    def compute_pure_pursuit_delta(self):
        ind, Lf = self.target_course.search_target_index(self.state)
        N = len(self.path)
 
        if (self.target_ind - ind) % N < (ind - self.target_ind) % N:
            ind = self.target_ind
        self.target_ind = ind
 
        tx = self.target_course.cx[ind]
        ty = self.target_course.cy[ind]
 
        if ind != self.last_logged_ind:
            self.last_logged_ind = ind
            self.get_logger().info(
                f'idx={ind} | ({tx:.3f},{ty:.3f}) '
                f'| dist={self.state.calc_distance(tx,ty):.3f} m | Lf={Lf:.3f} m'
            )
 
        alpha = math.atan2(ty - self.state.yr, tx - self.state.xr) - self.state.theta
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))
        return math.atan2(2.0 * self.L * math.sin(alpha), max(Lf, 1e-3))
 
    # -- Publicadores PWM ----------------------------------------------
 
    def _pub_throttle_val(self, value: int):
        msg = String(); msg.data = str(int(value))
        self.pub_throttle.publish(msg)
 
    def _pub_direction_val(self, value: int):
        msg = String(); msg.data = str(int(value))
        self.pub_direction.publish(msg)
 
    # -- Path helpers ----------------------------------------------
 
    def load_path(self, csv_file: str):
        p = Path(csv_file)
        if not p.exists():
            self.get_logger().error(f'CSV no encontrado: {csv_file}')
            return []
        points = []
        with p.open('r') as f:
            for row in csv.DictReader(f):
                try:
                    points.append((float(row['x']), float(row['y'])))
                except (KeyError, ValueError):
                    continue
        self.get_logger().info(f'CSV cargado: {len(points)} puntos')
        return points
 
    def generate_circle_path(self):
        R = float(self.get_parameter('circle_radius').value)
        N = max(3, int(self.get_parameter('circle_points').value))
        pts = [
            (R * math.cos(2 * math.pi * i / N),
             R + R * math.sin(2 * math.pi * i / N))
            for i in range(N)
        ]
        self.get_logger().info(f'Círculo generado: R={R:.3f} m, N={N} pts')
        return pts
 
    # -- Análisis al cierre ----------------------------------------------
 
    def find_closest_point(self, point):
        min_d, closest = float('inf'), None
        for ref in self.path:
            d = math.hypot(point[0] - ref[0], point[1] - ref[1])
            if d < min_d:
                min_d, closest = d, ref
        return min_d, closest
 
    def calculate_tracking_error(self):
        if not self.generated_trajectory or not self.path:
            return 0.0, 0.0, []
        errors = [self.find_closest_point(p)[0] for p in self.generated_trajectory]
        return float(np.mean(errors)), float(np.max(errors)), errors
 
    def report_performance(self, avg, max_e, errors):
        std = float(np.std(errors))
        s5  = sum(1 for e in errors if e <= 0.05) / len(errors) * 100.0
        s10 = sum(1 for e in errors if e <= 0.10) / len(errors) * 100.0
        tag = ('EXCELENTE' if avg < 0.05 else 'BUENO' if avg < 0.10
               else 'ACEPTABLE' if avg < 0.20 else 'DEFICIENTE')
        print('\n======== EVALUACIÓN DE TRAYECTORIA ========')
        print(f'Error promedio:      {avg:.3f} m')
        print(f'Error máximo:        {max_e:.3f} m')
        print(f'Desviación estándar: {std:.3f} m')
        print(f'% dentro de  5 cm:   {s5:.1f}%')
        print(f'% dentro de 10 cm:   {s10:.1f}%')
        print(f'Referencia:          {len(self.path)} pts')
        print(f'Trayectoria real:    {len(self.generated_trajectory)} pts')
        print(f'Evaluación general:  {tag}')
        print('============================================\n')
 
    def save_trajectory_csv(self, errors, out_dir, ts, tag):
        csv_path = out_dir / f'trayectoria_{tag}_{ts}.csv'
        with csv_path.open('w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['index', 't', 'x_real', 'y_real', 'yaw', 'v_ref',
                        'x_ref', 'y_ref', 'error'])
            for i, (xr, yr) in enumerate(self.generated_trajectory):
                t   = self.log_t[i]   if i < len(self.log_t)   else ''
                yaw = self.log_yaw[i] if i < len(self.log_yaw) else ''
                v   = self.log_v[i]   if i < len(self.log_v)   else ''
                err = errors[i]       if i < len(errors)        else ''
                _, cl = self.find_closest_point((xr, yr))
                xc, yc = (cl[0], cl[1]) if cl else ('', '')
                w.writerow([i, t, xr, yr, yaw, v, xc, yc, err])
        print(f'CSV guardado: {csv_path}')
 
    def plot_results(self):
        if not self.path or not self.generated_trajectory:
            print('Sin datos para graficar.')
            return
 
        avg_e, max_e, errors = self.calculate_tracking_error()
        if errors:
            self.report_performance(avg_e, max_e, errors)
 
        out_dir = (Path.home() / 'Workspaces' / 'traxxas_ws'
                   / 'src' / 'traxxas_stop' / 'resultados_pure_pursuit')
        out_dir.mkdir(parents=True, exist_ok=True)
 
        ts  = datetime.now().strftime('%Y%m%d_%H%M%S')
        tag = f'L{self.lookahead}_k{self.k_gain}'
 
        exp_x = [p[0] for p in self.path]
        exp_y = [p[1] for p in self.path]
        gen_x = [p[0] for p in self.generated_trajectory]
        gen_y = [p[1] for p in self.generated_trajectory]
 
        if self.log_t:
            fig, axes = plt.subplots(2, 2, figsize=(12, 10))
 
            ax = axes[0, 0]
            ax.plot(exp_x, exp_y, '-r', lw=2, label='Referencia')
            ax.plot(gen_x, gen_y, '-b', lw=1.5, label='Real')
            if len(gen_x) > 1:
                ax.scatter(gen_x[0], gen_y[0], s=120, c='green', marker='o', label='Inicio')
                ax.scatter(gen_x[-1], gen_y[-1], s=120, c='red', marker='x', label='Final')
            ax.set(xlabel='X [m]', ylabel='Y [m]', title='Trayectorias')
            ax.legend(); ax.grid(True); ax.axis('equal')
 
            if errors:
                ax = axes[0, 1]
                ax.plot(self.log_t[:len(errors)], errors, '-g', lw=1.5)
                ax.axhline(avg_e, linestyle='--', label=f'Prom: {avg_e:.3f} m')
                ax.set(xlabel='Tiempo [s]', ylabel='Error [m]', title='Error vs tiempo')
                ax.legend(); ax.grid(True)
 
            axes[1, 0].plot(self.log_t, self.log_yaw, '-m', lw=1.5)
            axes[1, 0].set(xlabel='Tiempo [s]', ylabel='Yaw [rad]', title='Orientación')
            axes[1, 0].grid(True)
 
            axes[1, 1].plot(gen_x, gen_y, '-b', lw=1, alpha=0.6)
            axes[1, 1].set(xlabel='X [m]', ylabel='Y [m]', title='Trayectoria real (detalle)')
            axes[1, 1].grid(True); axes[1, 1].axis('equal')
 
            plt.tight_layout()
            fig_path = out_dir / f'analisis_{ts}_{tag}.png'
            fig.savefig(fig_path, dpi=300, bbox_inches='tight')
            plt.close(fig)
            print(f'Gráficas guardadas: {fig_path}')
 
        self.save_trajectory_csv(errors, out_dir, ts, tag)
 
 
def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitTraxxas()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._pub_throttle_val(node.pwm.throttle_brake())
            node._pub_direction_val(node.pwm.steer_center())
        except Exception:
            pass
        node.plot_results()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()