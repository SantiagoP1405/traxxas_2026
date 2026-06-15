import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data

import cv2
import numpy as np
import math
import time
import signal
from concurrent.futures import ThreadPoolExecutor

import torch.nn.functional as F
from ultralytics import YOLO

# ─────────────────────────────────────────────
# CONFIGURACIÓN GENERAL Y DE CARRERA
# ─────────────────────────────────────────────
MODEL_PATH     = "/home/edwin/workspace/traxxas_ws/src/traxxas_laptop/traxxas_laptop/best.engine"
CURVE_LOG_PATH = "/home/edwin/workspace/traxxas_ws/src/traxxas_laptop/traxxas_laptop/curve_debug.txt" 

CONF        = 0.35
COLOR_LEFT  = (0,   255, 100)
COLOR_RIGHT = (0,   180, 255)
ALPHA       = 0.45
SHOW_VIZ    = True
DEBUG_FILE  = True  # Activa la escritura del .txt

# Parámetros de Fusión
CURVE_THRESHOLD_DEG = 5.0    
SLOPE_MAX_DEG       = 30.0   
SLOPE_SIGN          = -1.0   
CONSENSUS_TOL_DEG   = 2.0    
W_STRAIGHT = np.array([0.60, 0.25, 0.15])  
W_CURVE    = np.array([0.30, 0.30, 0.40])   
CURVE_POST_LINES    = 10   # cuántas líneas guardar al salir de la curva

# ─────────────────────────────────────────────
# FUNCIONES DE PROCESAMIENTO VISUAL Y MATEMÁTICO
# ─────────────────────────────────────────────
def warp(img, src, dst):
    M = cv2.getPerspectiveTransform(src.astype(np.float32), dst.astype(np.float32))
    return cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))

def yolo_mask_to_binary(result, target_h, target_w):
    if result.masks is None:
        return np.zeros((target_h, target_w), dtype=np.uint8)
    masks_gpu = result.masks.data                    
    if masks_gpu.shape[0] == 0:                      
        return np.zeros((target_h, target_w), dtype=np.uint8)
    combined  = masks_gpu.max(dim=0).values          
    resized = F.interpolate(
        combined.float().unsqueeze(0).unsqueeze(0),
        size=(target_h, target_w),
        mode='bilinear',
        align_corners=False
    ).squeeze()
    return (resized > 0.5).byte().cpu().numpy() * 255

def process_eye(binary_mask, vertices, dst):
    roi_mask = np.zeros_like(binary_mask, dtype=np.uint8)
    cv2.fillPoly(roi_mask, vertices, 255)
    masked = cv2.bitwise_and(binary_mask, binary_mask, mask=roi_mask)

    warped = warp(masked, vertices[0], dst)
    sliding_img, fit, ploty = sliding_window(warped)
    curve = measure_curvature(ploty, fit) if (fit is not None and ploty is not None) else None

    h_warped = warped.shape[0]
    warped_bottom = warped[int(h_warped * 0.4):, :]  
    return sliding_img, warped, warped_bottom, roi_mask, curve

def sliding_window(binary_warped):
    histogram = np.sum(binary_warped, axis=0)
    histogram = cv2.GaussianBlur(histogram.astype(np.float32), (25, 1), 0)
    base = np.argmax(histogram)

    nwindows      = 12
    window_height = np.int32(binary_warped.shape[0] / nwindows)
    nonzero       = binary_warped.nonzero()
    nonzeroy      = np.array(nonzero[0])
    nonzerox      = np.array(nonzero[1])
    current_x     = base
    margin        = 60
    minpix        = 3
    lane_inds     = []
    out_img       = np.dstack((binary_warped, binary_warped, binary_warped)) * 255

    MIN_LANE_PIXELS = 100   
    MIN_WINDOWS_HIT = 7     
    windows_with_data = 0   

    for window in range(nwindows):
        win_y_low  = binary_warped.shape[0] - (window + 1) * window_height
        win_y_high = binary_warped.shape[0] - window * window_height
        win_low    = current_x - margin
        win_high   = current_x + margin

        cv2.rectangle(out_img, (win_low, win_y_low), (win_high, win_y_high), (0, 255, 0), 2)
        good_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_low) & (nonzerox < win_high)).nonzero()[0]
        lane_inds.append(good_inds)

        if len(good_inds) > minpix:
            current_x = int(np.mean(nonzerox[good_inds]))
            windows_with_data += 1   

    lane_inds = np.concatenate(lane_inds)
    x = nonzerox[lane_inds]
    y = nonzeroy[lane_inds]

    if len(x) < MIN_LANE_PIXELS or windows_with_data < MIN_WINDOWS_HIT:
        return out_img, None, None

    fit = np.polyfit(y, x, 2)
    if abs(fit[0]) > 0.01:
        return out_img, None, None

    ploty = np.linspace(0, binary_warped.shape[0] - 1, binary_warped.shape[0])
    fitx  = fit[0] * ploty**2 + fit[1] * ploty + fit[2]

    out_img[nonzeroy[lane_inds], nonzerox[lane_inds]] = [255, 0, 0]
    for i in range(len(ploty)):
        cv2.circle(out_img, (int(fitx[i]), int(ploty[i])), 3, (255, 255, 0), -1)

    return out_img, fit, ploty

def slope_angle_from_warped(warped_mask: np.ndarray):
    ny, nx = warped_mask.nonzero()
    if len(nx) < 30: return None, None
    a, b = np.polyfit(ny.astype(np.float32), nx.astype(np.float32), 1)
    angle = float(np.degrees(np.arctan(a)))
    return angle, (a, b)

def slope_to_steering(slope_left, slope_right, max_steer_deg=25.0):
    angles = [a for a in (slope_left, slope_right) if a is not None]
    if not angles: return None
    mean_slope = float(np.mean(angles))
    norm = np.clip(mean_slope / SLOPE_MAX_DEG, -1.0, 1.0)
    return SLOPE_SIGN * norm * max_steer_deg

def measure_curvature(ploty, fit):
    ym_per_pix = 1.6  / 720
    xm_per_pix = 0.40 / 384  
    y_eval = np.max(ploty)
    fitx   = fit[0] * ploty**2 + fit[1] * ploty + fit[2]
    fit_cr = np.polyfit(ploty * ym_per_pix, fitx * xm_per_pix, 2)
    curverad = ((1 + (2 * fit_cr[0] * y_eval * ym_per_pix + fit_cr[1])**2)**1.5) / np.absolute(2 * fit_cr[0])
    return curverad

def steering_from_curvature(left_curvature, right_curvature):
    L = 0.18
    if left_curvature is not None and right_curvature is not None: R = min(left_curvature, right_curvature)
    elif left_curvature is not None: R = left_curvature
    elif right_curvature is not None: R = right_curvature
    else: return None
    return math.degrees(math.atan(L / R))

def signed_delta_from_curvature(left_curve, right_curve):
    if left_curve is None or right_curve is None: return None
    delta = steering_from_curvature(left_curve, right_curve)
    return -abs(delta) if right_curve > left_curve else abs(delta)

def delta_to_pwm(delta_deg: float, max_steer_deg: float = 16.0) -> int:
    pwm_min, pwm_center, pwm_max = 1669, 2525, 3276
    delta_deg = float(np.clip(delta_deg, -max_steer_deg, max_steer_deg))
    norm = delta_deg / max_steer_deg
    if norm >= 0: return int(pwm_center + norm * (pwm_max - pwm_center))
    return int(pwm_center + norm * (pwm_center - pwm_min))

def fuse_steering_angles(sliding_deg, slope_deg, prev_deg):
    estimates = [sliding_deg, slope_deg, prev_deg]
    available = np.array([e is not None for e in estimates], dtype=float)
    vals      = np.array([e if e is not None else 0.0 for e in estimates])

    n = int(available.sum())
    if n == 0: return 0.0, np.zeros(3), 0.0
    if n == 1:
        idx = int(np.argmax(available))
        return float(vals[idx]), available / available.sum(), 0.0

    ref_angle   = prev_deg if abs(prev_deg) > 0.1 else (sliding_deg or 0.0)
    curve_level = float(np.clip(abs(ref_angle) / CURVE_THRESHOLD_DEG, 0.0, 1.0))

    weights = (1.0 - curve_level) * W_STRAIGHT + curve_level * W_CURVE
    weights *= available          
    weights /= weights.sum()

    return float(np.dot(weights, vals)), weights, curve_level


#para calcular el centroide del carril y su offset respecto al centro del coche, 
# lo que da una indicación de cuánto el coche está desplazado lateralmente dentro del carril. 
def lateral_offset(warped_left, warped_right, img_width):
    """
    Equivalente a la distancia D del paper (ec. 14),
    pero adaptado a dos carriles en vista cenital.
    """
    ny_l, nx_l = warped_left.nonzero()
    ny_r, nx_r = warped_right.nonzero()
    
    if len(nx_l) < 30 or len(nx_r) < 30:
        return None
    
    # Solo la zona baja (cerca del coche) — como el "primer punto" del paper
    h = warped_left.shape[0]
    bottom_l = ny_l > (h * 0.8)
    bottom_r = ny_r > (h * 0.8)
    
    if bottom_l.sum() < 10 or bottom_r.sum() < 10:
        return None
    
    left_x = np.mean(nx_l[bottom_l])
    right_x = np.mean(nx_r[bottom_r])
    
    lane_center = (left_x + right_x) / 2.0
    car_center = img_width / 2.0
    
    # Positivo = coche desplazado a la izquierda, necesita girar a la derecha
    return lane_center - car_center

# ─────────────────────────────────────────────
# NODO ROS 2 (EL CEREBRO EN LA LAPTOP)
# ─────────────────────────────────────────────
class LaneDetectorNode(Node):
    def __init__(self):
        super().__init__('laptop_brain_node')

        # Utilizamos el perfil Best Effort (sensor_data) para video, 
        # y Reliable para los comandos de motor.
        qos_motor = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.img_sub = self.create_subscription(CompressedImage, '/zed/stereo/compressed', self.image_callback, qos_profile_sensor_data)
        self.direction_pwm_pub = self.create_publisher(String, 'direction_servo', qos_motor)
        self.throttle_pwm_pub  = self.create_publisher(String, 'throttle_motor',  qos_motor)

        self.get_logger().info("Cargando modelo YOLO en la Laptop...")
        self.model = YOLO(MODEL_PATH, task="segment")
        self.get_logger().info("Modelo listo.")

        self.executor_cv = ThreadPoolExecutor(max_workers=2)
        
        # Variables de estado y telemetría
        self.prev_steering_deg = 0.0
        self.prev_sliding_sign = 1.0  # +1 o -1, para detectar cambios de dirección en sliding
        self.in_curve_mode      = False
        self.post_curve_counter = 0
        self.start_time = time.time()
        
        if DEBUG_FILE:
            self._log_file = open(CURVE_LOG_PATH, "a", encoding="utf-8")
            self.get_logger().info(f"Guardando log de telemetría en: {CURVE_LOG_PATH}")

    def destroy_node(self):
        if DEBUG_FILE:
            self._log_file.close()
        super().destroy_node()

    def image_callback(self, msg):
        start_time = time.time()
        
        # 1. Descomprimir imagen estéreo
        np_arr = np.frombuffer(msg.data, np.uint8)
        stereo_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        # 2. Separar Ojos
        h_total, w_total = stereo_img.shape[:2]
        w = w_total // 2 
        img_left = stereo_img[:, :w]
        img_right = stereo_img[:, w:]

        # 3. Inferencia YOLO
        results = self.model([img_left, img_right], conf=CONF, half=True, imgsz=640, retina_masks=True, verbose=False)

        # 4. Upscale a 720p
        mask_left = yolo_mask_to_binary(results[0], h_total, w)
        mask_right = yolo_mask_to_binary(results[1], h_total, w)

        # 5. Definir Vertices
        vertices_l = np.array([[(int(0.16*w), int(0.56*h_total)), (int(0.47*w), 
                                int(0.56*h_total)), (int(0.55*w), 
                                h_total), (int(0.08*w), h_total)]])
        
        vertices_r = np.array([[(int(0.64*w), int(0.56*h_total)), (int(0.95*w), int(0.56*h_total)), (int(1.0*w), h_total), (int(0.56*w), h_total)]])
        dst = np.array([[int(0.35*w), 0], [int(0.65*w), 0], [int(0.65*w), h_total], [int(0.35*w), h_total]])

        
        # 6. Procesamiento en Paralelo
        fut_l = self.executor_cv.submit(process_eye, mask_left, vertices_l, dst)
        fut_r = self.executor_cv.submit(process_eye, mask_right, vertices_r, dst)

        sliding_img_left, warped_left, warped_bottom_l, roi_l, left_curve = fut_l.result()
        sliding_img_right, warped_right, warped_bottom_r, roi_r, right_curve = fut_r.result()

        # 7. Cálculos y Fusión
        sliding_delta = signed_delta_from_curvature(left_curve, right_curve)
        SLIDING_GAIN = 1
        if sliding_delta is not None:
            # Ambos ojos → actualizar signo y usar normalmente
            self.prev_sliding_sign = 1.0 if sliding_delta >= 0 else -1.0
            sliding_delta *= SLIDING_GAIN
        elif left_curve is not None or right_curve is not None:
            # Un solo ojo → calcular magnitud con lo que hay, signo del anterior
            R = left_curve if left_curve is not None else right_curve
            delta_mag = abs(math.degrees(math.atan(0.18 / R)))
            sliding_delta = self.prev_sliding_sign * delta_mag * SLIDING_GAIN

        slope_left, fit_line_l = slope_angle_from_warped(warped_bottom_l)
        slope_right, fit_line_r = slope_angle_from_warped(warped_bottom_r)
        slope_delta = slope_to_steering(slope_left, slope_right)

        prev_delta = self.prev_steering_deg
        fused_delta, weights, curve_level = fuse_steering_angles(sliding_delta, slope_delta, prev_delta)
        self.prev_steering_deg = fused_delta
        offset = lateral_offset(warped_left, warped_right, w)
        OFFSET_GAIN = 0.05
        OFFSET_BIAS = 1.5 #ajuste de que el offset si sea el verdadero centro respecto el coche
          # grados por píxel — calibrar empíricamente
        if offset is not None:
            offset_correction = np.clip(offset * OFFSET_GAIN + OFFSET_BIAS, -5.0, 5.0)
            fused_delta += offset_correction
        else:
            offset_correction = 0.0

        # ── LOGICA DE SUAVIZADsO RESTAURADA (Decay Rate) ──
        # DECAY_RATE = 1
        # if abselfs(fused_delta) < CURVE_THRESHOLD_DEG:
        #     self.prev_steering_deg = fused_delta * DECAY_RATE
        # else:
        
        

        # 8. Convertir a PWM y Publicar
        if sliding_delta is None and slope_delta is None:
            pwm = 2642
        else:
            pwm = delta_to_pwm(fused_delta)

        pwm_msg = String()
        pwm_msg.data = str(pwm)
        self.direction_pwm_pub.publish(pwm_msg)

        
        throttle_msg = String()
        throttle_msg.data = str(2700)
        if time.time() - self.start_time < 6.0:
            pwm = 2642  # centro, sin moverse
            throttle_msg.data = str(2457)  # neutro

        self.throttle_pwm_pub.publish(throttle_msg)

        # 9. TELEMETRÍA: Archivo TXT y Consola
        is_curve = abs(fused_delta) > CURVE_THRESHOLD_DEG

        if DEBUG_FILE:
            # Detección de transiciones
            if is_curve and not self.in_curve_mode:
                self.in_curve_mode = True
                self.post_curve_counter = 0
                header = (
                    "\n" + "=" * 90 + "\n"
                    f"  ENTRANDO EN MODO CURVA  —  fused={fused_delta:+.1f}°  threshold={CURVE_THRESHOLD_DEG}°\n"
                    + "=" * 90 + "\n"
                    f"{'timestamp':>10}  {'slide':>8}  {'slope':>8}  {'prev':>8}  "
                    f"{'fused':>8}  {'PWM':>5}  {'curva':>6}  "
                    f"{'w_sld':>6}  {'w_slp':>6}  {'w_prv':>6}\n"
                    + "-" * 90 + "\n"
                )
                self._log_file.write(header)
                self._log_file.flush()

            elif not is_curve and self.in_curve_mode:
                self.in_curve_mode = False
                self.post_curve_counter = CURVE_POST_LINES
                footer = "-" * 90 + "\n  SALIENDO DE MODO CURVA  — guardando últimas líneas\n"
                self._log_file.write(footer)
                self._log_file.flush()

            should_log = is_curve or (self.post_curve_counter > 0)
            if should_log:
                ts = time.strftime("%H:%M:%S")
                line = (
                    f"{ts:>10}  "
                    f"{sliding_delta or 0.0:>+8.1f}  "
                    f"{slope_delta   or 0.0:>+8.1f}  "
                    f"{prev_delta:>+8.1f}  "
                    f"{fused_delta:>+8.1f}  "
                    f"{pwm:>5}  "
                    f"{curve_level:>6.2f}  "
                    f"{weights[0]:>6.2f}  "
                    f"{weights[1]:>6.2f}  "
                    f"{weights[2]:>6.2f}\n"
                )
                self._log_file.write(line)
                self._log_file.flush()

                if not is_curve and self.post_curve_counter > 0:
                    self.post_curve_counter -= 1
                    if self.post_curve_counter == 0:
                        self._log_file.write("=" * 90 + "\n\n")
                        self._log_file.flush()

        # Consola en vivo (más limpia)
        end_time = time.time()
        latencia_proc = (end_time - start_time) * 1000
        self.get_logger().info(
            f"slide={sliding_delta or 0.0:+.1f}°  slope={slope_delta or 0.0:+.1f}°  "
            f"prev={prev_delta:+.1f}°  fused={fused_delta:+.1f}°  PWM={pwm}  "
            f"offset={offset_correction:+.1f}° w=[{weights[0]:.2f},{weights[1]:.2f},{weights[2]:.2f}]"
        )

        # 10. Visualización (Opcional)
        if SHOW_VIZ:
            roi_overlay = img_left.copy()
            cv2.polylines(roi_overlay, vertices_l, isClosed=True, color=(0, 255, 100), thickness=2)
            cv2.imshow("1. ROI", roi_overlay)
            cv2.imshow("Ojo derecho w", warped_right)
            cv2.imshow("Ojo izquierdo w", warped_left)
            cv2.imshow("3. Sliding Window Right", sliding_img_right)
            cv2.imshow("Sliding Window left", sliding_img_left)

            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectorNode()

    def stop_handler(sig, frame):
        stop_dir = String()
        stop_dir.data = "2642"
        stop_thr = String()
        stop_thr.data = "2457"
        for _ in range(5):
            node.direction_pwm_pub.publish(stop_dir)
            node.throttle_pwm_pub.publish(stop_thr)
            time.sleep(0.05)
        print("\n[INFO] Motores detenidos. Saliendo de carrera.")
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, stop_handler)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
