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
MODEL_PATH     = "/home/edwin/workspace/traxxas_ws/src/traxxas_laptop/traxxas_laptop/mejor.engine"
CURVE_LOG_PATH = "/home/edwin/workspace/traxxas_ws/src/traxxas_laptop/traxxas_laptop/curve_debug.txt"

CONF        = 0.35
COLOR_LEFT  = (0,   255, 100)
COLOR_RIGHT = (0,   180, 255)
ALPHA       = 0.45
SHOW_VIZ    = True
DEBUG_FILE  = False

# ── SEÑALES (CROSSWALK Y STOP) ──
SIGNAL_MODEL_PATH = "/home/edwin/workspace/traxxas_ws/src/traxxas_laptop/traxxas_laptop/crosswalks_alto.engine"
SIGNAL_CONF       = 0.60
SIGNAL_EYE        = "right"

CLASS_CROSSWALK = 0
CLASS_STOP      = 1

CROSSWALK_AREA_MIN  = 4000
CROSSWALK_STOP_SECS = 5.0

STOP_AREA_MIN  = 500
STOP_STOP_SECS = 5.0

# ─────────────────────────────────────────────
# PARÁMETROS PURE PURSUIT
# ─────────────────────────────────────────────
# Distancia entre ejes del vehículo (m)
WHEELBASE_L = 0.18

# Conversión píxeles → metros en el espacio BEV (ajusta a tu calibración)
XM_PER_PIX = 0.40 / 384   # m/píxel en X (lateral)
YM_PER_PIX = 1.0  / 720   # m/píxel en Y (longitudinal)

# Lookahead en metros (distancia hacia adelante donde apuntar)
LOOKAHEAD_DIST_M = 0.60
# Límite físico del giro del servo
MAX_STEER_DEG = 16.0

# ─────────────────────────────────────────────
# FUNCIONES DE PROCESAMIENTO VISUAL
# ─────────────────────────────────────────────

def detect_signals(model, frame, conf_thresh=SIGNAL_CONF):
    """Busca señales y retorna el tipo, confianza y caja de la más grande."""
    results = model.predict(frame, conf=conf_thresh, half=True, imgsz=640, verbose=False)

    best_det = {"type": None, "conf": 0.0, "box": None, "area": 0}

    for result in results:
        if result.boxes is None: continue
        for box in result.boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            print(f"YOLO detectó algo. ID: {cls_id} | Confianza: {conf:.2f}")
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
            area = (x2 - x1) * (y2 - y1)

            if cls_id == CLASS_CROSSWALK and area >= CROSSWALK_AREA_MIN:
                if area > best_det["area"]:
                    best_det = {"type": "crosswalk", "conf": conf, "box": (x1, y1, x2, y2), "area": area}

            elif cls_id == CLASS_STOP and area >= STOP_AREA_MIN:
                if area > best_det["area"]:
                    best_det = {"type": "stop", "conf": conf, "box": (x1, y1, x2, y2), "area": area}

    return best_det["type"], best_det["conf"], best_det["box"]


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
    """Procesa un ojo: aplica ROI, warp a BEV y sliding window."""
    roi_mask = np.zeros_like(binary_mask, dtype=np.uint8)
    cv2.fillPoly(roi_mask, vertices, 255)
    masked = cv2.bitwise_and(binary_mask, binary_mask, mask=roi_mask)

    warped = warp(masked, vertices[0], dst)
    sliding_img, fit, ploty = sliding_window(warped)
    return sliding_img, warped, roi_mask, fit, ploty


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


def center_fit_from_lane_fits(left_fit, right_fit, image_shape):
    """Calcula el polinomio del carril central a partir de los fits izq/der."""
    if left_fit is None or right_fit is None:
        return None, None
    center_fit = (left_fit + right_fit) / 2.0
    height, _ = image_shape
    ploty = np.linspace(0, height - 1, height)
    center_x = center_fit[0]*ploty**2 + center_fit[1]*ploty + center_fit[2]
    return center_fit, np.column_stack((center_x, ploty))


# ─────────────────────────────────────────────
# PURE PURSUIT
# ─────────────────────────────────────────────
def pure_pursuit_from_center_fit(center_fit, image_shape,
                                 lookahead_m=LOOKAHEAD_DIST_M,
                                 wheelbase=WHEELBASE_L,
                                 xm_per_pix=XM_PER_PIX,
                                 ym_per_pix=YM_PER_PIX):
    """
    Calcula el ángulo de dirección con Pure Pursuit a partir del polinomio
    del carril central en el espacio BEV.

    Convenio de ejes del vehículo:
        - Origen: centro del parachoques del coche (bottom-center del BEV)
        - Eje X: hacia el frente del coche (positivo)
        - Eje Y: hacia la izquierda del coche (positivo)

    En el BEV:
        - y_bev crece hacia abajo (coche abajo)
        - x_bev crece hacia la derecha

    Transformación BEV → vehículo:
        x_car = (h - y_bev) * ym_per_pix        (distancia longitudinal)
        y_car = (car_center_x - x_bev) * xm_per_pix   (lateral, + izquierda)

    Retorna:
        delta_deg: ángulo de dirección en grados (+ = giro a la izquierda aquí
                   se convierte al signo que maneja el servo en delta_to_pwm).
        target_xy_bev: coordenadas (x, y) en píxeles del BEV donde apunta el
                       lookahead (para visualización).
    """
    if center_fit is None:
        return None, None

    height, width = image_shape[:2]
    car_center_x_px = width / 2.0

    # Recorremos el polinomio desde abajo (coche) hacia arriba (lejos)
    ploty = np.linspace(height - 1, 0, height)
    fitx  = center_fit[0] * ploty**2 + center_fit[1] * ploty + center_fit[2]

    # Convertir a coordenadas del vehículo
    x_car = (height - ploty) * ym_per_pix              # longitudinal
    y_car = (car_center_x_px - fitx) * xm_per_pix      # lateral (+ izquierda)

    # Distancia euclidiana desde el coche a cada punto del centerline
    dist = np.sqrt(x_car**2 + y_car**2)

    # Buscar el primer punto cuya distancia supere el lookahead
    idx_candidates = np.where(dist >= lookahead_m)[0]
    if len(idx_candidates) == 0:
        # Si ningún punto está tan lejos, usar el más lejano disponible
        idx = len(dist) - 1
    else:
        idx = idx_candidates[0]

    Lx = x_car[idx]
    Ly = y_car[idx]
    Ld = math.sqrt(Lx**2 + Ly**2)

    if Ld < 1e-3 or Lx < 1e-3:
        return 0.0, (int(fitx[idx]), int(ploty[idx]))

    # Fórmula de Pure Pursuit:
    #   delta = atan( 2 * L * y_target / Ld^2 )
    # donde L = wheelbase, Ld = distancia al punto objetivo.
    delta_rad = math.atan2(2.0 * wheelbase * Ly, Ld * Ld)
    delta_deg = math.degrees(delta_rad)

    # Convención del servo: en el código original SLOPE_SIGN = -1 indicaba
    # que un ángulo "a la izquierda" en el mundo corresponde a PWM negativo.
    # Pure Pursuit aquí devuelve + para giro a la izquierda (y_car positivo),
    # pero delta_to_pwm espera + para la derecha. Invertimos el signo.
    delta_deg = -delta_deg

    # Saturar al rango físico del servo
    delta_deg = float(np.clip(delta_deg, -MAX_STEER_DEG, MAX_STEER_DEG))

    target_xy_bev = (int(fitx[idx]), int(ploty[idx]))
    return delta_deg, target_xy_bev


def delta_to_pwm(delta_deg: float, max_steer_deg: float = MAX_STEER_DEG) -> int:
    pwm_min, pwm_center, pwm_max = 1669, 2525, 3276
    delta_deg = float(np.clip(delta_deg, -max_steer_deg, max_steer_deg))
    norm = delta_deg / max_steer_deg
    if norm >= 0:
        return int(pwm_center + norm * (pwm_max - pwm_center))
    return int(pwm_center + norm * (pwm_center - pwm_min))


def draw_bev_centerline(fit_l, fit_r, center_fit, image_shape, target_xy=None):
    """Dibuja polinomios izq/der, centro y el punto lookahead sobre BEV."""
    if fit_l is None or fit_r is None or center_fit is None:
        return None

    height, width = image_shape[:2]
    vis = np.zeros((height, width, 3), dtype=np.uint8)

    ploty = np.linspace(0, height - 1, height)
    left_fitx   = fit_l[0] * ploty**2 + fit_l[1] * ploty + fit_l[2]
    right_fitx  = fit_r[0] * ploty**2 + fit_r[1] * ploty + fit_r[2]
    center_fitx = center_fit[0] * ploty**2 + center_fit[1] * ploty + center_fit[2]

    pts_left   = np.array([np.transpose(np.vstack([left_fitx,   ploty]))], dtype=np.int32)
    pts_right  = np.array([np.transpose(np.vstack([right_fitx,  ploty]))], dtype=np.int32)
    pts_center = np.array([np.transpose(np.vstack([center_fitx, ploty]))], dtype=np.int32)

    pts_drivable = np.hstack((pts_left, np.fliplr(pts_right)))
    cv2.fillPoly(vis, np.int_([pts_drivable]), (0, 50, 0))

    cv2.polylines(vis, pts_left,   isClosed=False, color=(255,   0,   0), thickness=4)
    cv2.polylines(vis, pts_right,  isClosed=False, color=(0,     0, 255), thickness=4)
    cv2.polylines(vis, pts_center, isClosed=False, color=(0,   255, 255), thickness=2)

    cv2.line(vis, (width // 2, 0), (width // 2, height),
             (255, 255, 255), 1, cv2.LINE_AA)

    # Punto más cercano al coche
    closest_x = int(center_fitx[-1])
    closest_y = int(ploty[-1])
    cv2.circle(vis, (closest_x, closest_y), 8, (0, 255, 255), -1)

    # Punto lookahead (Pure Pursuit)
    if target_xy is not None:
        tx, ty = target_xy
        cv2.circle(vis, (tx, ty), 10, (0, 255, 0), -1)
        cv2.line(vis, (width // 2, height - 1), (tx, ty),
                 (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(vis, "lookahead", (tx + 12, ty),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    return vis


# ─────────────────────────────────────────────
# NODO ROS 2
# ─────────────────────────────────────────────
class LaneDetectorPurePursuit(Node):
    def __init__(self):
        super().__init__('laptop_brain_node')

        qos_motor = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.img_sub = self.create_subscription(
            CompressedImage, '/zed/stereo/compressed',
            self.image_callback, qos_profile_sensor_data
        )
        self.direction_pwm_pub = self.create_publisher(String, 'direction_servo', qos_motor)
        self.throttle_pwm_pub  = self.create_publisher(String, 'throttle_motor',  qos_motor)
        self.pub_led           = self.create_publisher(String, 'led_power', qos_motor)

        self.get_logger().info("Cargando modelo YOLO de carriles...")
        self.model = YOLO(MODEL_PATH, task="segment")
        self.get_logger().info("Modelo de carriles listo.")

        self.get_logger().info("Cargando modelo de SEÑALES...")
        self.signal_model = YOLO(SIGNAL_MODEL_PATH, task="detect")
        dummy = np.zeros((640, 640, 3), dtype=np.uint8)
        self.signal_model.predict(dummy, conf=SIGNAL_CONF, half=True, imgsz=640, verbose=False)
        self.get_logger().info("Modelo de señales listo.")

        # Estado de frenado por señales
        self.crosswalk_stop_until = 0.0
        self.crosswalk_cooldown_until = 0.0
        self.cw_consecutive = 0
        self.CW_MIN_CONSECUTIVE = 1

        self.brake_phase = "none"
        self.brake_phase_start = 0.0
        self.BRAKE_ACTIVE_SECS = 1.5
        self.PWM_BRAKE_ACTIVE = 1700
        self.PWM_BRAKE_HOLD   = 2457

        self.cw_frame_counter = 0
        self.CW_EVERY_N_FRAMES = 1
        self.fut_cw = None
        self.last_cw_result = (None, 0.0, None)

        self.executor_cv = ThreadPoolExecutor(max_workers=3)

        self.start_time = time.time()

        # Calibración cámaras
        self.camera_matrix_left = np.array([[533.395, 0, 626.57],
                                            [0, 533.58, 352.625],
                                            [0,   0,   1]], dtype=np.float32)

        self.camera_matrix_right = np.array([[532.495, 0, 640.71],
                                             [0, 532.685, 363.754],
                                             [0,   0,   1]], dtype=np.float32)

        self.undistortion_coeffs_left  = np.array(
            [-0.0625241, 0.0400944, -0.000127884, -0.000304066, -0.0161226],
            dtype=np.float32
        )
        self.undistortion_coeffs_right = np.array(
            [-0.0645392, 0.0433723, -0.000604329, -0.00019725, -0.01751],
            dtype=np.float32
        )

        if DEBUG_FILE:
            self._log_file = open(CURVE_LOG_PATH, "a", encoding="utf-8")
            self.get_logger().info(f"Guardando log de telemetría en: {CURVE_LOG_PATH}")

    def destroy_node(self):
        if DEBUG_FILE:
            self._log_file.close()
        super().destroy_node()

    def image_callback(self, msg):
        start_time = time.time()

        # 1. Descomprimir estéreo
        np_arr = np.frombuffer(msg.data, np.uint8)
        stereo_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # 2. Separar ojos
        h_total, w_total = stereo_img.shape[:2]
        w = w_total // 2
        img_left  = stereo_img[:, :w]
        img_left  = cv2.undistort(img_left, self.camera_matrix_left, self.undistortion_coeffs_left)
        img_right = stereo_img[:, w:]
        img_right = cv2.undistort(img_right, self.camera_matrix_right, self.undistortion_coeffs_right)

        cw_frame = img_left if SIGNAL_EYE == "left" else img_right

        # Señales asíncronas (SIN CAMBIOS)
        if self.fut_cw is not None and self.fut_cw.done():
            self.last_cw_result = self.fut_cw.result()
            self.fut_cw = None

            detected_type, det_conf, det_box = self.last_cw_result
            now_cw = time.time()

            if detected_type is not None:
                self.cw_consecutive += 1
                if (self.cw_consecutive >= self.CW_MIN_CONSECUTIVE
                        and now_cw > self.crosswalk_cooldown_until):

                    tiempo_freno = CROSSWALK_STOP_SECS if detected_type == "crosswalk" else STOP_STOP_SECS

                    self.crosswalk_stop_until = now_cw + tiempo_freno
                    self.crosswalk_cooldown_until = (now_cw + tiempo_freno + 3.0)
                    self.brake_phase = "active_brake"
                    self.brake_phase_start = now_cw
                    self.cw_consecutive = 0

                    self.get_logger().warn(
                        f"¡{detected_type.upper()} confirmado! Frenando {tiempo_freno}s"
                    )
            else:
                self.cw_consecutive = 0

        self.cw_frame_counter += 1
        if self.cw_frame_counter % self.CW_EVERY_N_FRAMES == 0 and self.fut_cw is None:
            self.fut_cw = self.executor_cv.submit(detect_signals, self.signal_model, cw_frame.copy())

        # 3. Inferencia YOLO carriles
        results = self.model([img_left, img_right], conf=CONF, half=True,
                             imgsz=640, retina_masks=True, verbose=False)

        # 4. Máscaras binarias
        mask_left  = yolo_mask_to_binary(results[0], h_total, w)
        mask_right = yolo_mask_to_binary(results[1], h_total, w)

        # 5. Vértices ROI / BEV
        vertices_l = np.array([[(int(0.16*w), int(0.63*h_total)),
                                (int(0.47*w), int(0.63*h_total)),
                                (int(0.55*w), h_total),
                                (int(0.08*w), h_total)]])

        vertices_r = np.array([[(int(0.55*w), int(0.63*h_total)),
                                (int(0.85*w), int(0.63*h_total)),
                                (int(1.0*w), h_total),
                                (int(0.48*w), h_total)]])

        dst = np.array([[int(0.40*w), 0],
                        [int(0.60*w), 0],
                        [int(0.60*w), h_total],
                        [int(0.40*w), h_total]])

        # 6. Sliding window paralelo
        fut_l = self.executor_cv.submit(process_eye, mask_left,  vertices_l, dst)
        fut_r = self.executor_cv.submit(process_eye, mask_right, vertices_r, dst)

        sliding_img_left,  warped_left,  roi_l, fit_l, _ = fut_l.result()
        sliding_img_right, warped_right, roi_r, fit_r, _ = fut_r.result()

        # 7. Fit del carril central
        center_fit, _ = center_fit_from_lane_fits(fit_l, fit_r, warped_left.shape)

        # 8. PURE PURSUIT
        delta_deg, target_xy = pure_pursuit_from_center_fit(
            center_fit, warped_left.shape,
            lookahead_m=LOOKAHEAD_DIST_M,
            wheelbase=WHEELBASE_L
        )

        # 9. Visualización BEV
        bev_vis = draw_bev_centerline(fit_l, fit_r, center_fit,
                                      warped_left.shape, target_xy=target_xy)

        # 10. Conversión a PWM y publicación
        now = time.time()

        if delta_deg is None:
            pwm_dir = 2642   # PWM de "centrado/seguro" cuando no hay info
        else:
            pwm_dir = delta_to_pwm(delta_deg)

        dir_msg = String()
        dir_msg.data = str(pwm_dir)
        self.direction_pwm_pub.publish(dir_msg)

        throttle_msg = String()
        led_msg = String()

        if now - self.start_time < 4.0:
            throttle_msg.data = str(self.PWM_BRAKE_HOLD)
            led_msg.data = "S"

        elif now < self.crosswalk_stop_until:
            elapsed_brake = now - self.brake_phase_start
            led_msg.data = "S"

            if self.brake_phase == "active_brake":
                if elapsed_brake < self.BRAKE_ACTIVE_SECS:
                    throttle_msg.data = str(self.PWM_BRAKE_ACTIVE)
                else:
                    self.brake_phase = "hold"
                    throttle_msg.data = str(self.PWM_BRAKE_HOLD)
            else:
                throttle_msg.data = str(self.PWM_BRAKE_HOLD)

        else:
            if self.brake_phase != "none":
                self.brake_phase = "none"
            throttle_msg.data = str(2730)
            led_msg.data = "F"

        self.throttle_pwm_pub.publish(throttle_msg)
        self.pub_led.publish(led_msg)

        if now < self.crosswalk_stop_until:
            self.get_logger().warn(
                f"SEÑAL BRAKE: phase={self.brake_phase}  "
                f"restante={self.crosswalk_stop_until - now:.1f}s  "
                f"throttle={throttle_msg.data}"
            )

        # 11. Log básico
        end_time = time.time()
        latencia_proc = (end_time - start_time) * 1000
        delta_str = f"{delta_deg:+.2f}°" if delta_deg is not None else "None"
        self.get_logger().info(
            f"[PP] delta={delta_str}  PWM={pwm_dir}  "
            f"target={target_xy}  lat={latencia_proc:.1f}ms"
        )

        # 12. Visualización ROI + señales
        if SHOW_VIZ:
            roi_left = img_left.copy()
            canvas_l = roi_left.copy()
            cv2.fillPoly(canvas_l, vertices_l, COLOR_LEFT)
            cv2.addWeighted(canvas_l, ALPHA, roi_left, 1 - ALPHA, 0, roi_left)
            cv2.polylines(roi_left, vertices_l, isClosed=True, color=COLOR_LEFT, thickness=2)

            roi_right = img_right.copy()
            canvas_r = roi_right.copy()
            cv2.fillPoly(canvas_r, vertices_r, COLOR_RIGHT)
            cv2.addWeighted(canvas_r, ALPHA, roi_right, 1 - ALPHA, 0, roi_right)
            cv2.polylines(roi_right, vertices_r, isClosed=True, color=COLOR_RIGHT, thickness=2)

            roi_overlay = img_right.copy()
            cv2.polylines(roi_overlay, vertices_l, isClosed=True, color=(0, 255, 100), thickness=2)

            if self.last_cw_result[0] is not None and self.last_cw_result[2] is not None:
                det_type = self.last_cw_result[0]
                cw_box = self.last_cw_result[2]
                x1, y1, x2, y2 = cw_box
                area = (x2 - x1) * (y2 - y1)

                color_box = (0, 0, 255) if det_type == "stop" else (255, 0, 255)
                cv2.rectangle(roi_overlay, (x1, y1), (x2, y2), color_box, 4)
                label = f"{det_type.upper()} Area: {area} px"
                cv2.putText(roi_overlay, label, (x1, max(y1 - 10, 20)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, color_box, 2)

            if bev_vis is not None:
                cv2.imshow("BEV - Lanes + Center + Lookahead", bev_vis)
            cv2.imshow("ROI Left", roi_left)
            cv2.imshow("ROI Right", roi_right)

            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectorPurePursuit()

    def stop_handler(sig, frame):
        stop_dir = String(); stop_dir.data = "2642"
        stop_thr = String(); stop_thr.data = "2457"
        stop_led = String(); stop_led.data = "S"

        for _ in range(10):
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