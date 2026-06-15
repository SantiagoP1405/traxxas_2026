from ultralytics import YOLO
import threading
import queue
from concurrent.futures import ThreadPoolExecutor
import torch.nn.functional as F
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
import math
import numpy as np
import cv2
import pyzed.sl as sl
import time
import signal



# ─────────────────────────────────────────────
# CONFIG RED NEURONAL
# ─────────────────────────────────────────────
MODEL_PATH  = "/home/traxxas/Workspaces/traxxas_pruebas/best.engine"
RUTA_SVO    = "/home/traxxas/Documents/ZED/circuito_completo.svo2"
#RUTA_SVO    = ""
USE_SVO     = True
CONF        = 0.35
COLOR_LEFT  = (0,   255, 100)
COLOR_RIGHT = (0,   180, 255)
ALPHA       = 0.45
# ─────────────────────────────────────────────
SHOW_VIZ = True  # True solo cuando hay display disponible
DEBUG_FILE = True # guarda datos de curvas en un txt para análisis offline
# ─────────────────────────────────────────────
# FUSIÓN ADAPTATIVA DE LOS 3 FACTORES
# ─────────────────────────────────────────────
CURVE_THRESHOLD_DEG = 5.0    # |ángulo| > esto → modo curva
SLOPE_MAX_DEG       = 22.0   # pendiente máxima esperada en warped (calibrar con datos)
"""
pendiente raw (grados)  →  normalizar con SLOPE_MAX_DEG=15  →  escalar a max_steer_deg=30°

Ejemplo:
  raw_slope = 15°  →  norm = 15/15 = 1.0  →  output = 1.0 * 30 = +30°
  raw_slope = 7.5° →  norm = 7.5/15 = 0.5 →  output = 0.5 * 30 = +15°
"""
SLOPE_SIGN          = -1.0   # invertir si el signo de la pendiente no coincide físicamente
CONSENSUS_TOL_DEG   = 2.0    # dos estimaciones "coinciden" si difieren menos de esto
# Pesos base [sliding, slope, anterior] — cada fila suma 1.0
W_STRAIGHT = np.array([0.60, 0.25, 0.15])   # carretera recta: confías en sliding
W_CURVE    = np.array([0.15, 0.45, 0.40])   # curva cerrada: sliding pierde credibilidad

# ─────────────────────────────────────────────
# DEBUGGING
# ─────────────────────────────────────────────
CURVE_LOG_PATH      = "/home/traxxas/Workspaces/traxxas_ws/src/curve_debug.txt" # ruta del log de curvas para análisis offline
CURVE_POST_LINES    = 10   # cuántas líneas guardar al salir de la curva
# ─────────────────────────────────────────────

def warp(img, src, dst):
    M = cv2.getPerspectiveTransform(src.astype(np.float32), dst.astype(np.float32))
    return cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))

    

def yolo_mask_to_binary(result, target_h, target_w):
    """
    Extrae la máscara combinada del resultado YOLO y la convierte
    a imagen binaria del tamaño del frame original.
    """
    if result.masks is None:
        return np.zeros((target_h, target_w), dtype=np.uint8)

    masks_gpu = result.masks.data                    # (N, h, w)

    if masks_gpu.shape[0] == 0:                      # ← guarda nueva
        return np.zeros((target_h, target_w), dtype=np.uint8)

    combined  = masks_gpu.max(dim=0).values          # (h, w) en GPU

    resized = F.interpolate(
        combined.float().unsqueeze(0).unsqueeze(0),
        size=(target_h, target_w),
        mode='bilinear',
        align_corners=False
    ).squeeze()

    return (resized > 0.5).byte().cpu().numpy() * 255



def process_eye(binary_mask, vertices, dst):
    """
    Pipeline: máscara YOLO → ROI → warp → sliding_window → curvature
    """
    roi_mask = np.zeros_like(binary_mask, dtype=np.uint8)
    cv2.fillPoly(roi_mask, vertices, 255)
    masked = cv2.bitwise_and(binary_mask, binary_mask, mask=roi_mask)


    warped = warp(masked, vertices[0], dst)
    # Sliding usa todo el warped (necesita ver la curva completa)
    sliding_img, fit, ploty = sliding_window(warped)
    curve = measure_curvature(ploty, fit) if (fit is not None and ploty is not None) else None

    # Slope usa solo la mitad inferior (zona limpia, sin carril opuesto)
    h_warped = warped.shape[0]
    warped_bottom = warped[int(h_warped * 0.4):, :]  #  60% inferior

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
    minpix        = 10
    lane_inds     = []
    out_img       = np.dstack((binary_warped, binary_warped, binary_warped)) * 255

    # ── Umbrales de calidad ───────────────────────────────────────────────────
    MIN_LANE_PIXELS = 300   # píxeles totales mínimos → sube si aún ves detecciones basura
    MIN_WINDOWS_HIT = 7     # ventanas mínimas con señal real
    # ─────────────────────────────────────────────────────────────────────────

    windows_with_data = 0   # contador para el chequeo de distribución

    for window in range(nwindows):
        win_y_low  = binary_warped.shape[0] - (window + 1) * window_height
        win_y_high = binary_warped.shape[0] - window * window_height
        win_low    = current_x - margin
        win_high   = current_x + margin

        cv2.rectangle(out_img, (win_low, win_y_low), (win_high, win_y_high), (0, 255, 0), 2)

        good_inds = (
            (nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
            (nonzerox >= win_low)   & (nonzerox < win_high)
        ).nonzero()[0]

        lane_inds.append(good_inds)

        if len(good_inds) > minpix:
            current_x = int(np.mean(nonzerox[good_inds]))
            windows_with_data += 1   # ← esta ventana tenía señal real

    lane_inds = np.concatenate(lane_inds)
    x = nonzerox[lane_inds]
    y = nonzeroy[lane_inds]

    # ── Chequeos de calidad ───────────────────────────────────────────────────
    if len(x) < MIN_LANE_PIXELS:
        return out_img, None, None

    if windows_with_data < MIN_WINDOWS_HIT:
        return out_img, None, None
    # ─────────────────────────────────────────────────────────────────────────

    fit = np.polyfit(y, x, 2)

    if abs(fit[0]) > 0.01:
        return out_img, None, None

    ploty = np.linspace(0, binary_warped.shape[0] - 1, binary_warped.shape[0])
    fitx  = fit[0] * ploty**2 + fit[1] * ploty + fit[2]

    out_img[nonzeroy[lane_inds], nonzerox[lane_inds]] = [255, 0, 0]
    for i in range(len(ploty)):
        cv2.circle(out_img, (int(fitx[i]), int(ploty[i])), 3, (255, 255, 0), -1)

    return out_img, fit, ploty
# ──────────────────────────────────────────────────────────────────────────────
# ESTIMADOR 2 — Pendiente lineal de la línea de carril en la imagen warped
# ──────────────────────────────────────────────────────────────────────────────


def slope_angle_from_warped(warped_mask: np.ndarray) -> float | None:
    """
    Ajusta una recta (polinomio grado 1) a los píxeles blancos del warped
    y devuelve el ángulo de inclinación de la línea respecto a la vertical,
    en grados.


    Convenio de signo:
      positivo → línea se inclina a la derecha conforme nos acercamos al coche
               → carretera curva hacia la derecha
      negativo → curva a la izquierda


    Devuelve None si no hay suficientes píxeles para un ajuste fiable.
    """
    ny, nx = warped_mask.nonzero()
    if len(nx) < 30:
        return None, None


    # Ajustar x = a·y + b  (y es la variable independiente porque la línea
    # es casi vertical en el warped y poly(x) no funciona bien)
    a, b = np.polyfit(ny.astype(np.float32), nx.astype(np.float32), 1)
    angle = float(np.degrees(np.arctan(a)))
    return angle, (a, b)



def slope_to_steering(slope_left: float | None,
                      slope_right: float | None,
                      max_steer_deg: float = 25.0) -> float | None:
    """
    Convierte los ángulos de pendiente de ambos ojos a un único ángulo
    de dirección con signo, en el mismo espacio que usan los otros estimadores.


    SLOPE_SIGN invierte el convenio si la instalación física lo requiere.
    SLOPE_MAX_DEG es la pendiente (grados) que se mapea al volante máximo.
    """
    angles = [a for a in (slope_left, slope_right) if a is not None]
    if not angles:
        return None


    mean_slope = float(np.mean(angles))
    norm = np.clip(mean_slope / SLOPE_MAX_DEG, -1.0, 1.0)
    return SLOPE_SIGN * norm * max_steer_deg


def measure_curvature(ploty, fit):
    ym_per_pix = 1.0  / 720
    xm_per_pix = 0.40 / 384 #lo que mide de ancho el dst que transforma de ROI a Warped


    y_eval = np.max(ploty)
    fitx   = fit[0] * ploty**2 + fit[1] * ploty + fit[2]


    fit_cr = np.polyfit(ploty * ym_per_pix, fitx * xm_per_pix, 2)


    curverad = (
        (1 + (2 * fit_cr[0] * y_eval * ym_per_pix + fit_cr[1])**2)**1.5
    ) / np.absolute(2 * fit_cr[0])


    #print(f"  [CURV] ym_per_pix={ym_per_pix:.6f}  xm_per_pix={xm_per_pix:.6f}")
    #print(f"  [CURV] fit_cr[0]={fit_cr[0]:.4f}  fit_cr[1]={fit_cr[1]:.4f}")
    #print(f"  [CURV] radio calculado: {curverad:.4f} m")
    #print("curve rad: ", curverad)
    return curverad

def steering_from_curvature(left_curvature, right_curvature):
    L = 0.28

    #print(f"\\\\n[STEER] left_curve={left_curvature}  right_curve={right_curvature}")

    if left_curvature is not None and right_curvature is not None:
        R = min(left_curvature, right_curvature)
    elif left_curvature is not None:
        R = left_curvature
    elif right_curvature is not None:
        R = right_curvature
    else:
        R = right_curvature

    #print(f"[STEER] R usado={R:.4f} m | L={L} m")

    delta_rad = math.atan(L / R)
    delta_deg = math.degrees(delta_rad)

    #print(f"[STEER] delta_rad={delta_rad:.6f}  delta_deg={delta_deg:.4f}°")

    return delta_deg


# ──────────────────────────────────────────────────────────────────────────────
# REFACTORIZACIÓN DE LAS FUNCIONES EXISTENTES
# (se separa el signo de la conversión a PWM para poder fusionar en ángulos)
# ──────────────────────────────────────────────────────────────────────────────


def signed_delta_from_curvature(left_curve: float | None,
                                right_curve: float | None) -> float | None:
    """
    Igual que steering_from_curvature() + la lógica de signo de angle_to_pwm(),
    pero SIN convertir a PWM.  Devuelve None si no hay ningún carril.
    """
     # Solo calculamos si tenemos LOS DOS carriles detectados
    if left_curve is None or right_curve is None:
        return None

    delta = steering_from_curvature(left_curve, right_curve)
    return -abs(delta) if right_curve > left_curve else abs(delta)



def delta_to_pwm(delta_deg: float, max_steer_deg: float = 25.0) -> int:
    """Convierte ángulo con signo (grados) a valor PWM."""
    pwm_min, pwm_center, pwm_max = 1669, 2642, 3276
    delta_deg = float(np.clip(delta_deg, -max_steer_deg, max_steer_deg))
    norm = delta_deg / max_steer_deg
    if norm >= 0:
        return int(pwm_center + norm * (pwm_max - pwm_center))
    return int(pwm_center + norm * (pwm_center - pwm_min))



# ──────────────────────────────────────────────────────────────────────────────
# FUSIÓN ADAPTATIVA
# ──────────────────────────────────────────────────────────────────────────────


def fuse_steering_angles(
    sliding_deg: float | None,
    slope_deg:   float | None,
    prev_deg:    float,
) -> tuple[float, np.ndarray, float]:
    """
    Combina tres estimadores de ángulo de dirección con pesos adaptativos.


    Parámetros
    ----------
    sliding_deg : de sliding window + curvatura (puede ser None)
    slope_deg   : de ajuste lineal de la línea warped (puede ser None)
    prev_deg    : ángulo enviado en el ciclo anterior (siempre disponible)


    Devuelve
    --------
    fused_deg   : ángulo fusionado final (grados, con signo)
    weights     : pesos efectivos [w_slide, w_slope, w_prev]  → útil para debug
    curve_level : 0 = recto  …  1 = curva cerrada             → útil para debug
    """
    estimates = [sliding_deg, slope_deg, prev_deg]
    available = np.array([e is not None for e in estimates], dtype=float)
    vals      = np.array([e if e is not None else 0.0 for e in estimates])


    n = int(available.sum())
    if n == 0:
        return 0.0, np.zeros(3), 0.0
    if n == 1:
        idx = int(np.argmax(available))
        w = available / available.sum()
        return float(vals[idx]), w, 0.0


    # ── 1. Nivel de curvatura ─────────────────────────────────────────────────
    # Usamos prev_deg como referencia estable; si no hay historial, sliding.
    ref_angle   = prev_deg if abs(prev_deg) > 0.1 else (sliding_deg or 0.0)
    curve_level = float(np.clip(abs(ref_angle) / CURVE_THRESHOLD_DEG, 0.0, 1.0))


    # ── 2. Pesos base interpolados según nivel de curvatura ───────────────────
    weights = (1.0 - curve_level) * W_STRAIGHT + curve_level * W_CURVE
    weights *= available          # anular estimadores no disponibles
    weights /= weights.sum()


    # ── 4. Ángulo fusionado ───────────────────────────────────────────────────
    fused = float(np.dot(weights, vals))
    return fused, weights, curve_level



def iniciar_zed(ruta_svo=None):
    zed         = sl.Camera()
    init_params = sl.InitParameters()


    init_params.depth_mode        = sl.DEPTH_MODE.PERFORMANCE
    init_params.coordinate_units  = sl.UNIT.METER


    if ruta_svo:
        print(f"Modo: Leyendo archivo grabado -> {ruta_svo}")
        init_params.set_from_svo_file(ruta_svo)
        init_params.svo_real_time_mode = False
    else:
        print("Modo: Cámara ZED 2 en vivo")
        init_params.camera_resolution = sl.RESOLUTION.HD720
        init_params.camera_fps        = 30


    status = zed.open(init_params)
    if status != sl.ERROR_CODE.SUCCESS:
        print(f"Error al abrir ZED: {status}")
        return None


    return zed



class LaneDetectorNode(Node):


    def __init__(self):
        super().__init__('lane_detector_node')


        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )


        self.direction_pwm_pub = self.create_publisher(String, 'direction_servo', qos_profile)
        self.throttle_pwm_pub  = self.create_publisher(String, 'throttle_motor',  qos_profile)


        self.zed = iniciar_zed(RUTA_SVO)
        if self.zed is None:
            self.get_logger().error("No se pudo inicializar la ZED con el SVO")
            raise RuntimeError("Error inicializando ZED")


        self.model = YOLO(MODEL_PATH, task="segment")
        self.get_logger().info("YOLO TRT engine listo.")


        self.frame_queue = queue.Queue(maxsize=1)
        self.stop_event  = threading.Event()


        self.prod_thread = threading.Thread(target=self._zed_producer, daemon=True)
        self.prod_thread.start()


        self.executor_cv = ThreadPoolExecutor(max_workers=2)


        self.left_curve  = None
        self.right_curve = None
        self.prev_steering_deg = 0.0
        self.in_curve_mode      = False
        self.post_curve_counter = 0
        self._log_file          = open(CURVE_LOG_PATH, "a", encoding="utf-8")

        self.timer = self.create_timer(1/30, self.timer_callback)
        
        """
        # ── Kalman 1D - Right ─────────────────────────────────────────────────
        self.kf_x_right = None   # estado estimado
        self.kf_P_right = 10.0   # covarianza del error
        self.kf_Q       = 0.5    # ruido de proceso
        self.kf_R       = 8.0    # ruido de medición


        # ── Kalman 1D - Left ──────────────────────────────────────────────────
        self.kf_x_left  = None
        self.kf_P_left  = 10.0

        # ──────────────────────────────────────────────────────────────────────────
        """
    def destroy_node(self):
        self._log_file.close()
        super().destroy_node()

    def _zed_producer(self):
        img_l   = sl.Mat()
        img_r   = sl.Mat()
        runtime = sl.RuntimeParameters()


        while not self.stop_event.is_set():
            grab = self.zed.grab(runtime)


            if grab == sl.ERROR_CODE.END_OF_SVOFILE_REACHED:
                self.get_logger().warn("Fin del SVO → reiniciando")
                self.zed.set_svo_position(0)
                continue
            elif grab != sl.ERROR_CODE.SUCCESS:
                self.get_logger().warn(f"Error grab: {grab}")
                break


            self.zed.retrieve_image(img_l, sl.VIEW.LEFT)
            self.zed.retrieve_image(img_r, sl.VIEW.RIGHT)


            fl = img_l.get_data()[:, :, :3].copy()
            fr = img_r.get_data()[:, :, :3].copy()


            if self.frame_queue.full():
                try:
                    self.frame_queue.get_nowait()
                except queue.Empty:
                    pass


            self.frame_queue.put((fl, fr))


        self.frame_queue.put(None)


    # ──────────────────────────────────────────────────────────────────────────
    def timer_callback(self):
        start_time = time.time()


        try:
            item = self.frame_queue.get_nowait()
        except queue.Empty:
            return
        if item is None:
            return


        img_bgr_left, img_bgr_right = item
        h, w = img_bgr_left.shape[:2]


        
        # ── YOLO inference ────────────────────────────────────────────────────
        results = self.model(
            [img_bgr_left, img_bgr_right],
            conf         = CONF,
            half         = True,
            imgsz        = 640,
            classes      = [0],
            retina_masks = True,
            verbose      = False,
        )


        mask_left  = yolo_mask_to_binary(results[0], h, w)
        mask_right = yolo_mask_to_binary(results[1], h, w)


        # ── ROI ───────────────────────────────────────────────────────────────
        vertices_left = np.array([[
            (int(0.16 * w), int(0.56 * h)),   # TL
            (int(0.47 * w), int(0.56 * h)),   # TR
            (int(0.55 * w), int(1 * h)),      # BR
            (int(0.08 * w), int(1 * h)),      # BL
        ]], dtype=np.int32)


        vertices_right = np.array([[
            (int(0.64 * w), int(0.56 * h)),   # TL
            (int(0.95 * w), int(0.56 * h)),   # TR
            (int(1.00 * w), int(1 * h)),      # BR
            (int(0.56 * w), int(1 * h)),      # BL
        ]], dtype=np.int32)


        #----------------------------------------------------------------------
        # ── Visualización de ROI ───────────────────────────────────────────────────
        roi_overlay = img_bgr_left.copy()


        # Relleno semitransparente
        canvas = roi_overlay.copy()
        cv2.fillPoly(canvas, vertices_left,  COLOR_LEFT)
        cv2.fillPoly(canvas, vertices_right, COLOR_RIGHT)
        cv2.addWeighted(canvas, ALPHA, roi_overlay, 1 - ALPHA, 0, roi_overlay)


        # Contorno sólido de cada polígono
        cv2.polylines(roi_overlay, vertices_left,  isClosed=True, color=COLOR_LEFT,  thickness=2)
        cv2.polylines(roi_overlay, vertices_right, isClosed=True, color=COLOR_RIGHT, thickness=2)


        # Etiquetas
        cv2.putText(roi_overlay, "ROI Izq", 
                    (vertices_left[0][0][0],  vertices_left[0][0][1]  - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, COLOR_LEFT,  2)
        cv2.putText(roi_overlay, "ROI Der",
                    (vertices_right[0][0][0], vertices_right[0][0][1] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, COLOR_RIGHT, 2)


        # ── Warp destination ──────────────────────────────────────────────────


        dst1 = np.array([
            [int(0.35 * w), 0], #TR
            [int(0.65 * w), 0],	#TL
            [int(0.65 * w), h],	#BR
            [int(0.35 * w), h],	#BL
        ], dtype=np.int32)


        # ── Paralelo: ROI + warp + sliding ────────────────────────────────────
        fut_l = self.executor_cv.submit(process_eye, mask_left,  vertices_left,  dst1)
        fut_r = self.executor_cv.submit(process_eye, mask_right, vertices_right, dst1)


        sliding_img_left,  warped_left,  warped_bottom_l, roi_l, self.left_curve  = fut_l.result()
        sliding_img_right, warped_right, warped_bottom_r, roi_r, self.right_curve = fut_r.result()


        ## ── Factor 1: Sliding window ─────
        sliding_delta = signed_delta_from_curvature(self.left_curve, self.right_curve)
        #ganancia empirica 
        SLIDING_GAIN = 2.5
        if sliding_delta is not None:
            sliding_delta *= SLIDING_GAIN

        # ── Factor 2: Pendiente lineal ────────────────────────────────────────
        slope_left,  fit_line_l = slope_angle_from_warped(warped_bottom_l)
        slope_right, fit_line_r = slope_angle_from_warped(warped_bottom_r)
        slope_delta = slope_to_steering(slope_left, slope_right)



        # ── Factor 3: Steering anterior ───────────────────────────────────────
        prev_delta = self.prev_steering_deg


        # ── Fusión adaptativa ─────────────────────────────────────────────────
        fused_delta, weights, curve_level = fuse_steering_angles(sliding_delta, slope_delta, prev_delta)


        self.prev_steering_deg = fused_delta

        if DEBUG_FILE:
            # ── Convertir a PWM ───────────────────────────────────────────────────
            if sliding_delta is None and slope_delta is None:
                pwm = 2642
                self.get_logger().warn("Sin detección → PWM centro")
            else:
                pwm = delta_to_pwm(fused_delta)

            # ── Logging de curva ──────────────────────────────────────────────────
            is_curve = abs(fused_delta) > CURVE_THRESHOLD_DEG

            # Detección de transiciones
            if is_curve and not self.in_curve_mode:
                self.in_curve_mode      = True
                self.post_curve_counter = 0
                header = (
                    "\n" + "=" * 90 + "\n"
                    f"  ENTRANDO EN MODO CURVA  —  fused={fused_delta:+.1f}°  "
                    f"threshold={CURVE_THRESHOLD_DEG}°\n"
                    + "=" * 90 + "\n"
                    f"{'timestamp':>10}  {'slide':>8}  {'slope':>8}  {'prev':>8}  "
                    f"{'fused':>8}  {'PWM':>5}  {'curva':>6}  "
                    f"{'w_sld':>6}  {'w_slp':>6}  {'w_prv':>6}\n"
                    + "-" * 90 + "\n"
                )
                self._log_file.write(header)
                self._log_file.flush()
                self.get_logger().warn("📐 ENTRANDO EN MODO CURVA")

            elif not is_curve and self.in_curve_mode:
                self.in_curve_mode = False
                self.post_curve_counter = CURVE_POST_LINES   # arranca cuenta regresiva
                footer = "-" * 90 + "\n  SALIENDO DE MODO CURVA  — guardando últimas líneas\n"
                self._log_file.write(footer)
                self._log_file.flush()
                self.get_logger().warn("✅ SALIENDO DE MODO CURVA")

            # Decidir si escribir esta línea
            should_log = is_curve or (self.post_curve_counter > 0)

            if should_log:
                ts   = time.strftime("%H:%M:%S")
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

            # Log normal en consola (solo en curva)
            if is_curve or self.post_curve_counter > 0:
                self.get_logger().info(
                    "slide={:+.1f}°  slope={:+.1f}°  prev={:+.1f}°  "
                    "fused={:+.1f}°  PWM={}  curva={:.2f}  "
                    "w=[{:.2f},{:.2f},{:.2f}]".format(
                        sliding_delta or 0.0,
                        slope_delta   or 0.0,
                        prev_delta,
                        fused_delta,
                        pwm,
                        curve_level,
                        *weights,
                    )
                )
               
        else: #solo impirmir en terminal
            self.get_logger().info(
                "slide={:+.1f}°  slope={:+.1f}°  prev={:+.1f}°  "
                "fused={:+.1f}°  PWM={}  curva={:.2f}  "
                "w=[{:.2f},{:.2f},{:.2f}]".format(
                    sliding_delta or 0.0,
                    slope_delta   or 0.0,
                    prev_delta,
                    fused_delta,
                    pwm,
                    curve_level,
                    *weights,
                )
            )


        # ── Overlay texto ─────────────────────────────────────────────────────
        if self.left_curve is None:
            cv2.putText(sliding_img_left, "Carril izquierdo no detectado",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        else:
            cv2.putText(sliding_img_left, f"Left: {int(self.left_curve * 100)}cm",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)


        if self.right_curve is None:
            cv2.putText(sliding_img_right, "Carril derecho no detectado",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        else:
            cv2.putText(sliding_img_right, f"Right: {int(self.right_curve * 100)}cm",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)


        # ── Publicar PWM ──────────────────────────────────────────────────────
        pwm_msg      = String()
        pwm_msg.data = str(pwm)
        self.direction_pwm_pub.publish(pwm_msg)


        throttle_msg      = String()
        throttle_msg.data = str(2610)
        self.throttle_pwm_pub.publish(throttle_msg)


        end_time = time.time()
        # ── Visualización ─────────────────────────────────────────────────────
        if SHOW_VIZ:
            warped_viz_l = cv2.cvtColor(warped_bottom_l,  cv2.COLOR_GRAY2BGR)
            warped_viz_r = cv2.cvtColor(warped_bottom_r, cv2.COLOR_GRAY2BGR)

            for viz, fit_line, angle in [
                (warped_viz_l, fit_line_l, slope_left),
                (warped_viz_r, fit_line_r, slope_right),
            ]:
                if fit_line is not None:
                    a, b = fit_line
                    # Dibujar la recta de extremo a extremo (y=0 a y=h)
                    y0, y1 = 0, viz.shape[0] - 1
                    x0 = int(a * y0 + b)
                    x1 = int(a * y1 + b)
                    cv2.line(viz, (x0, y0), (x1, y1), (0, 0, 255), 2)
                    # Mostrar el ángulo
                    cv2.putText(viz, f"slope: {angle:+.1f} deg",
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                else:
                    cv2.putText(viz, "Sin slope", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            #cv2.imshow("Warped + Slope Left",  warped_viz_l)
            cv2.imshow("Warped + Slope Right", warped_viz_r)
            #cv2.imshow("Original left",        img_bgr_left)
            #cv2.imshow("Original right",       img_bgr_right)
            #cv2.imshow("Sliding Window Left",  sliding_img_left)
            cv2.imshow("Sliding Window Right", sliding_img_right)
            #cv2.imshow("Warped left",          warped_left)
            #cv2.imshow("Warped right",         warped_right)
            #cv2.imshow("roi l",                roi_l)
            cv2.imshow("ROI Overlay", roi_overlay)
            #cv2.imshow("roi R",                roi_r)
            cv2.waitKey(1)


        self.get_logger().info(f"Tiempo de procesamiento: {(end_time - start_time) * 1000:.2f} ms")

import signal

def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectorNode()

    def stop_handler(sig, frame):
        stop_dir = String()
        stop_dir.data = "2642"
        stop_thr = String()
        stop_thr.data = "2457"
        for _ in range(10):
            node.direction_pwm_pub.publish(stop_dir)
            node.throttle_pwm_pub.publish(stop_thr)
            time.sleep(0.05)
        print("Motores detenidos")
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, stop_handler)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
