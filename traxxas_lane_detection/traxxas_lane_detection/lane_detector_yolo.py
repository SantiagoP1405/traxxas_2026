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


# ─────────────────────────────────────────────
# CONFIG
# ─────────────────────────────────────────────
MODEL_PATH  = "/home/traxxas/Workspaces/traxxas_pruebas/best.engine"
RUTA_SVO    = "/home/traxxas/Documents/ZED/Vuelta_derecha.svo2"
#RUTA_SVO    = ""
USE_SVO     = True
CONF        = 0.35
COLOR_LEFT  = (0,   255, 100)
COLOR_RIGHT = (0,   180, 255)
ALPHA       = 0.45
NEW_TOP     = 0.54 #este es para la ROI
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

    masks_gpu = result.masks.data                    # (N, h, w) CUDA tensor
    combined  = masks_gpu.max(dim=0).values          # (h, w) en GPU

    resized = F.interpolate(
        combined.float().unsqueeze(0).unsqueeze(0),  # (1,1,h,w)
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

    sliding_img, fit, ploty = sliding_window(warped)

    curve = measure_curvature(ploty, fit) if (fit is not None and ploty is not None) else None

    return sliding_img, warped, roi_mask, curve


def sliding_window(binary_warped):
    histogram = np.sum(binary_warped, axis=0)
    histogram = cv2.GaussianBlur(histogram.astype(np.float32), (51, 1), 0)
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

    #print(f"  [SW] warped shape: {binary_warped.shape} | píxeles totales: {len(nonzerox)} | base: {base}")

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

    lane_inds = np.concatenate(lane_inds)
    #print("lane inds concatenados: ", lane_inds)
    x = nonzerox[lane_inds]
    #print("lane en x: ", x)
    y = nonzeroy[lane_inds]
    #print("lane en y: ", y)
    #print(f"  [SW] píxeles capturados por ventanas: {len(x)}")

    if len(x) < 10:
        #print(f"  [SW] ❌ RECHAZADO: len(x)={len(x)} < 10")
        return out_img, None, None

    fit = np.polyfit(y, x, 2)
    #print("fit: ", fit)
    #print(f"  [SW] fit[0]={fit[0]:.7f}  fit[1]={fit[1]:.4f}  fit[2]={fit[2]:.2f}")

    if abs(fit[0]) > 0.003:
        #print(f"  [SW] ❌ RECHAZADO: |fit[0]|={abs(fit[0]):.7f} > 0.003")
        return out_img, None, None

    #print(f"  [SW] ✅ fit aceptado")
    ploty = np.linspace(0, binary_warped.shape[0] - 1, binary_warped.shape[0])
    fitx  = fit[0] * ploty**2 + fit[1] * ploty + fit[2]

    out_img[nonzeroy[lane_inds], nonzerox[lane_inds]] = [255, 0, 0]
    for i in range(len(ploty)):
        cv2.circle(out_img, (int(fitx[i]), int(ploty[i])), 3, (255, 255, 0), -1)

    return out_img, fit, ploty


def measure_curvature(ploty, fit):
    ym_per_pix = 1.4  / 720
    xm_per_pix = 0.40 / (0.30 * 1280)  # ← sospechoso principal

    y_eval = np.max(ploty)
    fitx   = fit[0] * ploty**2 + fit[1] * ploty + fit[2]

    fit_cr = np.polyfit(ploty * ym_per_pix, fitx * xm_per_pix, 2)

    curverad = (
        (1 + (2 * fit_cr[0] * y_eval * ym_per_pix + fit_cr[1])**2)**1.5
    ) / np.absolute(2 * fit_cr[0])

    #print(f"  [CURV] ym_per_pix={ym_per_pix:.6f}  xm_per_pix={xm_per_pix:.6f}")
    #print(f"  [CURV] fit_cr[0]={fit_cr[0]:.4f}  fit_cr[1]={fit_cr[1]:.4f}")
    #print(f"  [CURV] radio calculado: {curverad:.4f} m")
    print("curve rad: ", curverad)
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



def angle_to_pwm(delta_deg, left_curv, right_curv):
    pwm_min    = 1669
    pwm_center = 2642
    pwm_max    = 3276
    max_steer_deg = 30.0

    if right_curv is not None and left_curv is not None and right_curv > left_curv:
        delta_deg = -abs(delta_deg)
    elif right_curv is not None and left_curv is not None and left_curv > right_curv:
        delta_deg = abs(delta_deg)
    elif right_curv is None and left_curv is not None:
        delta_deg = -abs(delta_deg)
    elif left_curv is None and right_curv is not None:
        delta_deg = abs(delta_deg)

    delta_deg = max(-max_steer_deg, min(max_steer_deg, delta_deg))

    norm = delta_deg / max_steer_deg

    if norm >= 0:
        pwm = pwm_center + norm * (pwm_max - pwm_center)
    else:
        pwm = pwm_center + norm * (pwm_center - pwm_min)

    return int(pwm)


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
	
	# ── Kalman 1D - Right ─────────────────────────────────────────────────
	self.kf_x_right = None   # estado estimado
	self.kf_P_right = 10.0   # covarianza del error
	self.kf_Q       = 0.5    # ruido de proceso
	self.kf_R       = 8.0    # ruido de medición

	# ── Kalman 1D - Left ──────────────────────────────────────────────────
	self.kf_x_left  = None
	self.kf_P_left  = 10.0
        self.timer = self.create_timer(1/30, self.timer_callback)

    # ──────────────────────────────────────────────────────────────────────────
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
            retina_masks = False,
            verbose      = False,
        )

        mask_left  = yolo_mask_to_binary(results[0], h, w)
        mask_right = yolo_mask_to_binary(results[1], h, w)

        # ── ROI ───────────────────────────────────────────────────────────────
        vertices_left = np.array([[
            (int(0.18 * w), int(NEW_TOP * h)),   # TL
            (int(0.44 * w), int(NEW_TOP * h)),   # TR
            (int(0.50 * w), int(0.78 * h)),      # BR
            (int(0.06 * w), int(0.78 * h)),      # BL
        ]], dtype=np.int32)

        vertices_right = np.array([[
            (int(0.56 * w), int(NEW_TOP * h)),   # TL
            (int(0.82 * w), int(NEW_TOP * h)),   # TR
            (int(1.00 * w), int(0.78 * h)),      # BR
            (int(0.52 * w), int(0.78 * h)),      # BL
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
            [int(0.35 * w), 0],
            [int(0.65 * w), 0],
            [int(0.65 * w), h],
            [int(0.35 * w), h],
        ], dtype=np.int32)

        # ── Paralelo: ROI + warp + sliding ────────────────────────────────────
        fut_l = self.executor_cv.submit(process_eye, mask_left,  vertices_left,  dst1)
        fut_r = self.executor_cv.submit(process_eye, mask_right, vertices_right, dst1)

        sliding_img_left,  warped_left,  roi_l, self.left_curve  = fut_l.result()
        sliding_img_right, warped_right, roi_r, self.right_curve = fut_r.result()
        
        # ── Kalman 1D Right ───────────────────────────────────────────────────
        #quedo mal calibrado
	raw_right = self.right_curve
	if raw_right is None:
	    # Sin detección: solo propaga incertidumbre
	    if self.kf_x_right is not None:
		self.kf_P_right += self.kf_Q
	    self.right_curve = self.kf_x_right
	else:
	    if self.kf_x_right is None:
		# Primera medición válida: inicializar
		self.kf_x_right = raw_right
	    else:
		# Predict
		P_pred          = self.kf_P_right + self.kf_Q
		# Update
		K               = P_pred / (P_pred + self.kf_R)
		self.kf_x_right = self.kf_x_right + K * (raw_right - self.kf_x_right)
		self.kf_P_right = (1 - K) * P_pred
	    self.right_curve = self.kf_x_right

	# ── Kalman 1D Left ────────────────────────────────────────────────────
	raw_left = self.left_curve
	if raw_left is None:
	    if self.kf_x_left is not None:
		self.kf_P_left += self.kf_Q
	    self.left_curve = self.kf_x_left
	else:
	    if self.kf_x_left is None:
		self.kf_x_left = raw_left
	    else:
		P_pred         = self.kf_P_left + self.kf_Q
		K              = P_pred / (P_pred + self.kf_R)
		self.kf_x_left = self.kf_x_left + K * (raw_left - self.kf_x_left)
		self.kf_P_left = (1 - K) * P_pred
	    self.left_curve = self.kf_x_left

        # ── PWM ───────────────────────────────────────────────────────────────
        if self.left_curve is None and self.right_curve is None:
            pwm = 2642
            self.get_logger().warn("No se detectaron carriles → PWM centro")
        else:
            delta_deg = steering_from_curvature(self.left_curve, self.right_curve)
            pwm       = angle_to_pwm(delta_deg, self.left_curve, self.right_curve)
            self.get_logger().info("Ángulo dirección: {:.2f}°  PWM: {}".format(delta_deg, pwm))

        # ── Overlay texto ─────────────────────────────────────────────────────
        # if self.left_curve is None:
        #     cv2.putText(sliding_img_left, "Carril izquierdo no detectado",
        #                 (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        # else:
        #     cv2.putText(sliding_img_left, f"Left: {int(self.left_curve * 100)}cm",
        #                 (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

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
        throttle_msg.data = str(2625)
        self.throttle_pwm_pub.publish(throttle_msg)

        end_time = time.time()

        # ── Visualización ─────────────────────────────────────────────────────
        #cv2.imshow("Original left",        img_bgr_left)
        #cv2.imshow("Original right",       img_bgr_right)
        #cv2.imshow("Sliding Window Left",  sliding_img_left)
        cv2.imshow("Sliding Window Right", sliding_img_right)
        #cv2.imshow("Warped left",          warped_left)
        #cv2.imshow("Warped right",         warped_right)
        #cv2.imshow("roi l",                roi_l)
        #cv2.imshow("ROI Overlay", roi_overlay)
        #cv2.imshow("roi R",                roi_r)
        cv2.waitKey(1)

        #self.get_logger().info(f"Tiempo de procesamiento: {(end_time - start_time) * 1000:.2f} ms")


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
