#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import cv2
import numpy as np
import math
import threading
import queue
from concurrent.futures import ThreadPoolExecutor
from ultralytics import YOLO
import torch.nn.functional as F
import pyzed.sl as sl


# ─────────────────────────────────────────────
# CONFIG
# ─────────────────────────────────────────────
MODEL_PATH = "/home/traxxas/Workspaces/traxxas_pruebas/best_m.engine"
RUTA_SVO   = ""
USE_SVO    = False
CONF       = 0.15


def iniciar_zed(ruta_svo=None):
    zed         = sl.Camera()
    init_params = sl.InitParameters()
    init_params.depth_mode       = sl.DEPTH_MODE.PERFORMANCE
    init_params.coordinate_units = sl.UNIT.METER

    if ruta_svo:
        print(f"Modo: SVO -> {ruta_svo}")
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


def yolo_mask_to_binary(result, target_h, target_w):
    if result.masks is None:
        return np.zeros((target_h, target_w), dtype=np.uint8)
    masks_gpu = result.masks.data
    if masks_gpu.shape[0] == 0:
        return np.zeros((target_h, target_w), dtype=np.uint8)
    combined = masks_gpu.max(dim=0).values
    resized = F.interpolate(
        combined.float().unsqueeze(0).unsqueeze(0),
        size=(target_h, target_w),
        mode='bilinear',
        align_corners=False
    ).squeeze()
    return (resized > 0.5).byte().cpu().numpy() * 255


def detectar_angulos_en_mascara(binary_mask, vertices, usar_borde_derecho_roi=False):
    """
    Toma la máscara binaria limpia de YOLO, aplica ROI,
    y usa HoughLinesP para sacar ángulos. Permite inyectar
    el borde derecho de la ROI como una línea virtual.
    """
    # Aplicar ROI
    roi_mask = np.zeros_like(binary_mask)
    cv2.fillPoly(roi_mask, vertices, 255)
    roi = cv2.bitwise_and(binary_mask, binary_mask, mask=roi_mask)

    # Bordes de la máscara limpia
    edges = cv2.Canny(roi, 50, 150)

    # Detectar líneas
    lineas = cv2.HoughLinesP(edges, rho=1, theta=np.pi / 180,
                              threshold=40, minLineLength=30, maxLineGap=100)

    angulos = []
    lineas_salida = []

    if lineas is not None:
        for linea in lineas:
            x1, y1, x2, y2 = linea[0]
            lineas_salida.append((x1, y1, x2, y2))

            if y2 > y1:
                x1, y1, x2, y2 = x2, y2, x1, y1

            angulo_rad = math.atan2(y2 - y1, x2 - x1)
            angulo_deg = math.degrees(angulo_rad)

            if 20 < abs(angulo_deg) < 160:
                angulos.append(angulo_deg)

    # --- NUEVA LÓGICA: INVENTAR CARRIL DERECHO USANDO EL BORDE DEL ROI ---
    if usar_borde_derecho_roi:
        # En tu arreglo, el índice 1 es Abajo-Derecha y el índice 2 es Arriba-Derecha
        br_x, br_y = vertices[0][1] # Bottom-Right
        tr_x, tr_y = vertices[0][2] # Top-Right
        
        # Lo agregamos a lineas_salida para que lo veas dibujado de verde en el imshow
        lineas_salida.append((br_x, br_y, tr_x, tr_y))
        
        # Calculamos el ángulo con tu misma fórmula
        x1, y1 = br_x, br_y
        x2, y2 = tr_x, tr_y
        
        if y2 > y1:
            x1, y1, x2, y2 = x2, y2, x1, y1

        angulo_rad = math.atan2(y2 - y1, x2 - x1)
        angulo_deg = math.degrees(angulo_rad)

        if 20 < abs(angulo_deg) < 160:
            angulos.append(angulo_deg)
    # ---------------------------------------------------------------------

    return angulos, lineas_salida, roi


class LaneDetector(Node):
    def __init__(self):
        super().__init__('lane_detector_camera')

        # --- SWITCHES ---
        self.MOSTRAR_VENTANAS = False

        # --- CALIBRACIÓN ---
        self.OFFSET_ANGULO = -20#10

        # --- PUBLICADORES ---
        self.raw_pub = self.create_publisher(Float32, '/qcar/lane_angle_raw', 10)
        self.ema_pub = self.create_publisher(Float32, '/qcar/lane_angle_ema', 10)

        # --- EMA ---
        self.ema_angle = None
        self.alpha = 0.7

        # --- ZED (SDK nativo, ambos ojos) ---
        self.zed = iniciar_zed(RUTA_SVO if USE_SVO else None)
        if self.zed is None:
            self.get_logger().error('No se pudo abrir la ZED.')
            raise RuntimeError("Error ZED")

        # --- YOLO ---
        self.model = YOLO(MODEL_PATH, task="segment")
        self.get_logger().info("YOLO engine listo.")

        # --- HILO PRODUCTOR (lee frames sin bloquear el nodo) ---
        self.frame_queue = queue.Queue(maxsize=1)
        self.stop_event  = threading.Event()
        self.prod_thread = threading.Thread(target=self._zed_producer, daemon=True)
        self.prod_thread.start()

        # --- POOL PARA PROCESAR AMBOS OJOS EN PARALELO ---
        self.executor_cv = ThreadPoolExecutor(max_workers=2)

        # --- TIMER ---
        self.timer = self.create_timer(0.05, self.process_frame)

        self.get_logger().info(
            f'Visión YOLO iniciada. Ventanas: {"ON" if self.MOSTRAR_VENTANAS else "OFF"}'
        )

    # ──────────────────────────────────────────
    # HILO PRODUCTOR ZED
    # ──────────────────────────────────────────
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

    # ──────────────────────────────────────────
    # CALLBACK PRINCIPAL
    # ──────────────────────────────────────────
    def process_frame(self):
        try:
            item = self.frame_queue.get_nowait()
        except queue.Empty:
            return
        if item is None:
            return

        img_left, img_right = item
        h, w = img_left.shape[:2]

        try:
            # ── YOLO batch: ambos ojos de un jalón ───────────────
            results = self.model(
                [img_left, img_right],
                conf         = CONF,
                half         = False,
                imgsz        = 640,
                classes      = [0],
                retina_masks = True,
                verbose      = False,
            )

            mask_left  = yolo_mask_to_binary(results[0], h, w)
            mask_right = yolo_mask_to_binary(results[1], h, w)
            
            # ── ROIs INDEPENDIENTES ────────
            # El orden de los puntos es:
            # 1. Abajo-Izquierda, 2. Abajo-Derecha, 3. Arriba-Derecha, 4. Arriba-Izquierda

            # ROI para el ojo izquierdo no mover
            vertices_l = np.array([[
                (int(w * 0.19), int(h * 0.99)), #abajo a la izq
                (int(w * 1.02), int(h * 0.97)),#abajo a la derecha 97
                (int(w * 0.68), int(h * 0.68)), #arriba a la derecha 68
                (int(w * 0.34), int(h * 0.70))  #arriba a al izqueirda
            ]], dtype=np.int32)

            # ROI para el ojo derecho no mover
            vertices_r = np.array([[
                (int(w * 0.0), h), #abajo a la izq
                (int(w * 0.85), h), #abajo a la derecha
                (int(w * 0.69), int(h * 0.77)), #arriba a la derecha
                (int(w * 0.18), int(h * 0.75))  #arriba a al izqueirda
            ]], dtype=np.int32)

            # ── Procesar ambos ojos en paralelo ──────────────────
            # Al ojo izquierdo le pasamos True para que se invente la línea con su ROI
            fut_l = self.executor_cv.submit(
                detectar_angulos_en_mascara, mask_left, vertices_l, True
            )
            # Al ojo derecho le pasamos False para que trabaje normal
            fut_r = self.executor_cv.submit(
                detectar_angulos_en_mascara, mask_right, vertices_r, False
            )

            # # ── Procesar ambos ojos en paralelo ──────────────────
            # fut_l = self.executor_cv.submit(
            #     detectar_angulos_en_mascara, mask_left, vertices_l
            # )
            # fut_r = self.executor_cv.submit(
            #     detectar_angulos_en_mascara, mask_right, vertices_r
            # )

            angulos_l, lineas_l, roi_l = fut_l.result()
            angulos_r, lineas_r, roi_r = fut_r.result()

            # ── Juntar ángulos de ambos ojos ─────────────────────
            angulos = angulos_l + angulos_r

            # ── Visualización ────────────────────────────────────
            if self.MOSTRAR_VENTANAS:
                # Dibujar líneas sobre copia de cada ojo
                debug_l = img_left.copy()
                debug_r = img_right.copy()

                for x1, y1, x2, y2 in lineas_l:
                    cv2.line(debug_l, (x1, y1), (x2, y2), (0, 255, 0), 3)
                for x1, y1, x2, y2 in lineas_r:
                    cv2.line(debug_r, (x1, y1), (x2, y2), (0, 255, 0), 3)
               
                cv2.polylines(debug_l, [vertices_l], True, (255, 0, 0), 2)
                cv2.polylines(debug_r, [vertices_r], True, (0, 255, 255), 2) # Lo puse amarillo para distinguirlo

                cv2.imshow("Lineas - Ojo Izq", debug_l)
                cv2.imshow("Lineas - Ojo Der", debug_r)
                cv2.imshow("Mascara YOLO Izq", mask_left)
                cv2.imshow("Mascara YOLO Der", mask_right)
                cv2.imshow("ROI Izq", roi_l)
                cv2.imshow("ROI Der", roi_r)
                cv2.waitKey(1)

            # ── Publicar ángulo (misma lógica original) ──────────
            if angulos:
                angulo_promedio = sum(angulos) / len(angulos)
                angulo_promedio += self.OFFSET_ANGULO

                msg_raw = Float32()
                msg_raw.data = angulo_promedio
                self.raw_pub.publish(msg_raw)

                if self.ema_angle is None:
                    self.ema_angle = angulo_promedio
                else:
                    self.ema_angle = (self.alpha * angulo_promedio) + \
                                     ((1.0 - self.alpha) * self.ema_angle)

                msg_ema = Float32()
                msg_ema.data = self.ema_angle
                self.ema_pub.publish(msg_ema)

                self.get_logger().info(
                    f'Raw: {angulo_promedio:.1f}° | EMA: {self.ema_angle:.1f}° '
                    f'(Offset: {self.OFFSET_ANGULO})'
                )

        except Exception as e:
            self.get_logger().error(f'Error en procesamiento: {e}')

    # ──────────────────────────────────────────
    def destroy_node(self):
        self.stop_event.set()
        if hasattr(self, 'prod_thread'):
            self.prod_thread.join(timeout=2.0)
        if hasattr(self, 'executor_cv'):
            self.executor_cv.shutdown(wait=False)
        if hasattr(self, 'zed') and self.zed is not None:
            self.zed.close()
            self.get_logger().info('ZED cerrada.')
        if self.MOSTRAR_VENTANAS:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()