#!/usr/bin/env python3
"""
=======================================================================
 CrosswalkDetector  -  traxxas_stop
-----------------------------------------------------------------------
 Detecta pasos peatonales (franjas transversales blanco-negro sobre
 piso negro) usando la imagen RGB de la ZED2i (1280x720, horizontal).

 ESTRATEGIA
 ----------
 El paso peatonal aparece en la franja INFERIOR-CENTRAL de la imagen
 (el suelo cercano al carro). Las lineas del carril aparecen en los
 BORDES laterales de esa misma franja.

 Pasos del algoritmo:
   1. ROI: recortar franja inferior-central de la imagen
      (filas 65%-100%, columnas 20%-80%)
      - Inferior: donde cae el suelo con camara horizontal
      - Central:  evita los bordes donde estan las lineas del carril

   2. Escala de grises + umbral adaptativo (robusto a iluminacion)

   3. Proyeccion de perfil horizontal:
      Sumar pixeles blancos por FILA dentro del ROI.
      Un paso peatonal produce picos altos alternados (franja blanca)
      separados por valles (franja negra).
      Una linea de carril produce una banda uniforme solo en los bordes,
      que ya fue recortada por el ROI.

   4. Analisis de picos:
      - Buscar picos en la proyeccion vertical
      - Validar que hay >= min_stripes picos (franjas blancas)
      - Validar que el ancho de cada pico es consistente con 2 cm
        proyectados a la distancia estimada
      - Validar que los picos cubren al menos el 60% del ancho del ROI

   5. Distancia: profundidad ZED en el centro del ROI detectado

 TOPICOS SUSCRITOS
   /zed/zed_node/left/image_rect_color   Image (BGR)
   /zed/zed_node/depth/depth_registered  Image (32FC1, metros)

 TOPICOS PUBLICADOS
   /traxxas/crosswalk/detected_raw   Bool
   /traxxas/crosswalk/distance_m     Float32  (-1.0 si invalida)
   /traxxas/crosswalk/stripe_count   Float32  (num franjas detectadas)
   /traxxas/crosswalk/debug_image    Image    (solo si debug=True)

 PARAMETROS
   color_topic        str    topico de imagen RGB
   depth_topic        str    topico de profundidad
   roi_top_frac       float  fraccion superior del ROI (default 0.65)
   roi_bottom_frac    float  fraccion inferior del ROI (default 1.00)
   roi_left_frac      float  fraccion izquierda del ROI (default 0.20)
   roi_right_frac     float  fraccion derecha del ROI  (default 0.80)
   min_stripes        int    minimo de franjas blancas para detectar (3)
   min_stripe_width   int    ancho minimo de una franja en pixeles (3)
   max_stripe_width   int    ancho maximo de una franja en pixeles (40)
   white_threshold    int    umbral de brillo para considerar blanco (180)
   min_fill_ratio     float  fraccion minima del ROI cubierta (0.50)
   debug              bool   publicar imagen de debug
=======================================================================
"""

import math
import cv2
import numpy as np
import rclpy

from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32
from message_filters import Subscriber, ApproximateTimeSynchronizer


class CrosswalkDetector(Node):

    def __init__(self):
        super().__init__('crosswalk_detector')

        self.bridge = CvBridge()

        # -- Parametros -----------------------------------------------
        self.declare_parameter('color_topic',
            '/zed/zed_node/left/image_rect_color')
        self.declare_parameter('depth_topic',
            '/zed/zed_node/depth/depth_registered')

        # ROI como fraccion de la imagen (0.0 a 1.0)
        # Con camara horizontal 1280x720:
        #   filas 65-100% = suelo cercano (~30-80 cm frente al carro)
        #   cols  20-80%  = zona central, excluye lineas de carril laterales
        self.declare_parameter('roi_top_frac',    0.65)
        self.declare_parameter('roi_bottom_frac', 1.00)
        self.declare_parameter('roi_left_frac',   0.20)
        self.declare_parameter('roi_right_frac',  0.80)

        # Deteccion de franjas
        self.declare_parameter('min_stripes',      3)    # franjas blancas minimas
        self.declare_parameter('min_stripe_width', 3)    # px ancho minimo franja
        self.declare_parameter('max_stripe_width', 40)   # px ancho maximo franja
        self.declare_parameter('white_threshold',  180)  # brillo minimo para "blanco"
        self.declare_parameter('min_fill_ratio',   0.50) # fraccion del ROI cubierta

        self.declare_parameter('debug', True)

        # Leer parametros
        self.color_topic     = self.get_parameter('color_topic').value
        self.depth_topic     = self.get_parameter('depth_topic').value
        self.roi_top_frac    = float(self.get_parameter('roi_top_frac').value)
        self.roi_bot_frac    = float(self.get_parameter('roi_bottom_frac').value)
        self.roi_left_frac   = float(self.get_parameter('roi_left_frac').value)
        self.roi_right_frac  = float(self.get_parameter('roi_right_frac').value)
        self.min_stripes     = int(self.get_parameter('min_stripes').value)
        self.min_sw          = int(self.get_parameter('min_stripe_width').value)
        self.max_sw          = int(self.get_parameter('max_stripe_width').value)
        self.white_thresh    = int(self.get_parameter('white_threshold').value)
        self.min_fill        = float(self.get_parameter('min_fill_ratio').value)
        self.debug_enabled   = bool(self.get_parameter('debug').value)

        # -- Publicadores ---------------------------------------------
        self.pub_raw      = self.create_publisher(
            Bool,    '/traxxas/crosswalk/detected_raw', 10)
        self.pub_distance = self.create_publisher(
            Float32, '/traxxas/crosswalk/distance_m',   10)
        self.pub_stripes  = self.create_publisher(
            Float32, '/traxxas/crosswalk/stripe_count',  10)
        self.pub_debug    = self.create_publisher(
            Image,   '/traxxas/crosswalk/debug_image',   10)

        # -- Suscripciones sincronizadas (color + depth) --------------
        self.color_sub = Subscriber(self, Image, self.color_topic)
        self.depth_sub = Subscriber(self, Image, self.depth_topic)
        self.sync = ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub],
            queue_size=10,
            slop=0.1,
        )
        self.sync.registerCallback(self.synced_callback)

        self.get_logger().info('CrosswalkDetector iniciado')
        self.get_logger().info(
            f'  ROI filas [{self.roi_top_frac:.0%} - {self.roi_bot_frac:.0%}] '
            f'cols [{self.roi_left_frac:.0%} - {self.roi_right_frac:.0%}]'
        )
        self.get_logger().info(
            f'  min_stripes={self.min_stripes} '
            f'stripe_width=[{self.min_sw},{self.max_sw}]px'
        )

    # =================================================================
    #  DETECCION PRINCIPAL
    # =================================================================

    def detect_crosswalk(self, frame):
        """
        Detecta franjas transversales de paso peatonal en el ROI.

        Retorna:
          detected    bool
          stripe_count int   numero de franjas blancas encontradas
          roi_coords  tuple  (y0, y1, x0, x1) del ROI en imagen original
          debug_frame ndarray imagen BGR con anotaciones
        """
        h, w = frame.shape[:2]
        debug = frame.copy()

        # -- 1. Calcular coordenadas del ROI --------------------------
        y0 = int(h * self.roi_top_frac)
        y1 = int(h * self.roi_bot_frac)
        x0 = int(w * self.roi_left_frac)
        x1 = int(w * self.roi_right_frac)

        # Dibujar ROI en debug
        cv2.rectangle(debug, (x0, y0), (x1, y1), (255, 200, 0), 2)
        cv2.putText(debug, 'ROI', (x0 + 4, y0 + 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 200, 0), 1)

        roi = frame[y0:y1, x0:x1]
        roi_h, roi_w = roi.shape[:2]

        if roi_h < 10 or roi_w < 10:
            return False, 0, (y0, y1, x0, x1), debug

        # -- 2. Gris + umbral -----------------------------------------
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

        # Umbral adaptativo: robusto a variaciones de iluminacion.
        # blockSize=51 cubre un area suficiente para ver franjas de ~2cm.
        binary = cv2.adaptiveThreshold(
            gray, 255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,
            blockSize=51,
            C=-10,
        )

        # Morfologia: cierra pequenos huecos dentro de las franjas
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

        # -- 3. Proyeccion de perfil horizontal (suma por fila) --------
        # Cada fila da un valor entre 0 (todo negro) y roi_w*255 (todo blanco).
        # Normalizamos a [0, 255].
        profile = binary.sum(axis=1).astype(np.float32)
        if profile.max() > 0:
            profile_norm = (profile / profile.max() * 255).astype(np.uint8)
        else:
            profile_norm = profile.astype(np.uint8)

        # Umbral del perfil: fila es "blanca" si > white_threshold
        row_is_white = profile_norm > self.white_thresh

        # -- 4. Encontrar franjas blancas consecutivas ----------------
        stripes = []          # lista de (fila_inicio, fila_fin, ancho_px)
        in_stripe   = False
        stripe_start = 0

        for row_idx, is_white in enumerate(row_is_white):
            if is_white and not in_stripe:
                in_stripe    = True
                stripe_start = row_idx
            elif not is_white and in_stripe:
                in_stripe = False
                width = row_idx - stripe_start
                if self.min_sw <= width <= self.max_sw:
                    stripes.append((stripe_start, row_idx, width))

        # Cerrar ultima franja si termina en el borde
        if in_stripe:
            width = roi_h - stripe_start
            if self.min_sw <= width <= self.max_sw:
                stripes.append((stripe_start, roi_h, width))

        stripe_count = len(stripes)

        # -- 5. Validacion de cobertura lateral -----------------------
        # Las franjas del paso peatonal deben cubrir un buen porcentaje
        # del ancho del ROI (no son solo bordes).
        # Tomamos la franja con mayor cobertura horizontal.
        max_coverage = 0.0
        if stripes:
            for (rs, re, _) in stripes:
                stripe_row_binary = binary[rs:re, :]
                white_cols = (stripe_row_binary > 0).any(axis=0).sum()
                coverage = white_cols / max(roi_w, 1)
                max_coverage = max(max_coverage, coverage)

        detected = (stripe_count >= self.min_stripes and
                    max_coverage >= self.min_fill)

        # -- 6. Anotaciones de debug ----------------------------------
        # Dibujar perfil como histograma lateral
        profile_vis_w = 60
        for row_idx, val in enumerate(profile_norm):
            bar_len = int(val / 255 * profile_vis_w)
            color = (0, 255, 0) if row_is_white[row_idx] else (80, 80, 80)
            cv2.line(debug,
                     (x1 + 2, y0 + row_idx),
                     (x1 + 2 + bar_len, y0 + row_idx),
                     color, 1)

        # Dibujar franjas detectadas en el ROI
        for (rs, re, sw) in stripes:
            cv2.rectangle(debug,
                          (x0, y0 + rs),
                          (x1, y0 + re),
                          (0, 255, 0), 1)

        status = 'CROSSWALK' if detected else 'no crosswalk'
        color  = (0, 255, 0) if detected else (0, 0, 255)
        cv2.putText(debug,
                    f'{status} | franjas={stripe_count} | cob={max_coverage:.0%}',
                    (x0, y0 - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)

        return detected, stripe_count, (y0, y1, x0, x1), debug

    # =================================================================
    #  PROFUNDIDAD EN EL CENTRO DEL ROI
    # =================================================================

    def get_depth_at_roi_center(self, depth_img, roi_coords) -> float:
        """
        Mide la profundidad en el centro del ROI detectado.
        Usa una ventana 10x10 y devuelve la mediana de valores validos.
        """
        y0, y1, x0, x1 = roi_coords
        cy = (y0 + y1) // 2
        cx = (x0 + x1) // 2
        h, w = depth_img.shape[:2]

        py0 = max(0, cy - 5)
        py1 = min(h, cy + 5)
        px0 = max(0, cx - 5)
        px1 = min(w, cx + 5)

        patch = depth_img[py0:py1, px0:px1]
        valid = patch[np.isfinite(patch)]
        valid = valid[valid > 0.0]

        return float(np.median(valid)) if valid.size > 0 else float('nan')

    # =================================================================
    #  CALLBACK SINCRONIZADO
    # =================================================================

    def synced_callback(self, color_msg, depth_msg):
        # Convertir imagen
        try:
            frame = self.bridge.imgmsg_to_cv2(
                color_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error convirtiendo imagen: {e}')
            return

        try:
            depth = self.bridge.imgmsg_to_cv2(
                depth_msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Error convirtiendo profundidad: {e}')
            return

        # Deteccion
        detected, stripe_count, roi_coords, debug = \
            self.detect_crosswalk(frame)

        # Profundidad
        distance_m = float('nan')
        if detected:
            distance_m = self.get_depth_at_roi_center(depth, roi_coords)
            if math.isfinite(distance_m):
                y0, y1, x0, x1 = roi_coords
                cv2.putText(debug,
                            f'dist={distance_m:.2f} m',
                            (x0, y1 + 18),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (255, 255, 0), 2)

        # Publicar
        raw_msg = Bool()
        raw_msg.data = detected
        self.pub_raw.publish(raw_msg)

        dist_msg = Float32()
        dist_msg.data = distance_m if math.isfinite(distance_m) else -1.0
        self.pub_distance.publish(dist_msg)

        stripe_msg = Float32()
        stripe_msg.data = float(stripe_count)
        self.pub_stripes.publish(stripe_msg)

        if self.debug_enabled:
            try:
                dbg_msg = self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
                dbg_msg.header = color_msg.header
                self.pub_debug.publish(dbg_msg)
            except Exception as e:
                self.get_logger().error(f'Error publicando debug: {e}')


# =====================================================================
def main(args=None):
    rclpy.init(args=args)
    node = CrosswalkDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()