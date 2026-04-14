#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import cv2
import numpy as np
import math

class LaneDetector(Node):
    def __init__(self):
        super().__init__('lane_detector_camera')
        
        # --- SWITCH PARA ACTIVAR/DESACTIVAR VENTANAS ---
        self.MOSTRAR_VENTANAS = False  
        
        # ==========================================================
        # --- CALIBRACIÓN ZED2 ---
        # ==========================================================
        # 1. DERECHO IZQUEIRDO
        self.LENTE_A_USAR = 'DERECHO' 
        
        # 2. Offset de ángulo
        self.OFFSET_ANGULO = -19.0  
        # ==========================================================

        # --- PUBLICADORES ---
        self.raw_pub = self.create_publisher(Float32, '/qcar/lane_angle_raw', 10)
        self.ema_pub = self.create_publisher(Float32, '/qcar/lane_angle_ema', 10)
        
        # --- VARIABLES DEL FILTRO EMA ---
        self.ema_angle = None
        self.alpha = 0.5 

        # --- INICIALIZAR CÁMARA ZED (Vía OpenCV) ---
        self.cap = cv2.VideoCapture(0)
        
        # La ZED en modo VGA manda una imagen Side-by-Side de 1344x376
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1344)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 376)

        if not self.cap.isOpened():
            self.get_logger().error('¡Error! No se pudo abrir la cámara ZED. Revisa la conexión.')
            
        self.timer = self.create_timer(0.05, self.process_frame)

        if self.MOSTRAR_VENTANAS:
            self.get_logger().info(f'Visión iniciada. Modo visual ACTIVADO. Usando lente {self.LENTE_A_USAR}.')
        else:
            self.get_logger().info(f'Visión iniciada en modo "headless". Usando lente {self.LENTE_A_USAR}.')

    def process_frame(self):
        ret, frame_completo = self.cap.read()
        
        if not ret or frame_completo is None:
            self.get_logger().warn("Fallo al leer el frame. Reintentando...")
            return

        try:
            # --- ADAPTACIÓN ZED: EXTRAER OJO CORRECTO ---
            alto, ancho_total, _ = frame_completo.shape
            mitad = ancho_total // 2
            
            # Recortamos la imagen dependiendo de qué lente elegiste arriba
            if self.LENTE_A_USAR == 'IZQUIERDO':
                cv_image = frame_completo[:, :mitad]
            else:
                cv_image = frame_completo[:, mitad:]
            
            # Copia para dibujar si las ventanas están activadas
            img_debug = cv_image.copy() if self.MOSTRAR_VENTANAS else None

            # --- PROCESAMIENTO DE IMAGEN ---
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(gray, (7, 7), 0)
            canny = cv2.Canny(blur, 50, 150)
            
            alto_img, ancho_img = canny.shape
            
            # ROI dinámica
            vertices = np.array([[
                (0, alto_img),                        
                (ancho_img, alto_img),                    
                (int(ancho_img * 0.65), int(alto_img * 0.73)), 
                (int(ancho_img * 0.3), int(alto_img * 0.73))  
            ]], dtype=np.int32)
            
            mask = np.zeros_like(canny)
            cv2.fillPoly(mask, vertices, 255)
            roi_canny = cv2.bitwise_and(canny, mask)

            # Detectar líneas
            lineas = cv2.HoughLinesP(roi_canny, rho=1, theta=np.pi/180, threshold=40, 
                                     minLineLength=30, maxLineGap=100)

            angulos = []
            
            if lineas is not None:
                for linea in lineas:
                    x1, y1, x2, y2 = linea[0]
                    
                    # Dibujar líneas en la imagen de debug
                    if self.MOSTRAR_VENTANAS:
                        cv2.line(img_debug, (x1, y1), (x2, y2), (0, 255, 0), 3)
                    
                    if y2 > y1:
                        x1, y1, x2, y2 = x2, y2, x1, y1
                    
                    angulo_rad = math.atan2(y2 - y1, x2 - x1)
                    angulo_deg = math.degrees(angulo_rad)
                    
                    # Filtrar ángulos válidos
                    if abs(angulo_deg) > 20 and abs(angulo_deg) < 160:
                        angulos.append(angulo_deg)

            # --- MOSTRAR VENTANAS SI ESTÁ ACTIVADO ---
            if self.MOSTRAR_VENTANAS:
                # Dibujar el cuadro del ROI
                cv2.polylines(img_debug, [vertices], isClosed=True, color=(255, 0, 0), thickness=2)
                
                cv2.imshow("Binario / ROI", roi_canny)
                cv2.imshow("Vision / Lineas", img_debug)
                cv2.waitKey(1) 

            # --- PROCESAMIENTO Y PUBLICACIÓN ---
            if angulos:
                angulo_promedio = sum(angulos) / len(angulos)
                
                # APLICAMOS EL OFFSET DE CALIBRACIÓN AQUÍ
                angulo_promedio = angulo_promedio + self.OFFSET_ANGULO
                
                # Publicar crudo
                msg_raw = Float32()
                msg_raw.data = angulo_promedio
                self.raw_pub.publish(msg_raw)

                # EMA
                if self.ema_angle is None:
                    self.ema_angle = angulo_promedio 
                else:
                    self.ema_angle = (self.alpha * angulo_promedio) + ((1.0 - self.alpha) * self.ema_angle)
                
                msg_ema = Float32()
                msg_ema.data = self.ema_angle
                self.ema_pub.publish(msg_ema)

                self.get_logger().info(f'Ángulo detectado -> Raw: {angulo_promedio:.1f}° | EMA: {self.ema_angle:.1f}° (Offset: {self.OFFSET_ANGULO})')
                
        except Exception as e:
            self.get_logger().error(f'Error en el procesamiento de visión: {e}')

    def destroy_node(self):
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
            self.get_logger().info('Cámara ZED liberada correctamente.')
        
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