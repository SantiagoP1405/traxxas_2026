#!/usr/bin/env python3
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


def warp(img, src, dst):
    M = cv2.getPerspectiveTransform(src.astype(np.float32), dst.astype(np.float32))
    return cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))

def filters(img_bgr):
        # 1) Filtrado por color negro en TODA la imagen
        img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 80, 60])
        mask_black = cv2.inRange(img_hsv, lower_black, upper_black)

        kernel = np.ones((30,30), np.uint8)
        mask_black = cv2.morphologyEx(mask_black, cv2.MORPH_CLOSE, kernel, iterations = 15)
        #mask_black = cv2.morphologyEx(mask_black, cv2.MORPH_OPEN, kernel, iterations = 1)
        
        # 2) Gris + blur en TODA la imagen
        img_grey = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
        img_blur = cv2.GaussianBlur(img_grey, (3, 3), 0, 0)

        # 3) Aplicar máscara negra sobre el gris
        masked_gray = cv2.bitwise_and(img_blur, img_blur, mask=mask_black)

        # 4) Canny sobre la imagen ya filtrada por color
        img_canny = cv2.Canny(masked_gray, 40, 120)
        
        return img_canny


def sliding_window(binary_warped):
    histogram = np.sum(binary_warped, axis=0)
    base = np.argmax(histogram)
    
    nwindows = 12
    window_height = np.int32(binary_warped.shape[0]/nwindows)
    nonzero = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    current_x= base
    margin = 80
    minpix = 50
    lane_inds = []
    out_img = np.dstack((binary_warped, binary_warped, binary_warped))*255
    
    for window in range(nwindows):
        win_y_low = binary_warped.shape[0] - (window+1)*window_height
        win_y_high = binary_warped.shape[0] - window*window_height
        win_low = current_x - margin
        win_high = current_x + margin

        cv2.rectangle(out_img,(win_low,win_y_low),(win_high,win_y_high), (0,255,0), 2) 
        
        good_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
                          (nonzerox >= win_low) & (nonzerox < win_high)).nonzero()[0]
        
        lane_inds.append(good_inds)

        
        if len(good_inds) > minpix:
            current_x = np.int32(np.mean(nonzerox[good_inds]))

    
    lane_inds = np.concatenate(lane_inds)
    
    x = nonzerox[lane_inds]
    y = nonzeroy[lane_inds]
    min_pixels = 10
    if len(x) < min_pixels:
        return out_img, None, None

 
    fit = np.polyfit(y, x, 2)
    
    ploty = np.linspace(0, binary_warped.shape[0]-1, binary_warped.shape[0])
    fitx = fit[0]*ploty**2 + fit[1]*ploty + fit[2]
    
    out_img[nonzeroy[lane_inds], nonzerox[lane_inds]] = [255, 0, 0]
    
    for i in range(len(ploty)):
        cv2.circle(out_img, (int(fitx[i]), int(ploty[i])), 3, (255, 255, 0), -1)
    
    return out_img, fit, ploty



def measure_curvature(ploty, fit):
    ym_per_pix = 0.60/720
    xm_per_pix = 0.40/1280
    
    y_eval = np.max(ploty)
    
    fitx = fit[0]*ploty**2 + fit[1]*ploty + fit[2]

    
    fit_cr = np.polyfit(ploty*ym_per_pix, fitx*xm_per_pix, 2)
    
    curverad = ((1 + (2*fit_cr[0]*y_eval*ym_per_pix + fit_cr[1])**2)**1.5) / np.absolute(2*fit_cr[0])

    
    return curverad

def steering_from_curvature(left_curvature, right_curvature):
    L=0.28

    if right_curvature is not None and left_curvature is not None and right_curvature < left_curvature:
        R=right_curvature
    elif right_curvature is not None and left_curvature is not None and left_curvature < right_curvature:
        R=left_curvature
    elif right_curvature is None and left_curvature is not None:
        R=left_curvature
    elif left_curvature is None and right_curvature is not None:
        R=right_curvature
    else:
        R=right_curvature
    delta_rad=math.atan(L/R)
    delta_deg=math.degrees(delta_rad)
    return delta_deg


def angle_to_pwm(delta_deg, left_curv, right_curv):
    pwm_min = 1669
    pwm_center = 2642
    pwm_max = 3276
    max_steer_deg = 30.0
    
    if right_curv is not None and left_curv is not None and right_curv < left_curv:
        delta_deg = -abs(delta_deg)
    elif right_curv is not None and left_curv is not None and left_curv < right_curv:
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
    zed = sl.Camera()
    init_params = sl.InitParameters()

    init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
    init_params.coordinate_units = sl.UNIT.METER

    if ruta_svo:
        print(f"Modo: Leyendo archivo grabado -> {ruta_svo}")
        init_params.set_from_svo_file(ruta_svo)
        init_params.svo_real_time_mode = False
    else:
        print("Modo: Cámara ZED 2 en vivo")
        init_params.camera_resolution = sl.RESOLUTION.HD720
        init_params.camera_fps = 30

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
        self.throttle_pwm_pub  = self.create_publisher(String, 'throttle_motor', qos_profile)

        ruta_svo = "/home/traxxas/Documents/ZED/vid_2.svo2"
        #self.zed = iniciar_zed(ruta_svo)
        self.zed = iniciar_zed()

        if self.zed is None:
            self.get_logger().error("No se pudo inicializar la ZED con el SVO")
            raise RuntimeError("Error inicializando ZED")

        self.zed_image_left = sl.Mat()
        self.zed_image_right = sl.Mat()
        self.runtime_params = sl.RuntimeParameters()

        cam_info = self.zed.get_camera_information()

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.left_curve = None
        self.right_curve = None

    def timer_callback(self):
        start_time = time.time()
        grab_state = self.zed.grab(self.runtime_params)
        if grab_state == sl.ERROR_CODE.END_OF_SVOFILE_REACHED:
            self.get_logger().warn("Fin del SVO alcanzado, reiniciando al inicio")
            self.zed.set_svo_position(0)
            return
        elif grab_state != sl.ERROR_CODE.SUCCESS:
            self.get_logger().warn(f"Error al leer frame: {grab_state}")
            return

        self.zed.retrieve_image(self.zed_image_left, sl.VIEW.LEFT)
        self.zed.retrieve_image(self.zed_image_right, sl.VIEW.RIGHT)

        img_bgr_left = self.zed_image_left.get_data()[:, :, :3]
        img_bgr_right = self.zed_image_right.get_data()[:, :, :3]

        img_canny_left = filters(img_bgr_left)
        img_canny_right = filters(img_bgr_right)

        h, w = img_canny_left.shape[:2]
        # 5) Definir ROI SOLO PARA CANNY 
        vertices_left = np.array([[
            (int(0.24 * w), int(0.514 * h)),
            (int(0.531 * w), int(0.514 * h)),
            (int(0.531 * w), int(0.83 * h)),
            (int(0.195 * w), int(0.83* h) )
        ]], dtype=np.int32)

        vertices_right = np.array([[
            (int(0.469 * w), int(0.583 * h)),
            (int(0.86 * w), int(0.583 * h)),
            (int(0.86 * w), int(0.83 * h)),
            (int(0.469 * w), int(0.83 * h))
        ]], dtype=np.int32)

        #Imagen izquierda
        img_roi_mask_left = np.zeros_like(img_canny_left, dtype=np.uint8)
        cv2.fillPoly(img_roi_mask_left, vertices_left, 255)
        #Imagen derecha
        img_roi_mask_right = np.zeros_like(img_canny_right, dtype=np.uint8)
        cv2.fillPoly(img_roi_mask_right, vertices_right, 255)

        # 6) imagen roi
        img_canny_roi_left = cv2.bitwise_and(img_canny_left, img_canny_left, mask=img_roi_mask_left)
        img_canny_roi_right = cv2.bitwise_and(img_canny_right, img_canny_right, mask=img_roi_mask_right)


        # 7) Warp usando el Canny ya enmascarado
        dst1 = np.array([
            [int(0.35 * w), 0],
            [int(0.65 * w), 0],
            [int(0.65 * w), h],
            [int(0.35 * w), h]
        ], dtype=np.int32)

        warped_left = warp(img_canny_roi_left, vertices_left[0], dst1)
        warped_right = warp(img_canny_roi_right, vertices_right[0], dst1)

        # 8) Sliding window
        sliding_img_left, left_fit, left_ploty = sliding_window(warped_left)
        sliding_img_right, right_fit, right_ploty = sliding_window(warped_right)

        # Calcular curvaturas individuales
        if left_fit is not None and left_ploty is not None:
            self.left_curve = measure_curvature(left_ploty, left_fit)
        else:
            self.left_curve = None
        
        if right_fit is not None and right_ploty is not None:
            self.right_curve = measure_curvature(right_ploty, right_fit)
        else:
            self.right_curve = None

        # Calcular PWM basado en detección de carriles
        if self.left_curve is None and self.right_curve is None:
            pwm = 2642
            self.get_logger().warn("No se detectaron carriles → PWM centro")
        else:
            delta_deg = steering_from_curvature(self.left_curve, self.right_curve)
            pwm = angle_to_pwm(delta_deg, self.left_curve, self.right_curve)
            self.get_logger().info("Ángulo dirección: {:.2f}°  PWM: {}".format(delta_deg, pwm))

        # Visualización para imagen izquierda
        if left_fit is None:
            cv2.putText(sliding_img_left, "Carril izquierdo no detectado", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        else:
            cv2.putText(sliding_img_left,
                        f"Left: {int(self.left_curve*100)}cm",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (255, 255, 255), 2)

        # Visualización para imagen derecha
        if right_fit is None:
            cv2.putText(sliding_img_right, "Carril derecho no detectado", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        else:
            cv2.putText(sliding_img_right,
                        f"Right: {int(self.right_curve*100)}cm",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (255, 255, 255), 2)

        pwm_msg = String()
        pwm_msg.data = str(pwm)
        self.direction_pwm_pub.publish(pwm_msg)

        throttle_pwm = 2700
        throttle_msg = String()
        throttle_msg.data = str(throttle_pwm)
        self.throttle_pwm_pub.publish(throttle_msg)

        end_time = time.time()
        
        
        #cv2.imshow("Original left", img_bgr_left)
        #cv2.imshow("Original right", img_bgr_right)
        #cv2.imshow("Sliding Window Left", sliding_img_left)
        #cv2.imshow("Sliding Window Right", sliding_img_right)  
        #cv2.imshow("Warped left", warped_left)
        #cv2.imshow("Warped right", warped_right)
        #cv2.imshow("canny", img_canny_left )
        #cv2.imshow("canny", img_canny_roi_left )
        #cv2.imshow("canny right", img_canny_roi_right )
        cv2.waitKey(1)
        elapsed_time = end_time - start_time
        self.get_logger().info(f"Tiempo de procesamiento: {elapsed_time*1000:.2f} ms") 
        


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
