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


def warp(img, src, dst):
    M = cv2.getPerspectiveTransform(src.astype(np.float32), dst.astype(np.float32))
    return cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))


def sliding_window(binary_warped):
    histogram = np.sum(binary_warped[binary_warped.shape[0]//2:,:], axis=0)
    midpoint = np.int32(histogram.shape[0]/2)
    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint
    
    nwindows = 12
    window_height = np.int32(binary_warped.shape[0]/nwindows)
    nonzero = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    leftx_current = leftx_base
    rightx_current = rightx_base
    margin = 80
    minpix = 50
    left_lane_inds = []
    right_lane_inds = []
    
    out_img = np.dstack((binary_warped, binary_warped, binary_warped))*255
    
    for window in range(nwindows):
        win_y_low = binary_warped.shape[0] - (window+1)*window_height
        win_y_high = binary_warped.shape[0] - window*window_height
        win_xleft_low = leftx_current - margin
        win_xleft_high = leftx_current + margin
        win_xright_low = rightx_current - margin
        win_xright_high = rightx_current + margin
        
        cv2.rectangle(out_img,(win_xleft_low,win_y_low),(win_xleft_high,win_y_high), (0,255,0), 2) 
        cv2.rectangle(out_img,(win_xright_low,win_y_low),(win_xright_high,win_y_high), (0,255,0), 2)
        
        good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
                          (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
        good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
                           (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]
        
        left_lane_inds.append(good_left_inds)
        right_lane_inds.append(good_right_inds)
        
        if len(good_left_inds) > minpix:
            leftx_current = np.int32(np.mean(nonzerox[good_left_inds]))
        if len(good_right_inds) > minpix:
            rightx_current = np.int32(np.mean(nonzerox[good_right_inds]))
    
    if len(left_lane_inds) == 0 or len(right_lane_inds) == 0:
        return out_img, None, None, None

    left_lane_inds = np.concatenate(left_lane_inds)
    right_lane_inds = np.concatenate(right_lane_inds)
    
    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds]
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds]  
    
    min_pixels = 10
    if len(leftx) < min_pixels or len(rightx) < min_pixels:
        return out_img, None, None, None

    left_fit = np.polyfit(lefty, leftx, 2)
    right_fit = np.polyfit(righty, rightx, 2)
    
    ploty = np.linspace(0, binary_warped.shape[0]-1, binary_warped.shape[0])
    left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
    right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
    
    out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
    out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]
    
    for i in range(len(ploty)):
        cv2.circle(out_img, (int(left_fitx[i]), int(ploty[i])), 3, (255, 255, 0), -1)
        cv2.circle(out_img, (int(right_fitx[i]), int(ploty[i])), 3, (255, 255, 0), -1)
    
    return out_img, left_fit, right_fit, ploty


def measure_curvature(ploty, left_fit, right_fit):
    ym_per_pix = 0.60/720
    xm_per_pix = 0.40/1280
    
    y_eval = np.max(ploty)
    
    left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
    right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
    
    left_fit_cr = np.polyfit(ploty*ym_per_pix, left_fitx*xm_per_pix, 2)
    right_fit_cr = np.polyfit(ploty*ym_per_pix, right_fitx*xm_per_pix, 2)
    
    left_curverad = ((1 + (2*left_fit_cr[0]*y_eval*ym_per_pix + left_fit_cr[1])**2)**1.5) / np.absolute(2*left_fit_cr[0])
    right_curverad = ((1 + (2*right_fit_cr[0]*y_eval*ym_per_pix + right_fit_cr[1])**2)**1.5) / np.absolute(2*right_fit_cr[0])
    
    return left_curverad, right_curverad

def steering_from_curvature(left_curvature, right_curvature):
    L=0.28
    if right_curvature < left_curvature:
        R=right_curvature
    elif left_curvature < right_curvature:
        R=left_curvature
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
    
    if right_curv < left_curv:
        delta_deg = -abs(delta_deg)
    elif left_curv < right_curv:
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
        self.zed = iniciar_zed(ruta_svo)
        #self.zed = iniciar_zed()

        if self.zed is None:
            self.get_logger().error("No se pudo inicializar la ZED con el SVO")
            raise RuntimeError("Error inicializando ZED")

        self.zed_image = sl.Mat()
        self.runtime_params = sl.RuntimeParameters()

        cam_info = self.zed.get_camera_information()

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        grab_state = self.zed.grab(self.runtime_params)
        if grab_state == sl.ERROR_CODE.END_OF_SVOFILE_REACHED:
            self.get_logger().warn("Fin del SVO alcanzado, reiniciando al inicio")
            self.zed.set_svo_position(0)
            return
        elif grab_state != sl.ERROR_CODE.SUCCESS:
            self.get_logger().warn(f"Error al leer frame: {grab_state}")
            return

        self.zed.retrieve_image(self.zed_image, sl.VIEW.LEFT)
        img_bgr = self.zed_image.get_data()[:, :, :3]

        h, w = img_bgr.shape[:2]

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
        # 5) Definir ROI SOLO PARA CANNY (no para la imagen a color)
        vertices = np.array([[
            (int(0.24 * w), int(0.583 * h)),
            (int(0.80 * w), int(0.583 * h)),
            (int(0.86 * w), int(0.833 * h)),
            (int(0.15 * w), int(0.833 * h))
        ]], dtype=np.int32)

        img_roi_mask = np.zeros_like(img_canny, dtype=np.uint8)
        cv2.fillPoly(img_roi_mask, vertices, 255)

        # 6) Mantener solo bordes dentro del trapecio
        img_canny_roi = cv2.bitwise_and(img_canny, img_canny, mask=img_roi_mask)

        # 7) Warp usando el Canny ya enmascarado
        dst1 = np.array([
            [int(0.35 * w), 0],
            [int(0.65 * w), 0],
            [int(0.65 * w), h],
            [int(0.35 * w), h]
        ], dtype=np.int32)

        warped1 = warp(img_canny_roi, vertices[0], dst1)
        # 8) Sliding window
        sliding_img, left_fit, right_fit, ploty = sliding_window(warped1)

        img_roi_mask = np.zeros_like(img_canny, dtype=np.uint8)
        cv2.fillPoly(img_roi_mask, vertices, 255)
        
        if left_fit is None or right_fit is None:
            pwm = 2642
            left_curve = None
            right_curve = None
            self.get_logger().warn("No se detectaron carriles → PWM centro")
        else:
            left_curve, right_curve = measure_curvature(ploty, left_fit, right_fit)
            delta_deg = steering_from_curvature(left_curve, right_curve)
            pwm = angle_to_pwm(delta_deg, left_curve, right_curve)
            self.get_logger().info("Ángulo dirección: {:.2f}°  PWM: {}".format(delta_deg, pwm))

        if left_fit is None or right_fit is None:
            cv2.putText(sliding_img, "Carril no detectado", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        else:
            cv2.putText(sliding_img,
                        f"Left: {int(left_curve*100)}cm Right: {int(right_curve*100)}cm",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (255, 255, 255), 2)

        pwm_msg = String()
        pwm_msg.data = str(pwm)
        self.direction_pwm_pub.publish(pwm_msg)

        throttle_pwm = 2700
        throttle_msg = String()
        throttle_msg.data = str(throttle_pwm)
        self.throttle_pwm_pub.publish(throttle_msg)

        cv2.imshow("Original", img_bgr)
        cv2.imshow("Black Mask", mask_black)
        cv2.imshow("Masked Gray", masked_gray)
        cv2.imshow("Canny Filtered", img_canny)
        cv2.imshow("Warped", warped1)
        cv2.imshow("roi", img_roi_mask)
        cv2.imshow("Sliding Window", sliding_img)
        cv2.waitKey(1)


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
