#!/usr/bin/env python3
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
import os
import math
import numpy as np
import cv2


# warp para TOP VIEW
def warp(img, src, dst):
    M = cv2.getPerspectiveTransform(src.astype(np.float32), dst.astype(np.float32))
    return cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))


def sliding_window(binary_warped):
    histogram = np.sum(binary_warped[binary_warped.shape[0]//2:,:], axis=0)
    midpoint = np.int32(histogram.shape[0]/2)
    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint
    
    nwindows = 9
    window_height = np.int32(binary_warped.shape[0]/nwindows)
    nonzero = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    leftx_current = leftx_base
    rightx_current = rightx_base
    margin = 100
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
    ym_per_pix = 0.69/480
    xm_per_pix = 0.25/640
    
    y_eval = np.max(ploty)
    
    left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
    right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
    
    left_fit_cr = np.polyfit(ploty*ym_per_pix, left_fitx*xm_per_pix, 2)
    right_fit_cr = np.polyfit(ploty*ym_per_pix, right_fitx*xm_per_pix, 2)
    
    left_curverad = ((1 + (2*left_fit_cr[0]*y_eval*ym_per_pix + left_fit_cr[1])**2)**1.5) / np.absolute(2*left_fit_cr[0])
    right_curverad = ((1 + (2*right_fit_cr[0]*y_eval*ym_per_pix + right_fit_cr[1])**2)**1.5) / np.absolute(2*right_fit_cr[0])
    
    return left_curverad, right_curverad

def fuse_curve_rads(radius_left, radius_right):
    k_left=1.0/radius_left  
    k_right=1.0/radius_right
    k_center = 0.5 * (k_left + k_right)
    r_center = 1.0/k_center
    return r_center

def steering_from_curvature(R):
    #separacion entre ejes
    L=0.28
    delta_rad=math.atan(L/R)
    delta_deg=math.degrees(delta_rad)
    return delta_deg

def angle_to_pwm(delta_deg, left_curv, right_curv):
    pwm_min = 1669
    pwm_center = 2642
    pwm_max = 3276
    max_steer_deg = 30.0
    
    # Determinar dirección basado en cuál curvatura es menor
    if right_curv < left_curv:
        # Radio derecho pequeño → girar a la IZQUIERDA (PWM menor)
        delta_deg = -abs(delta_deg)  # Asegurar que sea negativo
    elif left_curv < right_curv:
        # Radio izquierdo pequeño  → girar a la DERECHA (PWM mayor)
        delta_deg = abs(delta_deg)   # Asegurar que sea positivo
    
    # Limitar el ángulo
    delta_deg = max(-max_steer_deg, min(max_steer_deg, delta_deg))
    
    norm = delta_deg / max_steer_deg

    if norm >= 0:
        # Girar a la derecha
        pwm = pwm_center + norm * (pwm_max - pwm_center)
    else:
        # Girar a la izquierda
        pwm = pwm_center + norm * (pwm_center - pwm_min)
    
    return int(pwm)


class LaneDetectorNode(Node):
    def __init__(self):
        super().__init__('lane_detector_node')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.direction_pwm_pub = self.create_publisher(String, 'direction_servo', qos_profile)
        self.throttle_pwm_pub = self.create_publisher(String, 'throttle_motor', qos_profile)
        video_path = "/home/traxxas/Workspaces/traxxas_pruebas/video_derecha.mp4"
        self.cap = cv2.VideoCapture(video_path)
        #self.cap = cv2.VideoCapture(0)
        width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.get_logger().info(f"Resolución cámara: {width} x {height}")
        #self.cap = cv2.VideoCapture(video_path)

        # if not self.cap.isOpened():
        #     self.get_logger().error(f"No se pudo abrir el video: {video_path}")
        # else:
        #     self.get_logger().info(f"Video cargado: {video_path}")

        self.timer = self.create_timer(0.1, self.timer_callback)
        #metodo que apaga los servos al cerrar el nodo
        #self.add_on_shutdown(self.send_safe_stop)


    def timer_callback(self):
        ret, img_bgr = self.cap.read()
        if not ret:
            self.get_logger().warn("Video finished")
            return

        img_grey = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY) 
        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        img_blur = cv2.GaussianBlur(img_grey, (3, 3), 0, 0)
        img_canny = cv2.Canny(img_blur, 40, 120) 

        vertices = np.array([[(0,200), (155, 200), (163, 480), (0, 448)]], dtype=np.int32)
        vertices2 = np.array([[(238,196), (340, 196), (443, 224), (570, 297), (640,361), (640, 480),(480,480), (442,283), (335, 226)]], dtype=np.int32)  

        dst1 = np.array([[75, 0], [250, 0], [250, 448], [75, 448]], dtype=np.int32)

        img_roi = np.zeros_like(img_grey)   
        cv2.fillPoly(img_roi, vertices, 255)
        cv2.fillPoly(img_roi, vertices2, 255)
        img_mask = cv2.bitwise_and(img_canny, img_roi)
        warped1 = warp(img_mask, vertices[0], dst1)

        sliding_img, left_fit, right_fit, ploty = sliding_window(warped1)
        
        if left_fit is None or right_fit is None:
            pwm = 2642  # centro
            left_curve = None
            right_curve = None
            self.get_logger().warn("No se detectaron carriles → PWM centro")

        else:
            left_curve, right_curve = measure_curvature(ploty, left_fit, right_fit)
            lane_center = fuse_curve_rads(left_curve, right_curve)
            delta_deg = steering_from_curvature(lane_center)
            pwm = angle_to_pwm(delta_deg, left_curve, right_curve)
            self.get_logger().info("Angulo de dirección: {:.2f} grados".format(delta_deg) + "PWM: {}".format(pwm))       

        if left_fit is None or right_fit is None:
            cv2.putText(sliding_img, "Carril no detectado", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        else:
            cv2.putText(sliding_img, f"Left: {int(left_curve*100)}cm Right: {int(right_curve*100)}cm", 
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            

        # === PUBLICAR ===
        # left_msg = Float32()
        # right_msg = Float32()
        # left_msg.data = float(left_curve)
        # right_msg.data = float(right_curve)
        # self.left_pub.publish(left_msg)
        # self.right_pub.publish(right_msg)
        pwm_msg = String()
        pwm_msg.data = str(pwm)
        self.direction_pwm_pub.publish(pwm_msg)
        
        #Publicador para throttle fijo
        throttle_pwm = 2700  # Valor fijo para avanzar
        throttle_msg = String()
        throttle_msg.data = str(throttle_pwm)
        self.throttle_pwm_pub.publish(throttle_msg)

        # === VENTANAS ===
        cv2.imshow("Original", img_bgr)
        cv2.imshow("ROI Mask", img_roi)
        cv2.imshow("ROI Applied", img_mask)
        cv2.imshow("Warped", warped1)
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
