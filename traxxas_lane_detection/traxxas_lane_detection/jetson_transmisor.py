import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
import pyzed.sl as sl
import cv2
import numpy as np

class JetsonStereoBridge(Node):
    def __init__(self):
        super().__init__('jetson_stereo_bridge')
        
        # Único trabajo de este nodo: Publicar el video comprimido
        self.img_pub = self.create_publisher(CompressedImage, '/zed/stereo/compressed', qos_profile_sensor_data)

        self.get_logger().info("Iniciando ZED Camera...")

        # Configuración ZED (HD720, requerido para tu matemática de curvatura)
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720
        init_params.camera_fps = 30
        
        if self.zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error("Falla al abrir ZED. Revisa la conexión USB 3.0.")
            exit()

        self.get_logger().info("ZED iniciada correctamente. Transmitiendo telemetría visual...")
        
        # Timer para capturar a 30 FPS
        self.timer = self.create_timer(1/30, self.stream_callback)

    def stream_callback(self):
        left_sl = sl.Mat()
        right_sl = sl.Mat()
        
        if self.zed.grab() == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_image(left_sl, sl.VIEW.LEFT)
            self.zed.retrieve_image(right_sl, sl.VIEW.RIGHT)
            
            # Concatenar 1280x720 + 1280x720 = 2560x720
            stereo_frame = np.hstack((left_sl.get_data()[:,:,:3], right_sl.get_data()[:,:,:3]))
            
            # Comprimir a JPEG
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.format = "jpeg"
            msg.data = np.array(cv2.imencode('.jpg', stereo_frame, [cv2.IMWRITE_JPEG_QUALITY, 70])[1]).tobytes()
            
            # Enviar a la Laptop
            self.img_pub.publish(msg)

def main():
    rclpy.init()
    nodo = JetsonStereoBridge()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.zed.close()
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
