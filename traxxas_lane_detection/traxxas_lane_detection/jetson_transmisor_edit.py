import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import pyzed.sl as sl
import cv2
import numpy as np


class JetsonStereoBridge(Node):
    def __init__(self):
        super().__init__('jetson_stereo_bridge')

        self.bridge = CvBridge()

        # Publicadores
        self.img_pub = self.create_publisher(
            CompressedImage,
            '/zed/stereo/compressed',
            qos_profile_sensor_data
        )

        self.depth_pub = self.create_publisher(
            Image,
            '/zed/depth_registered',
            qos_profile_sensor_data
        )

        self.get_logger().info("Iniciando ZED Camera...")

        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720
        init_params.camera_fps = 30
        init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
        init_params.coordinate_units = sl.UNIT.METER

        if self.zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error("Falla al abrir ZED. Revisa la conexión USB 3.0.")
            raise RuntimeError("No se pudo abrir la cámara ZED")

        self.runtime_params = sl.RuntimeParameters()

        self.left_sl = sl.Mat()
        self.right_sl = sl.Mat()
        self.depth_sl = sl.Mat()

        self.get_logger().info("ZED iniciada correctamente. Transmitiendo imagen y profundidad...")

        self.timer = self.create_timer(1.0 / 30.0, self.stream_callback)

    def stream_callback(self):
        if self.zed.grab(self.runtime_params) != sl.ERROR_CODE.SUCCESS:
            return

        # Recuperar imágenes izquierda y derecha
        self.zed.retrieve_image(self.left_sl, sl.VIEW.LEFT)
        self.zed.retrieve_image(self.right_sl, sl.VIEW.RIGHT)

        # Recuperar profundidad en metros
        self.zed.retrieve_measure(self.depth_sl, sl.MEASURE.DEPTH)

        left = self.left_sl.get_data()
        right = self.right_sl.get_data()
        depth = self.depth_sl.get_data()

        # ZED normalmente entrega BGRA -> convertir a BGR
        if left.shape[2] == 4:
            left_bgr = cv2.cvtColor(left, cv2.COLOR_BGRA2BGR)
        else:
            left_bgr = left[:, :, :3]

        if right.shape[2] == 4:
            right_bgr = cv2.cvtColor(right, cv2.COLOR_BGRA2BGR)
        else:
            right_bgr = right[:, :, :3]

        # Concatenar izquierda + derecha
        stereo_frame = np.hstack((left_bgr, right_bgr))

        stamp = self.get_clock().now().to_msg()

        # Publicar estéreo comprimido
        ok, enc = cv2.imencode(
            '.jpg',
            stereo_frame,
            [cv2.IMWRITE_JPEG_QUALITY, 70]
        )

        if ok:
            img_msg = CompressedImage()
            img_msg.header.stamp = stamp
            img_msg.header.frame_id = 'zed_stereo_frame'
            img_msg.format = "jpeg"
            img_msg.data = enc.tobytes()
            self.img_pub.publish(img_msg)

        # Publicar profundidad como Image 32FC1
        # depth ya viene en metros; asegurar float32
        depth_np = np.array(depth, dtype=np.float32)

        depth_msg = self.bridge.cv2_to_imgmsg(depth_np, encoding='32FC1')
        depth_msg.header.stamp = stamp
        depth_msg.header.frame_id = 'zed_left_camera_frame'
        self.depth_pub.publish(depth_msg)


def main(args=None):
    rclpy.init(args=args)
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