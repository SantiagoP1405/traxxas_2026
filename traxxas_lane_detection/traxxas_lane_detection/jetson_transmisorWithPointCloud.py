
# -*- coding: utf-8 -*-

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

        # ------------------------------------------------------------
        # Publicadores
        # ------------------------------------------------------------
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

        self.xyzrgba_pub = self.create_publisher(
            Image,
            '/zed/xyzrgba_image',
            qos_profile_sensor_data
        )

        self.get_logger().info('Iniciando ZED Camera...')

        # ------------------------------------------------------------
        # Inicialización ZED
        # ------------------------------------------------------------
        self.zed = sl.Camera()

        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720
        init_params.camera_fps = 30
        init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
        init_params.coordinate_units = sl.UNIT.METER
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP

        status = self.zed.open(init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error(f'Falla al abrir ZED: {repr(status)}')
            raise RuntimeError('No se pudo abrir la cámara ZED')

        self.runtime_params = sl.RuntimeParameters()

        # Mats persistentes para evitar realloc constante
        self.left_sl = sl.Mat()
        self.right_sl = sl.Mat()
        self.depth_sl = sl.Mat()
        self.xyzrgba_sl = sl.Mat()

        self.get_logger().info(
            'ZED iniciada correctamente. Publicando:\n'
            '  - /zed/stereo/compressed\n'
            '  - /zed/depth_registered\n'
            '  - /zed/xyzrgba_image'
        )

        # 30 Hz
        self.timer = self.create_timer(1.0 / 30.0, self.stream_callback)

    # ------------------------------------------------------------------
    # Callback principal
    # ------------------------------------------------------------------
    def stream_callback(self):
        if self.zed.grab(self.runtime_params) != sl.ERROR_CODE.SUCCESS:
            return

        # ------------------------------------------------------------
        # Recuperar datos de la ZED
        # ------------------------------------------------------------
        self.zed.retrieve_image(self.left_sl, sl.VIEW.LEFT)
        self.zed.retrieve_image(self.right_sl, sl.VIEW.RIGHT)

        # Profundidad por píxel (float32, metros)
        self.zed.retrieve_measure(self.depth_sl, sl.MEASURE.DEPTH)

        # XYZRGBA por píxel (float32, 4 canales)
        self.zed.retrieve_measure(self.xyzrgba_sl, sl.MEASURE.XYZRGBA)

        left = self.left_sl.get_data()
        right = self.right_sl.get_data()
        depth = self.depth_sl.get_data()
        xyzrgba = self.xyzrgba_sl.get_data()

        stamp = self.get_clock().now().to_msg()

        # ------------------------------------------------------------
        # Imagen estéreo comprimida
        # ------------------------------------------------------------
        # ZED normalmente entrega BGRA
        if left.ndim == 3 and left.shape[2] == 4:
            left_bgr = cv2.cvtColor(left, cv2.COLOR_BGRA2BGR)
        else:
            left_bgr = left[:, :, :3]

        if right.ndim == 3 and right.shape[2] == 4:
            right_bgr = cv2.cvtColor(right, cv2.COLOR_BGRA2BGR)
        else:
            right_bgr = right[:, :, :3]

        stereo_frame = np.hstack((left_bgr, right_bgr))

        ok, enc = cv2.imencode(
            '.jpg',
            stereo_frame,
            [cv2.IMWRITE_JPEG_QUALITY, 70]
        )

        if ok:
            img_msg = CompressedImage()
            img_msg.header.stamp = stamp
            img_msg.header.frame_id = 'zed_stereo_frame'
            img_msg.format = 'jpeg'
            img_msg.data = enc.tobytes()
            self.img_pub.publish(img_msg)

        # ------------------------------------------------------------
        # Profundidad: /zed/depth_registered
        # ------------------------------------------------------------
        depth_np = np.array(depth, dtype=np.float32, copy=True)

        depth_msg = self.bridge.cv2_to_imgmsg(depth_np, encoding='32FC1')
        depth_msg.header.stamp = stamp
        depth_msg.header.frame_id = 'zed_left_camera_frame'
        self.depth_pub.publish(depth_msg)

        # ------------------------------------------------------------
        # XYZRGBA: /zed/xyzrgba_image
        # ------------------------------------------------------------
        # La ZED devuelve una matriz HxWx4 float32 con:
        #   canal 0 -> X
        #   canal 1 -> Y
        #   canal 2 -> Z
        #   canal 3 -> RGBA empaquetado en float
        #
        # Esto le permitirá a see_obstacle_node reutilizar su lógica vieja:
        #   point_cloud_np[:, :, :3]
        #
        xyzrgba_np = np.array(xyzrgba, dtype=np.float32, copy=True)

        xyzrgba_msg = self.bridge.cv2_to_imgmsg(xyzrgba_np, encoding='32FC4')
        xyzrgba_msg.header.stamp = stamp
        xyzrgba_msg.header.frame_id = 'zed_left_camera_frame'
        self.xyzrgba_pub.publish(xyzrgba_msg)

    # ------------------------------------------------------------------
    # Cierre limpio
    # ------------------------------------------------------------------
    def destroy_node(self):
        self.get_logger().info('Cerrando cámara ZED...')
        try:
            self.zed.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = JetsonStereoBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()