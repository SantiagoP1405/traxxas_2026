#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu

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
MODEL_PATH = "/home/traxxas/Workspaces/traxxas_pruebas/best.engine"
RUTA_SVO   = ""
USE_SVO    = False
CONF       = 0.35

# Tracking / sensores
USE_POSITIONAL_TRACKING = True
USE_ZED_IMU             = True
USE_IMAGE_SYNC_IMU      = True   # True = IMU sincronizada al frame, False = IMU más reciente
ENABLE_AREA_MEMORY      = True

# Visualización
MOSTRAR_VENTANAS_DEFAULT = True


def quaternion_to_yaw_deg(qx, qy, qz, qw):
    """
    Convierte quaternion -> yaw en grados
    """
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return math.degrees(yaw)


def iniciar_zed(ruta_svo=None):
    zed = sl.Camera()
    init_params = sl.InitParameters()

    # Recomendado por ZED para tracking
    init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
    init_params.coordinate_units = sl.UNIT.METER
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FORWARD

    if ruta_svo:
        print(f"Modo: SVO -> {ruta_svo}")
        init_params.set_from_svo_file(ruta_svo)
        init_params.svo_real_time_mode = False
    else:
        print("Modo: Cámara ZED 2i en vivo")
        init_params.camera_resolution = sl.RESOLUTION.HD720
        init_params.camera_fps = 30

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


def detectar_angulos_en_mascara(binary_mask, vertices):
    """
    Toma la máscara binaria limpia de YOLO, aplica ROI,
    y usa HoughLinesP para sacar ángulos.
    """
    roi_mask = np.zeros_like(binary_mask)
    cv2.fillPoly(roi_mask, vertices, 255)
    roi = cv2.bitwise_and(binary_mask, binary_mask, mask=roi_mask)

    edges = cv2.Canny(roi, 50, 150)

    lineas = cv2.HoughLinesP(
        edges,
        rho=1,
        theta=np.pi / 180,
        threshold=40,
        minLineLength=30,
        maxLineGap=100
    )

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

    return angulos, lineas_salida, roi


class LaneDetector(Node):
    def __init__(self):
        super().__init__('lane_detector_camera')

        # ─────────────────────────────────────
        # SWITCHES / CALIBRACIÓN
        # ─────────────────────────────────────
        self.MOSTRAR_VENTANAS = MOSTRAR_VENTANAS_DEFAULT
        self.OFFSET_ANGULO = 0.0

        # EMA
        self.ema_angle = None
        self.alpha = 0.7

        # ─────────────────────────────────────
        # PUBLICADORES EXISTENTES
        # ─────────────────────────────────────
        self.raw_pub = self.create_publisher(Float32, '/qcar/lane_angle_raw', 10)
        self.ema_pub = self.create_publisher(Float32, '/qcar/lane_angle_ema', 10)

        # ─────────────────────────────────────
        # NUEVOS PUBLICADORES ZED
        # ─────────────────────────────────────
        self.pose_pub = self.create_publisher(PoseStamped, '/zed/pose', 10)
        self.imu_pub = self.create_publisher(Imu, '/zed/imu', 10)
        self.yaw_pub = self.create_publisher(Float32, '/zed/yaw_deg', 10)
        self.track_pub = self.create_publisher(String, '/zed/tracking_state', 10)

        # ─────────────────────────────────────
        # ZED
        # ─────────────────────────────────────
        self.zed = iniciar_zed(RUTA_SVO if USE_SVO else None)
        if self.zed is None:
            self.get_logger().error('No se pudo abrir la ZED.')
            raise RuntimeError("Error ZED")
        
        cam_info = self.zed.get_camera_information()
        self.tx_left_to_center = (
            cam_info.camera_configuration.calibration_parameters.T[0] * 0.5
        )
        self.get_logger().info(
            f'Transform left->center en X: {self.tx_left_to_center:.6f} m'
        )

        # Habilitar positional tracking
        self.pose_tracking_ok = False
        self.zed_pose = sl.Pose()
        self.sensors_data = sl.SensorsData()

        if USE_POSITIONAL_TRACKING:
            tracking_params = sl.PositionalTrackingParameters()
            tracking_params.enable_area_memory = ENABLE_AREA_MEMORY

            err = self.zed.enable_positional_tracking(tracking_params)
            if err != sl.ERROR_CODE.SUCCESS:
                self.get_logger().error(f'No se pudo activar positional tracking: {err}')
            else:
                self.pose_tracking_ok = True
                self.get_logger().info('Positional tracking ZED activado.')

        # Objetos auxiliares para extraer pose/orientación
        self.py_translation = sl.Translation()
        self.py_orientation = sl.Orientation()
        self.imu_orientation = sl.Orientation()

        # ─────────────────────────────────────
        # YOLO
        # ─────────────────────────────────────
        self.model = YOLO(MODEL_PATH, task="segment")
        self.get_logger().info("YOLO engine listo.")

        # ─────────────────────────────────────
        # HILO PRODUCTOR ZED
        # ─────────────────────────────────────
        self.frame_queue = queue.Queue(maxsize=1)
        self.stop_event = threading.Event()
        self.prod_thread = threading.Thread(target=self._zed_producer, daemon=True)
        self.prod_thread.start()

        # ─────────────────────────────────────
        # POOL PARA CV
        # ─────────────────────────────────────
        self.executor_cv = ThreadPoolExecutor(max_workers=2)

        # ─────────────────────────────────────
        # TIMER
        # ─────────────────────────────────────
        self.timer = self.create_timer(0.05, self.process_frame)

        self.get_logger().info(
            f'Visión YOLO + ZED pose/IMU iniciada. Ventanas: {"ON" if self.MOSTRAR_VENTANAS else "OFF"}'
        )

    # ──────────────────────────────────────────
    # PUBLICAR POSE / IMU
    # ──────────────────────────────────────────

    def _publish_zed_pose_and_imu(self):
        now_stamp = self.get_clock().now().to_msg()

        # ─────────────────────────────────────
        # 1) POSE / YAW / TRACKING STATE
        # ─────────────────────────────────────
        if self.pose_tracking_ok:
            try:
                tracking_state = self.zed.get_position(
                    self.zed_pose,
                    sl.REFERENCE_FRAME.WORLD
                )
            except AttributeError:
                tracking_state = self.zed.get_position(
                    self.zed_pose,
                    sl.REFERENCE_FRAME.FRAME_WORLD
                )

            track_msg = String()
            track_msg.data = tracking_state.name
            self.track_pub.publish(track_msg)

            if tracking_state == sl.POSITIONAL_TRACKING_STATE.OK:
                # Pose del ojo izquierdo
                t = self.zed_pose.get_translation(self.py_translation).get()
                q = self.zed_pose.get_orientation(self.py_orientation).get()

                qx, qy, qz, qw = q

                # Matriz de rotación a partir del quaternion
                R = np.array([
                    [1 - 2 * (qy * qy + qz * qz),     2 * (qx * qy - qz * qw),     2 * (qx * qz + qy * qw)],
                    [2 * (qx * qy + qz * qw),         1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
                    [2 * (qx * qz - qy * qw),         2 * (qy * qz + qx * qw),     1 - 2 * (qx * qx + qy * qy)]
                ], dtype=np.float64)

                # Offset local desde ojo izquierdo al centro de cámara
                offset_local = np.array(
                    [self.tx_left_to_center, 0.0, 0.0],
                    dtype=np.float64
                )

                # Llevar offset al frame global
                offset_world = R @ offset_local

                # Posición del centro de cámara en frame global
                t_center = np.array([
                    t[0] + offset_world[0],
                    t[1] + offset_world[1],
                    t[2] + offset_world[2]
                ], dtype=np.float64)

                pose_msg = PoseStamped()
                pose_msg.header.stamp = now_stamp
                pose_msg.header.frame_id = "zed_world"

                pose_msg.pose.position.x = float(t_center[0])
                pose_msg.pose.position.y = float(t_center[1])
                pose_msg.pose.position.z = float(t_center[2])

                # La orientación no cambia porque solo trasladamos
                pose_msg.pose.orientation.x = float(qx)
                pose_msg.pose.orientation.y = float(qy)
                pose_msg.pose.orientation.z = float(qz)
                pose_msg.pose.orientation.w = float(qw)

                self.pose_pub.publish(pose_msg)

                yaw_deg = quaternion_to_yaw_deg(qx, qy, qz, qw)
                yaw_msg = Float32()
                yaw_msg.data = float(yaw_deg)
                self.yaw_pub.publish(yaw_msg)

        # ─────────────────────────────────────
        # 2) IMU
        # ─────────────────────────────────────
        if USE_ZED_IMU:
            time_ref = sl.TIME_REFERENCE.IMAGE if USE_IMAGE_SYNC_IMU else sl.TIME_REFERENCE.CURRENT
            sensors_err = self.zed.get_sensors_data(self.sensors_data, time_ref)

            if sensors_err == sl.ERROR_CODE.SUCCESS:
                imu_data = self.sensors_data.get_imu_data()

                lin_acc = imu_data.get_linear_acceleration()
                ang_vel = imu_data.get_angular_velocity()
                imu_q = imu_data.get_pose().get_orientation(self.imu_orientation).get()

                imu_msg = Imu()
                imu_msg.header.stamp = now_stamp
                imu_msg.header.frame_id = "zed_imu"

                # Orientación fusionada del IMU
                imu_msg.orientation.x = float(imu_q[0])
                imu_msg.orientation.y = float(imu_q[1])
                imu_msg.orientation.z = float(imu_q[2])
                imu_msg.orientation.w = float(imu_q[3])

                # Aceleración lineal [m/s^2]
                imu_msg.linear_acceleration.x = float(lin_acc[0])
                imu_msg.linear_acceleration.y = float(lin_acc[1])
                imu_msg.linear_acceleration.z = float(lin_acc[2])

                # Velocidad angular [rad/s]
                imu_msg.angular_velocity.x = math.radians(float(ang_vel[0]))
                imu_msg.angular_velocity.y = math.radians(float(ang_vel[1]))
                imu_msg.angular_velocity.z = math.radians(float(ang_vel[2]))

                self.imu_pub.publish(imu_msg)

    # ──────────────────────────────────────────
    # HILO PRODUCTOR ZED
    # ──────────────────────────────────────────
    def _zed_producer(self):
        img_l = sl.Mat()
        img_r = sl.Mat()
        runtime = sl.RuntimeParameters()

        while not self.stop_event.is_set():
            grab = self.zed.grab(runtime)

            if grab == sl.ERROR_CODE.END_OF_SVOFILE_REACHED:
                self.get_logger().warn("Fin del SVO → reiniciando")
                self.zed.set_svo_position(0)
                continue

            if grab != sl.ERROR_CODE.SUCCESS:
                self.get_logger().warn(f"Error grab: {grab}")
                break

            # Recuperar imágenes
            self.zed.retrieve_image(img_l, sl.VIEW.LEFT)
            self.zed.retrieve_image(img_r, sl.VIEW.RIGHT)

            fl = img_l.get_data()[:, :, :3].copy()
            fr = img_r.get_data()[:, :, :3].copy()

            # Publicar pose/imu aquí mismo, sincronizado con cada frame
            try:
                self._publish_zed_pose_and_imu()
            except Exception as e:
                self.get_logger().warn(f'Error publicando pose/imu ZED: {e}')

            # Enviar frame al procesamiento CV
            if self.frame_queue.full():
                try:
                    self.frame_queue.get_nowait()
                except queue.Empty:
                    pass

            self.frame_queue.put((fl, fr))

        self.frame_queue.put(None)

    # ──────────────────────────────────────────
    # CALLBACK PRINCIPAL DE PROCESAMIENTO
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
            # YOLO batch: ambos ojos
            results = self.model(
                [img_left, img_right],
                conf=CONF,
                half=True,
                imgsz=640,
                classes=[0],
                retina_masks=True,
                verbose=False,
            )

            mask_left = yolo_mask_to_binary(results[0], h, w)
            mask_right = yolo_mask_to_binary(results[1], h, w)

            # ROI ojo izquierdo
            vertices_l = np.array([[ 
                (int(w * 0.19), int(h * 0.99)),
                (int(w * 0.99), int(h * 0.97)),
                (int(w * 0.70), int(h * 0.68)),
                (int(w * 0.34), int(h * 0.70))
            ]], dtype=np.int32)

            # ROI ojo derecho
            vertices_r = np.array([[ 
                (0, h),
                (int(w * 0.85), h),
                (int(w * 0.69), int(h * 0.77)),
                (int(w * 0.18), int(h * 0.8))
            ]], dtype=np.int32)

            # Procesar ambos ojos en paralelo
            fut_l = self.executor_cv.submit(
                detectar_angulos_en_mascara, mask_left, vertices_l
            )
            fut_r = self.executor_cv.submit(
                detectar_angulos_en_mascara, mask_right, vertices_r
            )

            angulos_l, lineas_l, roi_l = fut_l.result()
            angulos_r, lineas_r, roi_r = fut_r.result()

            angulos = angulos_l + angulos_r

            # Visualización
            if self.MOSTRAR_VENTANAS:
                debug_l = img_left.copy()
                debug_r = img_right.copy()

                for x1, y1, x2, y2 in lineas_l:
                    cv2.line(debug_l, (x1, y1), (x2, y2), (0, 255, 0), 3)

                for x1, y1, x2, y2 in lineas_r:
                    cv2.line(debug_r, (x1, y1), (x2, y2), (0, 255, 0), 3)

                cv2.polylines(debug_l, [vertices_l], True, (255, 0, 0), 2)
                cv2.polylines(debug_r, [vertices_r], True, (0, 255, 255), 2)

                cv2.imshow("Lineas - Ojo Izq", debug_l)
                cv2.imshow("Lineas - Ojo Der", debug_r)
                cv2.imshow("Mascara YOLO Izq", mask_left)
                cv2.imshow("Mascara YOLO Der", mask_right)
                cv2.imshow("ROI Izq", roi_l)
                cv2.imshow("ROI Der", roi_r)
                cv2.waitKey(1)

            # Publicar ángulo
            if angulos:
                angulo_promedio = sum(angulos) / len(angulos)
                angulo_promedio += self.OFFSET_ANGULO

                msg_raw = Float32()
                msg_raw.data = float(angulo_promedio)
                self.raw_pub.publish(msg_raw)

                if self.ema_angle is None:
                    self.ema_angle = angulo_promedio
                else:
                    self.ema_angle = (
                        self.alpha * angulo_promedio
                        + (1.0 - self.alpha) * self.ema_angle
                    )

                msg_ema = Float32()
                msg_ema.data = float(self.ema_angle)
                self.ema_pub.publish(msg_ema)

                self.get_logger().info(
                    f'Lane Raw: {angulo_promedio:.1f}° | EMA: {self.ema_angle:.1f}° | Offset: {self.OFFSET_ANGULO}'
                )

        except Exception as e:
            self.get_logger().error(f'Error en procesamiento: {e}')

    # ──────────────────────────────────────────
    # CIERRE
    # ──────────────────────────────────────────
    def destroy_node(self):
        self.stop_event.set()

        if hasattr(self, 'prod_thread'):
            self.prod_thread.join(timeout=2.0)

        if hasattr(self, 'executor_cv'):
            self.executor_cv.shutdown(wait=False)

        if hasattr(self, 'zed') and self.zed is not None:
            try:
                if self.pose_tracking_ok:
                    self.zed.disable_positional_tracking()
            except Exception:
                pass

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