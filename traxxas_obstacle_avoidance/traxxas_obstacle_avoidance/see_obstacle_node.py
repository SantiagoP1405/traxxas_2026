#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
from rclpy.qos import qos_profile_sensor_data


def check_proximity(point_cloud_np, threshold_m=0.70,
                    min_height_y=-0.2, max_height_y=0.0,
                    distance_offset_m=0.0):
    xyz = point_cloud_np[:, :, :3]
    valid_mask = np.isfinite(xyz).all(axis=2)
    height_mask = (xyz[:, :, 1] > min_height_y) & (xyz[:, :, 1] < max_height_y)
    combined_mask = valid_mask & height_mask
    if not combined_mask.any():
        return False, None
    valid_xyz = xyz[combined_mask]
    distances = np.linalg.norm(valid_xyz, axis=1)
    min_dist = np.percentile(distances, 5) + distance_offset_m
    return min_dist < threshold_m, min_dist


def check_center_proximity(point_cloud_np, threshold_m=0.70,
                           center_radius_frac=0.075,
                           min_height_y=-0.2, max_height_y=0.0,
                           distance_offset_m=0.0):
    h, w = point_cloud_np.shape[:2]
    cx, cy = w // 2, h // 2
    rx = int(w * center_radius_frac)
    ry = int(h * center_radius_frac)
    y0, y1 = max(0, cy - ry), min(h, cy + ry)
    x0, x1 = max(0, cx - rx), min(w, cx + rx)
    center_patch = point_cloud_np[y0:y1, x0:x1, :3]
    valid_mask = np.isfinite(center_patch).all(axis=2)
    height_mask = (center_patch[:, :, 1] > min_height_y) & (center_patch[:, :, 1] < max_height_y)
    combined_mask = valid_mask & height_mask
    if not combined_mask.any():
        return False, None
    valid_xyz = center_patch[combined_mask]
    distances = np.abs(valid_xyz[:, 2])
    min_dist = distances.min() + distance_offset_m
    return min_dist < threshold_m, min_dist


class SeeObstacleNode(Node):

    def __init__(self):
        super().__init__('see_obstacle_node')

        self.bridge = CvBridge()

        # ── Parámetros (igual que el nodo original) ───────────────────
        self.declare_parameter('proximity_threshold', 0.70)
        self.declare_parameter('min_height_y',        -0.2)
        self.declare_parameter('max_height_y',         0.0)
        self.declare_parameter('distance_offset',      0.00)
        self.declare_parameter('center_radius_frac',   0.075)

        self.threshold   = self.get_parameter('proximity_threshold').value
        self.min_height  = self.get_parameter('min_height_y').value
        self.max_height  = self.get_parameter('max_height_y').value
        self.offset      = self.get_parameter('distance_offset').value
        self.center_frac = self.get_parameter('center_radius_frac').value

        # ── Estado extra para integración ─────────────────────────────
        self.obstacle_active = False
        self.trigger_sent = False

        # ── Publishers originales ─────────────────────────────────────
        self.pub_global_dist  = self.create_publisher(Float32, 'zed/global/distance',  10)
        self.pub_global_alert = self.create_publisher(Bool,    'zed/global/too_close', 10)
        self.pub_center_dist  = self.create_publisher(Float32, 'zed/center/distance',  10)
        self.pub_center_alert = self.create_publisher(Bool,    'zed/center/too_close', 10)
        self.pub_status       = self.create_publisher(String,  'zed/status',           10)

        # ── Nuevo publisher para disparar maniobra ────────────────────
        self.pub_obstacle = self.create_publisher(Bool, '/obstacle', 10)

        # ── Subscribers ───────────────────────────────────────────────
        self.sub_xyzrgba = self.create_subscription(
            Image,
            '/zed/xyzrgba_image',
            self.xyz_callback,
            qos_profile_sensor_data
        )

        self.sub_obstacle_active = self.create_subscription(
            Bool,
            '/traxxas/obstacle_active',
            self.obstacle_active_callback,
            10
        )

        self.get_logger().info(
            f"SeeObstacleNode listo | Threshold: {self.threshold} m | "
            f"Altura Y: [{self.min_height}, {self.max_height}] m | "
            f"Offset: {self.offset*100:+.0f} cm"
        )

    # ──────────────────────────────────────────────────────────────────
    def obstacle_active_callback(self, msg: Bool):
        prev = self.obstacle_active
        self.obstacle_active = bool(msg.data)

        if self.obstacle_active != prev:
            if self.obstacle_active:
                self.get_logger().warn("Obstacle avoidance ACTIVO -> bloqueo nuevos disparos")
                self.trigger_sent = True
            else:
                self.get_logger().warn("Obstacle avoidance INACTIVO -> detector rearmado")
                self.trigger_sent = False

    # ──────────────────────────────────────────────────────────────────
    def xyz_callback(self, msg: Image):
        try:
            pc_np = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC4')
        except Exception as e:
            self.get_logger().error(f"Error convirtiendo /zed/xyzrgba_image: {e}")
            return

        # ── Detección global ──────────────────────────────────────────
        too_close, min_dist = check_proximity(
            pc_np, self.threshold, self.min_height, self.max_height, self.offset)

        # ── Detección central ─────────────────────────────────────────
        center_close, center_dist = check_center_proximity(
            pc_np, self.threshold, self.center_frac,
            self.min_height, self.max_height, self.offset)

        # ── Publicar y loggear global ─────────────────────────────────
        if min_dist is not None:
            self.pub_global_dist.publish(Float32(data=float(min_dist)))
            self.pub_global_alert.publish(Bool(data=bool(too_close)))
            if too_close:
                self.get_logger().warn(f" [GLOBAL] OBJETO A {min_dist*100:.1f} cm")
            else:
                self.get_logger().info(f" [GLOBAL] Libre | {min_dist*100:.1f} cm")
        else:
            self.get_logger().warn(" [GLOBAL] Sin puntos válidos")

        # ── Publicar y loggear central ────────────────────────────────
        if center_dist is not None:
            self.pub_center_dist.publish(Float32(data=float(center_dist)))
            self.pub_center_alert.publish(Bool(data=bool(center_close)))
            if center_close:
                self.get_logger().warn(f" [CENTRO] OBJETO A {center_dist*100:.1f} cm")
            else:
                self.get_logger().info(f" [CENTRO] Libre | {center_dist*100:.1f} cm")
        else:
            self.get_logger().warn(" [CENTRO] Sin puntos válidos")

        # ── Publicar status resumido ──────────────────────────────────
        gd = f"{min_dist*100:.1f}"    if min_dist    is not None else "N/A"
        cd = f"{center_dist*100:.1f}" if center_dist is not None else "N/A"
        self.pub_status.publish(String(data=f"global={gd}cm center={cd}cm"))

        # ── Disparo de /obstacle con realimentación ───────────────────
        detected_for_trigger = center_close

        if self.obstacle_active:
            self.trigger_sent = True
            return

        if detected_for_trigger and not self.trigger_sent:
            self.pub_obstacle.publish(Bool(data=True))
            self.trigger_sent = True
            self.get_logger().warn("Publicando /obstacle = True")

        elif not detected_for_trigger:
            self.trigger_sent = False


def main(args=None):
    rclpy.init(args=args)
    node = SeeObstacleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()