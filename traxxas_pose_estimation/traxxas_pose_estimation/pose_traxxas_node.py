import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Vector3Stamped
from sensor_msgs.msg import Imu


class PoseTraxxas(Node):
    """
    Pose estimator for Traxxas using:
      - linear velocity from /wheel/twist
      - yaw from /imu/data

    Publishes:
      /pose_traxxas   (geometry_msgs/Vector3Stamped)
        x [m], y [m], theta [rad]
    """

    def __init__(self):
        super().__init__('pose_traxxas')

        # ---------- Parameters ----------
        self.declare_parameter('twist_topic', '/wheel/twist')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('pose_topic', '/pose_traxxas')
        self.declare_parameter('frame_id', 'traxxas_pose')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('velocity_deadband', 0.01)
        self.declare_parameter('calibrate_yaw_on_start', True)
        self.declare_parameter('normalize_to_pi', True)

        twist_topic = self.get_parameter('twist_topic').value
        imu_topic = self.get_parameter('imu_topic').value
        pose_topic = self.get_parameter('pose_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        publish_rate = float(self.get_parameter('publish_rate').value)

        self.velocity_deadband = float(self.get_parameter('velocity_deadband').value)
        self.calibrate_yaw_on_start = bool(self.get_parameter('calibrate_yaw_on_start').value)
        self.normalize_to_pi = bool(self.get_parameter('normalize_to_pi').value)

        # ---------- State ----------
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.velocity = 0.0
        self.theta_imu = 0.0

        self.last_time = None

        self.theta_offset = None
        self.calibrated = False
        self.has_imu = False
        self.has_twist = False

        # ---------- Subscribers ----------
        self.twist_sub = self.create_subscription(
            TwistStamped,
            twist_topic,
            self.twist_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            imu_topic,
            self.imu_callback,
            10
        )

        # ---------- Publisher ----------
        self.pose_pub = self.create_publisher(
            Vector3Stamped,
            pose_topic,
            10
        )

        # ---------- Timer ----------
        period = 1.0 / publish_rate
        self.timer = self.create_timer(period, self.update_pose)

        self.get_logger().info('==========================================')
        self.get_logger().info(' POSE_TRAXXAS NODE STARTED')
        self.get_logger().info(f' Subscribed twist: {twist_topic}')
        self.get_logger().info(f' Subscribed imu:   {imu_topic}')
        self.get_logger().info(f' Publishing pose:  {pose_topic}')
        self.get_logger().info('==========================================')

    @staticmethod
    def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
        """
        Extract yaw from quaternion.
        Returns yaw in radians.
        """
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def normalize_angle_pi(angle: float) -> float:
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    @staticmethod
    def normalize_angle_2pi(angle: float) -> float:
        """Normalize angle to [0, 2pi)."""
        while angle >= 2.0 * math.pi:
            angle -= 2.0 * math.pi
        while angle < 0.0:
            angle += 2.0 * math.pi
        return angle

    def twist_callback(self, msg: TwistStamped):
        self.velocity = msg.twist.linear.x
        self.has_twist = True

    def imu_callback(self, msg: Imu):
        q = msg.orientation

        raw_theta = self.quaternion_to_yaw(
            q.x, q.y, q.z, q.w
        )

        if self.calibrate_yaw_on_start and not self.calibrated:
            self.theta_offset = raw_theta
            self.calibrated = True
            self.get_logger().info(
                f'IMU yaw calibrated. Offset = {self.theta_offset:.4f} rad '
                f'({math.degrees(self.theta_offset):.2f} deg)'
            )
        elif not self.calibrate_yaw_on_start and not self.calibrated:
            self.theta_offset = 0.0
            self.calibrated = True

        self.theta_imu = raw_theta - self.theta_offset

        if self.normalize_to_pi:
            self.theta_imu = self.normalize_angle_pi(self.theta_imu)
        else:
            self.theta_imu = self.normalize_angle_2pi(self.theta_imu)

        self.has_imu = True

    def update_pose(self):
        current_time = self.get_clock().now()

        if self.last_time is None:
            self.last_time = current_time
            return

        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if dt <= 0.0 or dt > 1.0:
            return

        if not self.has_imu or not self.has_twist:
            return

        # Use IMU heading directly
        self.theta = self.theta_imu

        # Small deadband to reduce drift when nearly stopped
        v = 0.0 if abs(self.velocity) < self.velocity_deadband else self.velocity

        # Dead-reckoning integration
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt

        # Publish pose
        pose_msg = Vector3Stamped()
        pose_msg.header.stamp = current_time.to_msg()
        pose_msg.header.frame_id = self.frame_id
        pose_msg.vector.x = self.x
        pose_msg.vector.y = self.y
        pose_msg.vector.z = self.theta

        self.pose_pub.publish(pose_msg)

        # Useful terminal print
        theta_deg = math.degrees(self.theta)
        print(
            f"\rPose -> X={self.x:.3f} m | Y={self.y:.3f} m | "
            f"Theta={self.theta:.3f} rad ({theta_deg:.1f} deg) | "
            f"V={v:.3f} m/s",
            end=''
        )


def main(args=None):
    rclpy.init(args=args)
    node = PoseTraxxas()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n==========================================')
        print(' POSE_TRAXXAS NODE STOPPED')
        print('==========================================')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()