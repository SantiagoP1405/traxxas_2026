import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import tf_transformations
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class AckermannOdomNode(Node):
    def __init__(self):
        super().__init__('ackermann_odom_node')

        self.declare_parameter('wheelbase', 0.335)      # L (m)
        self.declare_parameter('gear_ratio', 15.3)      # GR
        self.declare_parameter('wheel_radius', 0.055)   # r (m)

        self.declare_parameter('frame_odom', 'odom')
        self.declare_parameter('frame_base', 'base_link')

        self.L = float(self.get_parameter('wheelbase').value)
        self.GR = float(self.get_parameter('gear_ratio').value)
        self.r = float(self.get_parameter('wheel_radius').value)

        self.frame_odom = self.get_parameter('frame_odom').value
        self.frame_base = self.get_parameter('frame_base').value

        # State
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_t = None

        self.v = 0.0
        self.delta = 0.0  # optional, for debug/correction if you want

        # pubs/subs
        self.pub_odom = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Inputs:
        # - from serial_node you can publish TwistStamped with linear.x already v_mps
        self.sub_twist = self.create_subscription(TwistStamped, 'wheel/twist', self.cb_twist, 20)
        self.sub_imu = self.create_subscription(Imu, 'imu/data', self.cb_imu, 50)
        self.sub_delta = self.create_subscription(Float64, 'steering_angle', self.cb_delta, 10)

        self.timer = self.create_timer(0.02, self.update)  # 50 Hz

    def cb_twist(self, msg: TwistStamped):
        self.v = float(msg.twist.linear.x)

    def cb_delta(self, msg: Float64):
        self.delta = float(msg.data)

    def cb_imu(self, msg: Imu):
        # Extract yaw from quaternion
        q = msg.orientation
        quat = [q.x, q.y, q.z, q.w]
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(quat)
        self.yaw = yaw

    def update(self):
        now = self.get_clock().now()
        if self.last_t is None:
            self.last_t = now
            return

        dt = (now - self.last_t).nanoseconds * 1e-9
        self.last_t = now
        if dt <= 0.0:
            return

        # Integrate x,y using yaw from IMU and v from encoder
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt

        # Publish odometry
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.frame_odom
        odom.child_frame_id = self.frame_base

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        qx, qy, qz, qw = tf_transformations.quaternion_from_euler(0.0, 0.0, self.yaw)
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = self.v
        # optional: use bicycle model for yaw_rate estimate from steering for twist.angular.z
        odom.twist.twist.angular.z = (self.v / self.L) * math.tan(self.delta) if abs(self.L) > 1e-6 else 0.0

        self.pub_odom.publish(odom)

        # TF
        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        t.header.frame_id = self.frame_odom
        t.child_frame_id = self.frame_base
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = AckermannOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()