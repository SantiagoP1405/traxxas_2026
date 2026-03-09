import rclpy
from rclpy.node import Node
import serial
import math
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistStamped


class SerialSensorsNode(Node):

    def __init__(self):
        super().__init__('serial_sensors_node')

        # ---------------- Parameters ----------------
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('gear_ratio', 15.3)
        self.declare_parameter('wheel_radius', 0.055)
        self.declare_parameter('frame_id', 'base_link')

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        self.GR = float(self.get_parameter('gear_ratio').value)
        self.r = float(self.get_parameter('wheel_radius').value)
        self.frame_id = self.get_parameter('frame_id').value

        # Conversion factor omega_motor -> v_mps
        self.v_gain = self.r / self.GR

        # ---------------- Publishers ----------------
        self.pub_imu = self.create_publisher(Imu, 'imu/data', 20)
        self.pub_twist = self.create_publisher(TwistStamped, 'wheel/twist', 20)

        # ---------------- Serial ----------------
        try:
            self.ser = serial.Serial(port, baudrate=baud, timeout=0.1)
            self.get_logger().info(f"Serial opened: {port}")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial: {e}")
            raise

        # Timer to read serial
        self.timer = self.create_timer(0.01, self.read_serial)  # 100 Hz polling


    def read_serial(self):

        try:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                return

            if line.startswith("#"):
                return

            parts = line.split(',')
            if len(parts) != 12:
                return

            # ---------------- Parse CSV ----------------
            omega_motor = float(parts[1])

            qw = float(parts[2])
            qx = float(parts[3])
            qy = float(parts[4])
            qz = float(parts[5])

            gx = float(parts[6])
            gy = float(parts[7])
            gz = float(parts[8])

            ax = float(parts[9])
            ay = float(parts[10])
            az = float(parts[11])

            # ---------------- Compute velocity ----------------
            v_mps = omega_motor * self.v_gain

            now = self.get_clock().now().to_msg()

            # ---------------- Publish IMU ----------------
            imu_msg = Imu()
            imu_msg.header.stamp = now
            imu_msg.header.frame_id = self.frame_id

            imu_msg.orientation.w = qw
            imu_msg.orientation.x = qx
            imu_msg.orientation.y = qy
            imu_msg.orientation.z = qz

            imu_msg.angular_velocity.x = gx
            imu_msg.angular_velocity.y = gy
            imu_msg.angular_velocity.z = gz

            imu_msg.linear_acceleration.x = ax
            imu_msg.linear_acceleration.y = ay
            imu_msg.linear_acceleration.z = az

            self.pub_imu.publish(imu_msg)

            # ---------------- Publish Twist ----------------
            twist_msg = TwistStamped()
            twist_msg.header.stamp = now
            twist_msg.header.frame_id = self.frame_id
            twist_msg.twist.linear.x = v_mps
            twist_msg.twist.angular.z = gz  # opcional: yaw de IMU

            self.pub_twist.publish(twist_msg)

        except Exception as e:
            self.get_logger().warn(f"Serial parse error: {e}")


def main():
    rclpy.init()
    node = SerialSensorsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()