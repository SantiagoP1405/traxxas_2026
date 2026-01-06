import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
from builtin_interfaces.msg import Time
import time

class Imu_node(Node):
    def __init__(self):
        super().__init__('imu_node')
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
            time.sleep(2)
            self.get_logger().info("Serial abierto en /dev/ttyUSB0")
        except Exception as e:
            self.get_logger().error(f"No se pudo abrir /dev/ttyUSB0: {e}")
            exit(1)
        self.pub_=self.create_publisher(Imu, 'imu/data', 10)
        self.timer_=self.create_timer(0.01, self.timer_callback)


    def timer_callback(self):
        try:
            raw = self.ser.readline().decode('utf-8').strip()

            if raw == "":
                return

            self.get_logger().info(f"RAW: {raw}")
            parts = raw.split(",")

            if len(parts) < 13:
                self.get_logger().warn("Línea incompleta, saltando...")
                return
            
            #yaw=float(parts[0])
            #pitch=float(parts[1])
            #roll=float(parts[2])
            qx=float(parts[3])
            qy=float(parts[4])
            qz=float(parts[5])
            qw=float(parts[6])
            gx=float(parts[7])
            gy=float(parts[8])
            gz=float(parts[9])
            ax=float(parts[10])
            ay=float(parts[11])
            az=float(parts[12])
            
            msg=Imu()
            msg.orientation.x=qx
            msg.orientation.y=qy
            msg.orientation.z=qz
            msg.orientation.w=qw

            msg.angular_velocity.x=gx
            msg.angular_velocity.y=gy
            msg.angular_velocity.z=gz

            msg.linear_acceleration.x=ax
            msg.linear_acceleration.y=ay
            msg.linear_acceleration.z=az

            msg.orientation_covariance[0] = -1.0



            

            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "imu_link"

            self.pub_.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error leyendo serial: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = Imu_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()