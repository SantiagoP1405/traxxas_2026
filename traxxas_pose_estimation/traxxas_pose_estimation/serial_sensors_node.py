#!/usr/bin/env python3
"""
=======================================================================
 Serial Sensors Node - Traxxas / ESP32

 Parsea el stream serial del ESP32:
   t, ds, yaw, pitch, roll, qw, qx, qy, qz, gx, gy, gz, ax, ay, az

 donde:
   t   = timestamp [ms] del ESP32
   ds  = distancia incremental [m] desde la última publicación

 Publica:
   /imu/data      (sensor_msgs/Imu)
   /imu/euler     (geometry_msgs/Vector3Stamped)
   /wheel/twist   (geometry_msgs/TwistStamped)   -> linear.x = v [m/s]
   /wheel/debug   (geometry_msgs/Vector3Stamped) -> x=v [m/s], y=ds [m], z=dt [s]
=======================================================================
"""

import math
import rclpy
from rclpy.node import Node
import serial

from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistStamped, Vector3Stamped


class SerialSensorsNode(Node):

    def __init__(self):
        super().__init__('serial_sensors_node')

        # ---------------- Parameters ----------------
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('max_dt', 0.5)        # dt máximo válido [s]
        self.declare_parameter('min_dt', 0.001)      # dt mínimo válido [s]
        self.declare_parameter('v_deadband', 0.005)  # deadband velocidad [m/s]
        self.declare_parameter('debug_log', True)

        port            = self.get_parameter('port').value
        baud            = int(self.get_parameter('baud').value)
        self.frame_id   = self.get_parameter('frame_id').value
        self.max_dt     = float(self.get_parameter('max_dt').value)
        self.min_dt     = float(self.get_parameter('min_dt').value)
        self.v_deadband = float(self.get_parameter('v_deadband').value)
        self.debug_log  = bool(self.get_parameter('debug_log').value)

        # ---------------- Estado interno ----------------
        self.last_t_esp_ms = None   # último timestamp del ESP32 [ms]

        # ---------------- Publishers ----------------
        self.pub_imu   = self.create_publisher(Imu,            '/imu/data',    20)
        self.pub_twist = self.create_publisher(TwistStamped,   '/wheel/twist', 20)
        self.pub_euler = self.create_publisher(Vector3Stamped, '/imu/euler',   20)
        self.pub_debug = self.create_publisher(Vector3Stamped, '/wheel/debug', 20)

        # ---------------- Serial ----------------
        try:
            self.ser = serial.Serial(port, baudrate=baud, timeout=0.1)
            self.get_logger().info(f'Serial abierto: {port} @ {baud}')
        except Exception as e:
            self.get_logger().error(f'No se pudo abrir serial: {e}')
            raise

        # Timer de polling serial a 100 Hz
        self.timer = self.create_timer(0.01, self.read_serial)

        self.get_logger().info('==========================================')
        self.get_logger().info(' SERIAL_SENSORS NODE STARTED')
        self.get_logger().info(f' Puerto : {port}')
        self.get_logger().info(f' Frame  : {self.frame_id}')
        self.get_logger().info(' Formato: t,ds,yaw,pitch,roll,qw,qx,qy,qz,gx,gy,gz,ax,ay,az')
        self.get_logger().info(' Debug : /wheel/debug  -> x=v [m/s], y=ds [m], z=dt [s]')
        self.get_logger().info('==========================================')

    # ------------------------------------------------------------------
    def read_serial(self):
        raw = ''
        try:
            raw = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if not raw or raw.startswith('#'):
                return

            parts = raw.split(',')
            if len(parts) != 15:
                return

            # ---------- Parse ----------
            t_ms  = float(parts[0])   # timestamp ESP32 [ms]
            ds    = float(parts[1])   # distancia incremental [m]

            yaw   = float(parts[2])   # grados
            pitch = float(parts[3])   # grados
            roll  = float(parts[4])   # grados

            qw = float(parts[5])
            qx = float(parts[6])
            qy = float(parts[7])
            qz = float(parts[8])

            gx = float(parts[9])      # rad/s
            gy = float(parts[10])     # rad/s
            gz = float(parts[11])     # rad/s

            ax = float(parts[12])     # m/s²
            ay = float(parts[13])     # m/s²
            az = float(parts[14])     # m/s²

            # ---------- dt usando timestamp del ESP32 ----------
            if self.last_t_esp_ms is None:
                self.last_t_esp_ms = t_ms
                return  # primera muestra: solo inicializa

            dt = (t_ms - self.last_t_esp_ms) / 1000.0
            self.last_t_esp_ms = t_ms

            # Validar dt
            if dt < self.min_dt or dt > self.max_dt:
                self.get_logger().warn(f'dt fuera de rango: {dt:.6f} s — descartado')
                return

            # ---------- Velocidad lineal ----------
            v_mps = ds / dt

            # Deadband para reducir drift
            if abs(v_mps) < self.v_deadband:
                v_mps = 0.0

            now = self.get_clock().now().to_msg()

            # ---------- Publish IMU ----------
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

            # Covarianzas desconocidas
            imu_msg.orientation_covariance[0] = -1.0
            imu_msg.angular_velocity_covariance[0] = -1.0
            imu_msg.linear_acceleration_covariance[0] = -1.0

            self.pub_imu.publish(imu_msg)

            # ---------- Publish Euler ----------
            euler_msg = Vector3Stamped()
            euler_msg.header.stamp = now
            euler_msg.header.frame_id = self.frame_id
            euler_msg.vector.x = roll    # deg
            euler_msg.vector.y = pitch   # deg
            euler_msg.vector.z = yaw     # deg
            self.pub_euler.publish(euler_msg)

            # ---------- Publish Twist ----------
            twist_msg = TwistStamped()
            twist_msg.header.stamp = now
            twist_msg.header.frame_id = self.frame_id
            twist_msg.twist.linear.x = v_mps    # m/s
            twist_msg.twist.angular.z = gz      # rad/s
            self.pub_twist.publish(twist_msg)

            # ---------- Publish Debug ----------
            debug_msg = Vector3Stamped()
            debug_msg.header.stamp = now
            debug_msg.header.frame_id = self.frame_id
            debug_msg.vector.x = v_mps   # m/s
            debug_msg.vector.y = ds      # m
            debug_msg.vector.z = dt      # s
            self.pub_debug.publish(debug_msg)

            # ---------- Log throttled ----------
            if self.debug_log:
                self.get_logger().info(
                    f'DEBUG -> t_esp={t_ms:.1f} ms | ds={ds:.6f} m | dt={dt:.6f} s | '
                    f'v={v_mps:.4f} m/s | yaw={yaw:.2f} deg | gz={gz:.4f} rad/s',
                    throttle_duration_sec=0.5
                )

        except (ValueError, IndexError) as e:
            self.get_logger().warn(f'Parse error: {e} | línea: "{raw}"')
        except Exception as e:
            self.get_logger().error(f'Error inesperado: {e}')


# -----------------------------------------------------------------------
def main():
    rclpy.init()
    node = SerialSensorsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()