import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class SteeringAngleNode(Node):
    def __init__(self):
        super().__init__('steering_angle_node')

        self.declare_parameter('topic_in', 'direction_servo')
        self.declare_parameter('topic_out', 'steering_angle')

        self.declare_parameter('pwm_min', 1669.0)
        self.declare_parameter('pwm_mid', 2642.0)
        self.declare_parameter('pwm_max', 3276.0)

        self.declare_parameter('delta_max', 0.436)   # ~25°
        self.declare_parameter('invert', False)
        self.declare_parameter('deadband_pwm', 15.0)

        topic_in = self.get_parameter('topic_in').value
        topic_out = self.get_parameter('topic_out').value

        self.pwm_min = float(self.get_parameter('pwm_min').value)
        self.pwm_mid = float(self.get_parameter('pwm_mid').value)
        self.pwm_max = float(self.get_parameter('pwm_max').value)

        self.delta_max = float(self.get_parameter('delta_max').value)
        self.invert = bool(self.get_parameter('invert').value)
        self.deadband = float(self.get_parameter('deadband_pwm').value)

        self.pub = self.create_publisher(Float64, topic_out, 10)
        self.sub = self.create_subscription(String, topic_in, self.cb_pwm, 10)

        self.get_logger().info(
            f"SteeringAngleNode: {topic_in} -> {topic_out} | "
            f"min/mid/max={self.pwm_min}/{self.pwm_mid}/{self.pwm_max} | "
            f"delta_max={self.delta_max} rad"
        )

    def cb_pwm(self, msg: String):
        try:
            pwm = float(msg.data.strip())
        except ValueError:
            self.get_logger().warn(f"No se pudo convertir PWM: '{msg.data}'")
            return

        pwm = clamp(pwm, self.pwm_min, self.pwm_max)

        if abs(pwm - self.pwm_mid) <= self.deadband:
            delta = 0.0
        else:
            if pwm < self.pwm_mid:
                denom = self.pwm_mid - self.pwm_min
                norm = 0.0 if denom == 0 else (pwm - self.pwm_mid) / denom
            else:
                denom = self.pwm_max - self.pwm_mid
                norm = 0.0 if denom == 0 else (pwm - self.pwm_mid) / denom

            norm = clamp(norm, -1.0, 1.0)
            delta = norm * self.delta_max

        if self.invert:
            delta = -delta

        out = Float64()
        out.data = float(delta)
        self.pub.publish(out)


def main():
    rclpy.init()
    node = SteeringAngleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()