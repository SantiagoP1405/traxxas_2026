import rclpy

from rclpy.node import Node
from std_msgs.msg import Bool, Float32


class StopConfirmation(Node):
    def __init__(self):
        super().__init__('stop_confirmation')

        self.declare_parameter('score_gain', 2)
        self.declare_parameter('score_decay', 1)
        self.declare_parameter('score_max', 10)
        self.declare_parameter('score_trigger', 5)
        self.declare_parameter('min_area_for_confirmation', 400.0)
        self.declare_parameter('strong_area_for_confirmation', 5000.0)
        self.declare_parameter('max_distance_for_confirmation', 0.65) #3.0 antes, 0.65 ahora para Traxxas

        self.score_gain = int(self.get_parameter('score_gain').value)
        self.score_decay = int(self.get_parameter('score_decay').value)
        self.score_max = int(self.get_parameter('score_max').value)
        self.score_trigger = int(self.get_parameter('score_trigger').value)
        self.min_area = float(self.get_parameter('min_area_for_confirmation').value)
        self.strong_area = float(self.get_parameter('strong_area_for_confirmation').value)
        self.max_distance = float(self.get_parameter('max_distance_for_confirmation').value)

        self.last_raw_detected = False
        self.last_area = 0.0
        self.last_distance = -1.0
        self.score = 0

        self.create_subscription(Bool, '/traxxas/stop_sign/detected_raw', self.raw_callback, 10)
        self.create_subscription(Float32, '/traxxas/stop_sign/area', self.area_callback, 10)
        self.create_subscription(Float32, '/traxxas/stop_sign/distance_m', self.distance_callback, 10)

        self.confirmed_pub = self.create_publisher(Bool, '/traxxas/stop_sign/confirmed', 10)
        self.score_pub = self.create_publisher(Float32, '/traxxas/stop_sign/score', 10)

        self.timer = self.create_timer(0.1, self.update_logic)

        self.get_logger().info('Nodo de confirmación temporal iniciado')

    def raw_callback(self, msg):
        self.last_raw_detected = bool(msg.data)

    def area_callback(self, msg):
        self.last_area = float(msg.data)

    def distance_callback(self, msg):
        self.last_distance = float(msg.data)

    def update_logic(self):
        valid_area = self.last_area >= self.min_area
        valid_distance = (self.last_distance > 0.0) and (self.last_distance <= self.max_distance)
        strong_area = self.last_area >= self.strong_area

        if self.last_raw_detected:
            if valid_distance and valid_area:
                evidence = True
            elif strong_area:
                evidence = True
            else:  
                evidence = False
        else:
            evidence = False


        if evidence:
            self.score = min(self.score_max, self.score + self.score_gain)
        else:
            self.score = max(0, self.score - self.score_decay)

        confirmed = self.score >= self.score_trigger

        conf_msg = Bool()
        conf_msg.data = confirmed
        self.confirmed_pub.publish(conf_msg)

        score_msg = Float32()
        score_msg.data = float(self.score)
        self.score_pub.publish(score_msg)


def main(args=None):
    rclpy.init(args=args)
    node = StopConfirmation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()