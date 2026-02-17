#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math



class PWMToCmdVelNode(Node):
    def __init__(self):
        self.wheelbase = 0.35  # meters
        self.max_steering_angle = 0.5236  # 30 degrees in radians

        super().__init__('pwm_to_cmd_vel_node')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Subscribers
        self.throttle_sub = self.create_subscription(
            String,
            '/throttle_motor',
            self.throttle_callback,
            10
        )
        self.direction_sub = self.create_subscription(
            String,
            '/direction_servo',
            self.direction_callback,
            10
        )
        
        # PWM ranges for throttle
        self.throttle_neutral = 2457
        self.throttle_max_forward = 3276
        self.throttle_max_reverse = 1638
        
        # PWM ranges for direction
        self.direction_center = 2642
        self.direction_right = 3276
        self.direction_left = 1669
        
        # Current values
        self.current_linear_x = 0.0
        self.current_angular_z = 0.0
        
        self.get_logger().info('PWM to cmd_vel node initialized')
        self.get_logger().info(f'Throttle - Reverse: {self.throttle_max_reverse}, Neutral: {self.throttle_neutral}, Forward: {self.throttle_max_forward}')
        self.get_logger().info(f'Direction - Left: {self.direction_left}, Center: {self.direction_center}, Right: {self.direction_right}')
    
    def map_value(self, value, in_min, in_mid, in_max, out_min, out_mid, out_max):
        """
        Map a value with a neutral point from input range to output range
        """
        if value <= in_mid:
            # Map from in_min to in_mid -> out_min to out_mid
            if in_mid == in_min:
                return out_mid
            normalized = (value - in_min) / (in_mid - in_min)
            return out_min + normalized * (out_mid - out_min)
        else:
            # Map from in_mid to in_max -> out_mid to out_max
            if in_max == in_mid:
                return out_mid
            normalized = (value - in_mid) / (in_max - in_mid)
            return out_mid + normalized * (out_max - out_mid)
    
    def throttle_callback(self, msg):
        try:
            pwm_value = int(msg.data)
            
            # Map throttle PWM to linear velocity (-1.0 to 1.0)
            self.current_linear_x = self.map_value(
                pwm_value,
                self.throttle_max_reverse,  # 1638
                self.throttle_neutral,       # 2457
                self.throttle_max_forward,   # 3276
                -1.0,  # max reverse
                0.0,   # neutral
                1.0    # max forward
            )
            
            # Clamp values
            self.current_linear_x = max(-1.0, min(1.0, self.current_linear_x))
            
            self.get_logger().debug(f'Throttle PWM: {pwm_value} -> linear.x: {self.current_linear_x:.3f}')
            self.publish_cmd_vel()
            
        except ValueError:
            self.get_logger().warn(f'Invalid throttle value received: {msg.data}')
    
    def direction_callback(self, msg):
        pwm_value = int(msg.data)

        # Map PWM to steering angle δ in radians
        # Left -> +max_angle
        # Center -> 0
        # Right -> -max_angle

        delta = self.map_value(
            pwm_value,
            self.direction_left,
            self.direction_center,
            self.direction_right,
            self.max_steering_angle,
            0.0,
            -self.max_steering_angle
        )

        # Deadzone pequeña
        if abs(delta) < 0.02:
            delta = 0.0

        # Ackermann model
        # ω = V / L * tan(δ)
        if abs(self.current_linear_x) > 0.001:
            self.current_angular_z = (
                self.current_linear_x / self.wheelbase
            ) * math.tan(delta)
        else:
            self.current_angular_z = 0.0

        self.get_logger().info(
            f'PWM: {pwm_value} -> delta: {delta:.3f} rad -> omega: {self.current_angular_z:.3f}'
        )

        self.publish_cmd_vel()
    
    def publish_cmd_vel(self):
        """Publish the current velocity command"""
        twist = Twist()
        twist.linear.x = self.current_linear_x
        twist.angular.z = self.current_angular_z
        
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = PWMToCmdVelNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()