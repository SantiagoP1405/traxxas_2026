#!/usr/bin/env python3
"""
 Path Recorder Node for Traxxas / TMR - Dsositos
 
-----------------------------------------------------------------------
 ROS2 node that subscribes to a Vector3Stamped pose topic (x, y, theta)
 and records the full robot trajectory into a CSV file. Useful for 
 path generation, analysis, mapping, and Pure Pursuit reference tracks.

"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped

import csv, math, os
from datetime import datetime
import atexit  


def dist(p, q):
    """Euclidean distance between two 2D points."""
    return math.hypot(p[0] - q[0], p[1] - q[1])


class PathRecorder(Node):
    """Records QCar or AMR trajectories into a CSV file."""

    def __init__(self):
        super().__init__('path_recorder')

        # ----------------------------------------------------------
        # Parameters
        # ----------------------------------------------------------
        self.declare_parameter('topic', '/pose_traxxas')  
        # self.declare_parameter('topic', '/qcar/pose')
        self.declare_parameter('outfile', '')      

        self.points = []

        topic = self.get_parameter('topic').value
        self.outfile = self.get_parameter('outfile').value

        # Default output filename
        if not self.outfile:
            stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            self.outfile = os.path.join(os.getcwd(), f'waypoints_{stamp}.csv')

        # ----------------------------------------------------------
        # Subscriber
        # ----------------------------------------------------------
        self.create_subscription(Vector3Stamped, topic, self.pose_cb, 10)

        # Save on exit
        atexit.register(self.save_points)

        self.get_logger().info(f"Recording poses from: {topic}")
        self.get_logger().info(f"Saving to file: {self.outfile}")

    # ----------------------------------------------------------
    # Callback: store trajectory points
    # ----------------------------------------------------------
    def pose_cb(self, msg: Vector3Stamped):
        x = msg.vector.x
        y = msg.vector.y
        theta = msg.vector.z  # radians
        self.points.append((x, y, theta))

    # ----------------------------------------------------------
    # Save CSV file
    # ----------------------------------------------------------
    def save_points(self):
        try:
            with open(self.outfile, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['x', 'y', 'theta'])
                writer.writerows(self.points)

            self.get_logger().info(
                f"Saved {len(self.points)} points to: {self.outfile}"
            )

        except Exception as e:
            self.get_logger().error(f"Error saving CSV: {e}")


# ----------------------------------------------------------
# MAIN
# ----------------------------------------------------------
def main():
        # marml 2025
        rclpy.init()
        node = PathRecorder()

        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.save_points()  # failsafe storage
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
        main()
