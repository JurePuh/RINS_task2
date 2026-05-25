"""Periodically prints the robot's map-frame pose in Pose(x, y, theta) form.

Useful for building waypoint lists by hand: drive the robot to a spot, copy
the printed line into your path.
"""

import math

import rclpy
import tf2_ros
from rclpy.node import Node


class MapPosePrinter(Node):
    def __init__(self):
        super().__init__("map_pose_printer")
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._timer = self.create_timer(1.0, self._tick)
        self.get_logger().info("map_pose_printer ready")

    def _tick(self):
        try:
            t = self._tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time()
            )
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"tf map<-base_link unavailable: {e}")
            return

        x = t.transform.translation.x
        y = t.transform.translation.y
        q = t.transform.rotation
        # Yaw from quaternion.
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        theta = math.atan2(siny_cosp, cosy_cosp)

        # ` ` flag in format spec inserts a leading space for non-negative
        # numbers so the decimal points line up with negative entries.
        print(f"Pose({x: .2f}, {y: .2f}, {theta: .2f}),", flush=True)


def main():
    rclpy.init()
    node = MapPosePrinter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
