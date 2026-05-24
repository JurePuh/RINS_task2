import rclpy

from task2.line_fit.node import LineFitInDirectionNode


def main() -> None:
    rclpy.init()
    node = LineFitInDirectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
