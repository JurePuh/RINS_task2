import rclpy

from task2.map_query.node import WallNormalAtNode


def main():
    rclpy.init()
    node = WallNormalAtNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()