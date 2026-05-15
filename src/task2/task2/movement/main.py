
import rclpy

from task1.movement.controller import MovementController


def main():
    print('movement node starting.')

    rclpy.init(args=None)
    node = MovementController()
    executor = rclpy.executors.MultiThreadedExecutor() # type: ignore
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.shutdown()

if __name__ == '__main__':
    main()
