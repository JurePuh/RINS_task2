"""Entry point for the movement node.

Owns very little logic — just builds the tree, wires the /face_detect
subscription, and hands control to py_trees_ros' tick_tock loop.
"""

import py_trees_ros
import rclpy

from task2.movement.tree import attach_face_subscription, build_root


# How often the tree ticks. 100 ms = 10 Hz. Fast enough that a face arrival
# reacts within one tick; slow enough that we're not hammering the CPU.
_TICK_PERIOD_MS = 100


def main() -> None:
    rclpy.init()

    tree = py_trees_ros.trees.BehaviourTree(
        root=build_root(),
        unicode_tree_debug=True,  # prints the tree status each tick to stdout
    )
    # setup() creates and owns an rclpy node we can pull off `tree.node`.
    tree.setup(node_name="movement", timeout=15.0)

    attach_face_subscription(tree.node)

    tree.tick_tock(period_ms=_TICK_PERIOD_MS)

    try:
        rclpy.spin(tree.node)
    except KeyboardInterrupt:
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
