import py_trees
import py_trees_ros
import rclpy

from task2.movement.tree import attach_face_subscription, build_root


# How often the tree ticks. 100 ms = 10 Hz.
_TICK_PERIOD_MS = 100

# Print the unicode tree to stdout, but only on ticks where node status changed.
_PRINT_TREE_ON_CHANGE = True


def _print_on_change(tree) -> None:
    if not _PRINT_TREE_ON_CHANGE:
        return
    if tree.snapshot_visitor.changed:
        print(py_trees.display.unicode_tree(
            root=tree.root,
            visited=tree.snapshot_visitor.visited,
            previously_visited=tree.snapshot_visitor.previously_visited,
        ), flush=True)


def main() -> None:
    rclpy.init()

    tree = py_trees_ros.trees.BehaviourTree(
        root=build_root(),
        unicode_tree_debug=False,
    )
    tree.add_post_tick_handler(_print_on_change)
    # setup() creates and owns an rclpy node we can pull off `tree.node`.
    tree.setup(node_name="movement", timeout=15.0)
    assert tree.node is not None, "tree.node must be set after setup()"
    node = tree.node

    attach_face_subscription(node)

    tree.tick_tock(period_ms=_TICK_PERIOD_MS)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
