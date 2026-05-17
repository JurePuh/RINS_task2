"""Behaviour-tree assembly for the movement node.

Layout (see plan):

    Root: Selector "TaskSelector" (memory=False)
    ├── Sequence "ApproachUnhandledFace" (memory=True)
    │   ├── HasUnhandledFace
    │   ├── GoToFace           # ComputeFaceDestination → NavigateToPose
    │   └── MarkFaceHandled
    └── FollowPath             # waypoint loop

`memory=False` on the root Selector means every tick re-checks
HasUnhandledFace from scratch — the moment a new face appears on the
blackboard the next tick switches branches, automatically cancelling any
in-flight nav2 goal under FollowPath.

Face ingestion is a plain rclpy subscription (not a behaviour). Reasoning is
in [blackboard.py](blackboard.py) and the plan file; the short version is
that behaviour-style subscribers only sample at tick time and drop bursts.
"""

from collections import deque

import py_trees
import py_trees_ros
import rclpy.node

from msg_types.msg import FaceDetect

from task2.movement import blackboard as bb
from task2.movement.behaviours import follow_path, go_to_face
from task2.movement.behaviours.conditions import HasUnhandledFace, MarkFaceHandled


def build_root() -> py_trees.behaviour.Behaviour:
    """Construct the root behaviour. Blackboard initial values set here too."""

    # Seed the blackboard before any behaviour runs. Without this, the first
    # tick of HasUnhandledFace would KeyError on a missing variable.
    writer = py_trees.blackboard.Client(name="bootstrap")
    writer.register_key(key=bb.PENDING_FACES, access=py_trees.common.Access.WRITE)
    writer.register_key(key=bb.HANDLED_FACES, access=py_trees.common.Access.WRITE)
    writer.set(bb.PENDING_FACES, deque())
    writer.set(bb.HANDLED_FACES, set())

    approach = py_trees.composites.Sequence(name="ApproachUnhandledFace", memory=True)
    approach.add_children([
        HasUnhandledFace(),
        go_to_face.build(),
        MarkFaceHandled(),
    ])

    root = py_trees.composites.Selector(name="TaskSelector", memory=False)
    root.add_children([approach, follow_path.build()])
    return root


def attach_face_subscription(node: rclpy.node.Node) -> None:
    """Wire `/face_detect` into the blackboard's pending_faces deque.

    Called once, after the py_trees_ros tree has been set up (so the
    blackboard already has its seed values). Runs on the rclpy executor
    thread; the tree thread reads/pops on its own thread. deque is
    thread-safe for single-producer / single-consumer use, which is exactly
    this pattern, so no extra locking is needed.
    """
    reader = py_trees.blackboard.Client(name="face_subscription")
    reader.register_key(key=bb.PENDING_FACES, access=py_trees.common.Access.READ)
    reader.register_key(key=bb.HANDLED_FACES, access=py_trees.common.Access.READ)

    def on_face(msg: FaceDetect) -> None:
        pending = reader.get(bb.PENDING_FACES)
        handled = reader.get(bb.HANDLED_FACES)
        if msg.id in handled or any(f.id == msg.id for f in pending):
            return  # all dedup lives here; detect_faces just fires once
        pending.append(msg)
        node.get_logger().info(
            f"new face {msg.id} queued at ({msg.x:.2f}, {msg.y:.2f}); "
            f"pending={len(pending)}"
        )

    node.create_subscription(FaceDetect, "/face_detect", on_face, 10)
