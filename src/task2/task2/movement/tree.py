"""Behaviour-tree assembly + ROS subscription wiring for the movement node.

Root structure:

    Sequence "Mission" (memory=True)
    ├── Selector "Phase1" (memory=False)        # priority chooser
    │   ├── RunAnomalyTask
    │   ├── VisitHorizontalBarrel
    │   ├── ApproachUnhandledFace
    │   ├── FollowPath
    │   └── MarkExplorationDone                 # sentinel
    ├── Phase2_ExitAndReport
    └── FinalDance

Priorities (highest first): anomaly > barrel > face > explore. `memory=False`
on Phase1 makes every tick re-check from the top, so a fresh detection
preempts lower-priority work. Phase1 only "completes" when MarkExplorationDone
fires, which requires every queue to be empty and every asked-for task done.
"""

from collections import deque

import py_trees
import py_trees_ros
import rclpy.node

from msg_types.msg import FaceDetect, RingDetect

from task2.movement import blackboard as bb
from task2.movement.behaviours import (
    anomaly,
    barrel,
    exit_phase,
    face_interaction,
    follow_path,
    go_to_face,
)
from task2.movement.behaviours.conditions import (
    AnomalyGreenActive,
    BarrelVisitPending,
    MarkExplorationDone,
    AnomalyRedActive,
)
from task2.movement.behaviours.face_conditions import HasUnhandledFace, MarkFaceHandled
from task2.movement.models import (
    AnomalyTask,
    Barrel,
    CountRingsTask,
    InspectBarrelsTask,
    Ring,
)


def _seed_blackboard() -> None:
    """Initial values for every blackboard key — must happen before first tick."""
    w = py_trees.blackboard.Client(name="bootstrap")
    keys = [
        bb.PENDING_FACES, bb.HANDLED_FACES, bb.RECOMPUTE_FACE_DESTINATION,
        bb.ACTIVE_PERSON, bb.CONVERSATION_RESULT,
        bb.TASK_COUNT_RINGS, bb.TASK_INSPECT_BARRELS,
        bb.TASK_ANOMALY_RED, bb.TASK_ANOMALY_GREEN,
        bb.PENDING_BARRELS, bb.BARREL_ACTIVE,
        bb.ANOMALY_RED_ACTIVE, bb.ANOMALY_GREEN_ACTIVE,
        bb.EXPLORATION_DONE,
    ]
    for k in keys:
        w.register_key(key=k, access=py_trees.common.Access.WRITE)

    w.set(bb.PENDING_FACES, deque())
    w.set(bb.HANDLED_FACES, set())
    w.set(bb.RECOMPUTE_FACE_DESTINATION, False)
    w.set(bb.ACTIVE_PERSON, None)
    w.set(bb.CONVERSATION_RESULT, "")
    w.set(bb.TASK_COUNT_RINGS, CountRingsTask())
    w.set(bb.TASK_INSPECT_BARRELS, InspectBarrelsTask())
    w.set(bb.TASK_ANOMALY_RED, AnomalyTask())
    w.set(bb.TASK_ANOMALY_GREEN, AnomalyTask())
    w.set(bb.PENDING_BARRELS, deque())
    w.set(bb.BARREL_ACTIVE, None)
    w.set(bb.ANOMALY_RED_ACTIVE, False)
    w.set(bb.ANOMALY_GREEN_ACTIVE, False)
    w.set(bb.EXPLORATION_DONE, False)


def _build_phase1() -> py_trees.composites.Selector:
    run_red = py_trees.composites.Sequence(name="RunAnomalyRed", memory=True)
    run_red.add_children([AnomalyRedActive(), anomaly.build_red()])

    run_green = py_trees.composites.Sequence(name="RunAnomalyGreen", memory=True)
    run_green.add_children([AnomalyGreenActive(), anomaly.build_green()])

    visit_barrel = py_trees.composites.Sequence(name="VisitHorizontalBarrel", memory=True)
    visit_barrel.add_children([BarrelVisitPending(), barrel.build()])

    approach_face = py_trees.composites.Sequence(name="ApproachUnhandledFace", memory=True)
    approach_face.add_children([
        HasUnhandledFace(),
        go_to_face.build(),
        face_interaction.build(),
        MarkFaceHandled(),
    ])

    phase1 = py_trees.composites.Selector(name="Phase1", memory=False)
    phase1.add_children([
        run_red,
        run_green,
        visit_barrel,
        approach_face,
        follow_path.build(),
        MarkExplorationDone(),
    ])
    return phase1


def build_root() -> py_trees.behaviour.Behaviour:
    _seed_blackboard()

    mission = py_trees.composites.Sequence(name="Mission", memory=True)
    mission.add_children([
        _build_phase1(),
        exit_phase.build(),
        exit_phase.FinalDance(),
    ])
    return mission


# ---------------------------------------------------------------------------
# Subscriptions — same reasoning as before: plain rclpy callbacks, not tree
# behaviours, so we don't drop bursts that arrive between ticks.
# ---------------------------------------------------------------------------

def attach_face_subscription(node: rclpy.node.Node) -> None:
    reader = py_trees.blackboard.Client(name="face_subscription")
    reader.register_key(key=bb.PENDING_FACES, access=py_trees.common.Access.READ)
    reader.register_key(key=bb.HANDLED_FACES, access=py_trees.common.Access.READ)
    reader.register_key(key=bb.RECOMPUTE_FACE_DESTINATION, access=py_trees.common.Access.WRITE)

    def on_face(msg: FaceDetect) -> None:
        pending = reader.get(bb.PENDING_FACES)
        handled = reader.get(bb.HANDLED_FACES)

        if msg.id in handled:
            node.get_logger().debug(f"ignoring face_detect for already-handled {msg.id}")
            return

        for i, f in enumerate(pending):
            if f.id == msg.id:
                pending[i] = msg
                if i == 0:
                    reader.set(bb.RECOMPUTE_FACE_DESTINATION, True)
                node.get_logger().info(
                    f"updated face {msg.id} location to ({msg.x:.2f}, {msg.y:.2f})"
                )
                return

        pending.append(msg)
        node.get_logger().info(
            f"new face {msg.id} queued at ({msg.x:.2f}, {msg.y:.2f}); "
            f"pending={len(pending)}"
        )

    node.create_subscription(FaceDetect, "/face_detect", on_face, 10)


def attach_ring_subscription(node: rclpy.node.Node) -> None:
    reader = py_trees.blackboard.Client(name="ring_subscription")
    reader.register_key(key=bb.TASK_COUNT_RINGS, access=py_trees.common.Access.READ)

    def on_ring(msg: RingDetect) -> None:
        task = reader.get(bb.TASK_COUNT_RINGS)
        rid = str(msg.id)
        for existing in task.rings:
            if existing.id == rid:
                existing.x, existing.y = msg.x, msg.y
                node.get_logger().info(f"updated ring {rid} location")
                return
        task.rings.append(Ring(x=msg.x, y=msg.y, color=msg.color, id=rid))
        node.get_logger().info(
            f"new ring {rid} ({msg.color}) at ({msg.x:.2f}, {msg.y:.2f}); "
            f"total={len(task.rings)}"
        )

    node.create_subscription(RingDetect, "/ring_detect", on_ring, 10)


def attach_barrel_subscription(node: rclpy.node.Node) -> None:
    """Wire /barrel_detect → InspectBarrelsTask catalogue + /pending_barrels queue.

    Imports BarrelDetect lazily so the tree still comes up if msg_types hasn't
    been rebuilt yet.
    """
    try:
        from msg_types.msg import BarrelDetect
    except ImportError:
        node.get_logger().warning(
            "msg_types.msg.BarrelDetect not available — skipping barrel subscription"
        )
        return

    reader = py_trees.blackboard.Client(name="barrel_subscription")
    reader.register_key(key=bb.TASK_INSPECT_BARRELS, access=py_trees.common.Access.READ)
    reader.register_key(key=bb.PENDING_BARRELS, access=py_trees.common.Access.READ)

    def on_barrel(msg: BarrelDetect) -> None:
        task = reader.get(bb.TASK_INSPECT_BARRELS)
        queue = reader.get(bb.PENDING_BARRELS)
        bid = str(msg.id)

        existing = next((b for b in task.barrels if b.id == bid), None)
        if existing is not None:
            existing.x, existing.y = msg.x, msg.y
            existing.color = msg.color
            existing.horizontal = msg.horizontal
            node.get_logger().info(f"updated barrel {bid}")
            return

        b = Barrel(
            x=msg.x, y=msg.y, color=msg.color, horizontal=msg.horizontal, id=bid,
        )
        task.barrels.append(b)
        if b.horizontal:
            queue.append(b)
        node.get_logger().info(
            f"new barrel {bid} ({b.color}, horiz={b.horizontal}) "
            f"at ({b.x:.2f}, {b.y:.2f}); total={len(task.barrels)}"
        )

    node.create_subscription(BarrelDetect, "/barrel_detect", on_barrel, 10)
