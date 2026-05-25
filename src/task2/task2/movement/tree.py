from collections import deque
from msg_types import msg
import py_trees

import rclpy.node
from msg_types.msg import FaceDetect, RingDetect, BarrelDetect

from task2.movement import blackboard as bb
from task2.movement.behaviours import room1, room2
from task2.movement.viz import Visualizer
from task2.movement.models import (
    AnomalyTask,
    Barrel,
    CountRingsTask,
    InspectBarrelsTask,
    Person,
    Point,
    Ring,
)


def _seed_blackboard() -> None:
    """Initial values for every blackboard key — must happen before first tick."""
    w = py_trees.blackboard.Client(name="bootstrap")
    keys = [
        bb.PENDING_PEOPLE, bb.HANDLED_PEOPLE, bb.RECOMPUTE_FACE_DESTINATION,
        bb.CONVERSATION_RESULT, bb.LAST_HANDLED_PERSON,
        bb.TASK_COUNT_RINGS, bb.TASK_INSPECT_BARRELS,
        bb.TASK_ANOMALY_RED, bb.TASK_ANOMALY_GREEN,
        bb.PENDING_BARRELS, bb.BARREL_ACTIVE,
        bb.ANOMALY_RED_ACTIVE, bb.ANOMALY_GREEN_ACTIVE,
    ]
    for k in keys:
        w.register_key(key=k, access=py_trees.common.Access.WRITE)

    w.set(bb.PENDING_PEOPLE, deque()) # deque() 
    w.set(bb.HANDLED_PEOPLE, set()) # set()
    w.set(bb.LAST_HANDLED_PERSON, None) # None
    w.set(bb.RECOMPUTE_FACE_DESTINATION, False) # False
    w.set(bb.CONVERSATION_RESULT, "") # "" (empty string)
    w.set(bb.TASK_COUNT_RINGS, CountRingsTask()) # CountRingsTask() 
    w.set(bb.TASK_INSPECT_BARRELS, InspectBarrelsTask()) # InspectBarrelsTask() 
    w.set(bb.TASK_ANOMALY_RED, AnomalyTask()) # AnomalyTask() 
    w.set(bb.TASK_ANOMALY_GREEN, AnomalyTask()) # AnomalyTask() 
    w.set(bb.PENDING_BARRELS, deque()) # deque()
    w.set(bb.BARREL_ACTIVE, None) # None
    w.set(bb.ANOMALY_RED_ACTIVE, False) # False
    w.set(bb.ANOMALY_GREEN_ACTIVE, False) # False


def build_root() -> py_trees.behaviour.Behaviour:
    _seed_blackboard()
    # Logger not yet available here (no node); use py_trees' own logger.
    py_trees.logging.Logger("movement.tree").info(
        "blackboard seeded: pending_people=0 handled_people=0 pending_barrels=0 "
        "barrel_active=None anomaly_red_active=False anomaly_green_active=False"
    )

    mission = py_trees.composites.Sequence(name="Mission", memory=True)
    mission.add_children([
        room1.build(),
        room2.build(),
        room2.FinalDance(),
    ])
    return mission


# --- Subscriptions to ROS topics, to update blackboard state from sensors ---

def attach_person_subscription(node: rclpy.node.Node, viz: Visualizer) -> None:
    logger = node.get_logger()
    reader = py_trees.blackboard.Client(name="person_subscription")
    reader.register_key(key=bb.PENDING_PEOPLE, access=py_trees.common.Access.READ)
    reader.register_key(key=bb.HANDLED_PEOPLE, access=py_trees.common.Access.READ)
    reader.register_key(key=bb.RECOMPUTE_FACE_DESTINATION, access=py_trees.common.Access.WRITE)

    def on_person(msg: FaceDetect) -> None:
        pending = reader.get(bb.PENDING_PEOPLE)
        handled = reader.get(bb.HANDLED_PEOPLE)
        pid = msg.id
        # Mirror the new/updated person position to RViz.
        viz.update_person(pid, msg.x, msg.y)
        logger.debug(
            f"on_person: id={pid} xy=({msg.x:.2f},{msg.y:.2f}) "
            f"pending={len(pending)} handled={len(handled)}"
        )

        # Check if we've already talked to this person
        if pid in handled:
            logger.info(f"on_person: already handled person {pid}; ignoring")
            return

        # Check if this is an already-pending person with an updated location
        for i, person in enumerate(pending):
            if person.face_id == pid:
                person.point = Point(msg.x, msg.y)
                if i == 0:
                    reader.set(bb.RECOMPUTE_FACE_DESTINATION, True)
                    logger.info(
                        f"on_person: head-of-queue person {pid} moved; "
                        f"RECOMPUTE_FACE_DESTINATION=True"
                    )
                logger.info(
                    f"on_person: updated person {pid} location to ({msg.x:.2f}, {msg.y:.2f})"
                )
                return

        # Otherwise, add new person to pending queue
        pending.append(Person(point=Point(msg.x, msg.y), face_id=pid))
        logger.info(
            f"on_person: new person {pid} queued at ({msg.x:.2f}, {msg.y:.2f}); "
            f"pending={len(pending)}"
        )
        logger.debug(f"on_person: new person msg fields: id={pid} x={msg.x} y={msg.y}")

    node.create_subscription(FaceDetect, "/face_detect", on_person, 10)
    logger.debug("subscribed to /face_detect")


def attach_ring_subscription(node: rclpy.node.Node, viz: Visualizer) -> None:
    logger = node.get_logger()
    reader = py_trees.blackboard.Client(name="ring_subscription")
    reader.register_key(key=bb.TASK_COUNT_RINGS, access=py_trees.common.Access.READ)

    def on_ring(msg: RingDetect) -> None:
        task = reader.get(bb.TASK_COUNT_RINGS)
        rid = msg.id
        viz.update_ring(rid, msg.x, msg.y, msg.color)
        logger.debug(
            f"on_ring: id={rid} color={msg.color} xy=({msg.x:.2f},{msg.y:.2f}) "
            f"known_total={len(task.rings)}"
        )

        # Check if this is an already-known ring with an updated location
        for existing in task.rings:
            if existing.id == rid:
                existing.point = Point(msg.x, msg.y)
                existing.color = msg.color
                logger.info(f"on_ring: updated ring {rid} location to ({msg.x:.2f}, {msg.y:.2f})")
                return

        # Otherwise, add new ring to task list
        task.rings.append(Ring(point=Point(msg.x, msg.y), color=msg.color, id=rid))
        logger.info(
            f"on_ring: new ring {rid} ({msg.color}) at ({msg.x:.2f}, {msg.y:.2f}); "
            f"total={len(task.rings)}"
        )
        logger.debug(f"on_ring: new ring msg fields: id={rid} color={msg.color} x={msg.x} y={msg.y}")

    node.create_subscription(RingDetect, "/ring_detect", on_ring, 10)
    logger.debug("subscribed to /ring_detect")


def attach_barrel_subscription(node: rclpy.node.Node, viz: Visualizer) -> None:
    logger = node.get_logger()
    reader = py_trees.blackboard.Client(name="barrel_subscription")
    reader.register_key(key=bb.TASK_INSPECT_BARRELS, access=py_trees.common.Access.READ)
    reader.register_key(key=bb.PENDING_BARRELS, access=py_trees.common.Access.READ)

    def on_barrel(msg: BarrelDetect) -> None:
        task = reader.get(bb.TASK_INSPECT_BARRELS)
        queue = reader.get(bb.PENDING_BARRELS)
        bid = msg.id
        viz.update_barrel(bid, msg.x, msg.y, msg.color, msg.horizontal)
        logger.debug(
            f"on_barrel: id={bid} color={msg.color} horiz={msg.horizontal} "
            f"xy=({msg.x:.2f},{msg.y:.2f}) known_total={len(task.barrels)} "
            f"pending_queue={len(queue)}"
        )

        # Check if this is an already-known barrel with an updated location
        existing = next((b for b in task.barrels if b.id == bid), None)
        if existing is not None:
            existing.point = Point(msg.x, msg.y)
            existing.color = msg.color
            existing.horizontal = msg.horizontal
            logger.info(
                f"on_barrel: updated barrel {bid} to ({msg.x:.2f}, {msg.y:.2f}) "
                f"color={msg.color} horiz={msg.horizontal}"
            )
            return

        # Otherwise, add new barrel to task list (and pending queue, if horizontal)
        barrel = Barrel(
            point=Point(msg.x, msg.y), color=msg.color, horizontal=msg.horizontal, id=bid,
        )
        task.barrels.append(barrel)
        if barrel.horizontal:
            queue.append(barrel)
            logger.info(
                f"on_barrel: horizontal barrel {bid} appended to PENDING_BARRELS; "
                f"queue_size={len(queue)}"
            )
        logger.info(
            f"on_barrel: new barrel {bid} ({barrel.color}, horiz={barrel.horizontal}) "
            f"at ({barrel.point.x:.2f}, {barrel.point.y:.2f}); total={len(task.barrels)}"
        )
        logger.debug(
            f"on_barrel: new barrel msg fields: id={bid} color={msg.color} "
            f"horizontal={msg.horizontal} x={msg.x} y={msg.y}"
        )

    node.create_subscription(BarrelDetect, "/barrel_detect", on_barrel, 10)
    logger.debug("subscribed to /barrel_detect")
