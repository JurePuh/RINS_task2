"""GoAndInspectBarrel subtree."""

import math

import py_trees
import py_trees_ros
import tf2_ros
from nav2_msgs.action import Spin
from rclpy.time import Time
from rclpy.impl.rcutils_logger import RcutilsLogger

from task2.movement import blackboard as bb
from task2.movement.behaviours._nav import (
    NavigateToBlackboardGoal,
    lookup_robot_xy,
    standoff_goal_from_normal,
)
from task2.movement.behaviours.conditions import RecomputeNotRequested
from task2.movement.models import Vector, Barrel


_BARREL_STANDOFF_M = 0.6


def _wrap_to_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class ComputeBarrelDestination(py_trees.behaviour.Behaviour):
    """Compute a standoff goal for the head of PENDING_BARRELS into BARREL_DESTINATION.

    TODO: currently it just goes to a mid-point between robot and barrel; need barrel orientation.
    """

    def __init__(self, name: str = "ComputeBarrelDestination"):
        super().__init__(name=name)
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key(key=bb.PENDING_BARRELS, access=py_trees.common.Access.READ)
        self.bb.register_key(
            key=bb.BARREL_DESTINATION, access=py_trees.common.Access.WRITE
        )
        self.bb.register_key(
            key=bb.RECOMPUTE_BARREL_DESTINATION, access=py_trees.common.Access.WRITE
        )

        self._ros_logger: RcutilsLogger

    def setup(self, **kwargs):
        self.node = kwargs["node"]
        self._ros_logger = self.node.get_logger()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)

    def update(self) -> py_trees.common.Status:
        self.bb.set(bb.RECOMPUTE_BARREL_DESTINATION, False)
        queue = self.bb.get(bb.PENDING_BARRELS)
        assert queue is not None, "ComputeBarrelDestination: PENDING_BARRELS not found on blackboard"
        barrel: Barrel = queue[0]

        robot = lookup_robot_xy(self.tf_buffer, self._ros_logger)
        if robot is None:
            self._ros_logger.warning(
                "ComputeBarrelDestination: robot tf lookup failed; cannot derive "
                "placeholder barrel-orientation vector. This codepath disappears "
                "once barrel detection publishes a real orientation — safe to "
                "ignore as long as it self-recovers on the next tick."
            )
            return py_trees.common.Status.FAILURE

        dx = robot[0] - barrel.point.x
        dy = robot[1] - barrel.point.y
        dist = math.hypot(dx, dy) or 1.0
        normal = Vector(dx / dist, dy / dist)
        self.bb.set(
            bb.BARREL_DESTINATION,
            standoff_goal_from_normal(barrel.point, normal, standoff=_BARREL_STANDOFF_M),
        )
        return py_trees.common.Status.SUCCESS


class TurnTowardsBarrel(py_trees_ros.action_clients.FromConstant):
    """Spin in place so base_link faces the head of PENDING_BARRELS.

    Recomputes target_yaw in initialise() from current robot+barrel poses; if tf
    lookup fails, skips the spin entirely (no-op SUCCESS).
    """

    def __init__(self, name: str = "TurnTowardsBarrel"):
        self._spin_goal = Spin.Goal()
        super().__init__(
            name=name,
            action_type=Spin,
            action_name="spin",
            action_goal=self._spin_goal,
        )
        self._pbb = self.attach_blackboard_client(name=f"{name}_pending")
        self._pbb.register_key(key=bb.PENDING_BARRELS, access=py_trees.common.Access.READ)
        self._skip = False

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self.node = kwargs["node"]
        self._ros_logger: RcutilsLogger = self.node.get_logger()
        # Set up TF listener for robot and barrel poses
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)

    def initialise(self):
        self._skip = False
        queue = self._pbb.get(bb.PENDING_BARRELS)
        assert queue is not None, "TurnTowardsBarrel: PENDING_BARRELS not found on blackboard"
        barrel: Barrel = queue[0]

        # If TF lookup fails, skip rotating
        try:
            t = self.tf_buffer.lookup_transform("map", "base_link", Time())
        except Exception as exc:
            self._ros_logger.warning(
                f"TurnTowardsBarrel: tf lookup map->base_link failed: {exc}; "
                "skipping spin"
            )
            self._skip = True
            return

        # Compute target_yaw to face the barrel from the robot's current pose
        rx = t.transform.translation.x
        ry = t.transform.translation.y
        q = t.transform.rotation
        yaw = math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)
        desired = math.atan2(barrel.point.y - ry, barrel.point.x - rx)
        self._spin_goal.target_yaw = _wrap_to_pi(desired - yaw)
        self._ros_logger.info(
            f"TurnTowardsBarrel: spin {self._spin_goal.target_yaw:+.2f} rad"
        )
        super().initialise()

    def update(self):
        if self._skip:
            return py_trees.common.Status.SUCCESS
        # Return status of spin
        return super().update()


class CheckBarrelLeak(py_trees.behaviour.Behaviour):
    """TODO: Currently just sets barrel as not leaking, need the service to check if it is leaking."""

    def __init__(self, name: str = "CheckBarrelLeak"):
        super().__init__(name=name)
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key(key=bb.PENDING_BARRELS, access=py_trees.common.Access.READ)
        self.bb.register_key(
            key=bb.TASK_INSPECT_BARRELS, access=py_trees.common.Access.READ
        )

    def setup(self, **kwargs):
        self._ros_logger: RcutilsLogger = kwargs["node"].get_logger()

    def update(self) -> py_trees.common.Status:
        queue = self.bb.get(bb.PENDING_BARRELS)
        assert queue is not None, "CheckBarrelLeak: PENDING_BARRELS not found on blackboard"
        barrel: Barrel = queue[0]

        # Set barrel as not leaking (for now)
        barrel.leaking = False
        self._ros_logger.info(f"[stub] barrel {barrel.id} marked not-leaking")
        return py_trees.common.Status.SUCCESS


class ClearActiveBarrel(py_trees.behaviour.Behaviour):
    """Pop the now-handled barrel off PENDING_BARRELS."""

    def __init__(self, name: str = "ClearActiveBarrel"):
        super().__init__(name=name)
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key(key=bb.PENDING_BARRELS, access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        self._ros_logger: RcutilsLogger = kwargs["node"].get_logger()

    def update(self) -> py_trees.common.Status:
        queue = self.bb.get(bb.PENDING_BARRELS)
        assert queue is not None, "ClearActiveBarrel: PENDING_BARRELS not found on blackboard"
        
        # Remove seen barrel
        barrel: Barrel = queue.popleft()
        self._ros_logger.info(f"cleared barrel {barrel.id} from queue")
        return py_trees.common.Status.SUCCESS


def build() -> py_trees.composites.Sequence:
    drive_or_recompute = py_trees.composites.Sequence(
        name="DriveOrRecomputeBarrel", memory=False,
    )
    drive_or_recompute.add_children([
        RecomputeNotRequested(
            name="NoBarrelRecomputeRequested",
            flag_key=bb.RECOMPUTE_BARREL_DESTINATION,
        ),
        NavigateToBlackboardGoal(
            name="NavigateToBarrelDestination",
            goal_key=bb.BARREL_DESTINATION,
        ),
    ])

    seq = py_trees.composites.Sequence(name="GoAndInspectBarrel", memory=True)
    seq.add_children([
        ComputeBarrelDestination(),
        drive_or_recompute,
        TurnTowardsBarrel(),
        CheckBarrelLeak(),
        ClearActiveBarrel(),
    ])
    return seq
