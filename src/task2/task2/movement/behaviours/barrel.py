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
    NavRetryThenSucceed,
    build_nav_goal,
    lookup_robot_xy,
)
from task2.movement.behaviours._speak import Speak
from task2.movement.behaviours.conditions import (
    IsHeadBarrelLeaking,
    RecomputeNotRequested,
)
from task2.movement.models import Barrel, Pose


_BARREL_STANDOFF_M = 1.0


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
        self._ros_logger.debug(
            f"{self.name}: inputs queue_size={len(queue)} "
            f"barrel id={barrel.id} pos=({barrel.point.x:.2f},{barrel.point.y:.2f}) "
            f"color={barrel.color} horiz={barrel.horizontal}"
        )

        robot = lookup_robot_xy(self.tf_buffer, self._ros_logger)
        if robot is None:
            # Fallback: aim straight at the barrel.
            goal = build_nav_goal(Pose(barrel.point.x, barrel.point.y, 0.0))
            self.bb.set(bb.BARREL_DESTINATION, goal)
            self._ros_logger.warning(
                f"{self.name}: robot tf lookup failed; falling back to direct "
                f"goal at barrel {barrel.id} ({barrel.point.x:.2f}, {barrel.point.y:.2f})"
            )
            return py_trees.common.Status.SUCCESS

        dx = barrel.point.x - robot[0]
        dy = barrel.point.y - robot[1]
        dist = math.hypot(dx, dy) or 1.0
        ux, uy = dx / dist, dy / dist
        dest_x = barrel.point.x - _BARREL_STANDOFF_M * ux
        dest_y = barrel.point.y - _BARREL_STANDOFF_M * uy
        theta = math.atan2(dy, dx)
        goal = build_nav_goal(Pose(dest_x, dest_y, theta))
        self.bb.set(bb.BARREL_DESTINATION, goal)
        self._ros_logger.info(
            f"{self.name}: computed BARREL_DESTINATION for barrel {barrel.id} -> "
            f"({dest_x:.2f}, {dest_y:.2f}, θ={theta:.2f}); "
            f"RECOMPUTE_BARREL_DESTINATION=False"
        )
        self._ros_logger.debug(
            f"{self.name}: robot=({robot[0]:.2f},{robot[1]:.2f}) "
            f"barrel=({barrel.point.x:.2f},{barrel.point.y:.2f}) "
            f"dist={dist:.2f} standoff={_BARREL_STANDOFF_M:.2f}"
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
            self._ros_logger.info(
                f"{self.name}: skipping spin (tf lookup failed in initialise)"
            )
            return py_trees.common.Status.SUCCESS
        # Return status of spin
        return super().update()


class CheckBarrelLeak(py_trees.behaviour.Behaviour):
    """Read the head barrel's leaking flag (populated by /barrel_detect) and update RViz."""

    def __init__(self, name: str = "CheckBarrelLeak"):
        super().__init__(name=name)
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key(key=bb.PENDING_BARRELS, access=py_trees.common.Access.READ)

    def setup(self, **kwargs):
        self.node = kwargs["node"]
        self._ros_logger: RcutilsLogger = self.node.get_logger()

    def update(self) -> py_trees.common.Status:
        queue = self.bb.get(bb.PENDING_BARRELS)
        assert queue is not None, "CheckBarrelLeak: PENDING_BARRELS not found on blackboard"
        barrel: Barrel = queue[0]

        leaking = bool(barrel.leaking)
        self.node.visualizer.set_barrel_leak(barrel.id, leaking)  # type: ignore[attr-defined]
        self._ros_logger.info(
            f"{self.name}: barrel {barrel.id} leaking={leaking} "
            f"(color={barrel.color}, horiz={barrel.horizontal})"
        )
        return py_trees.common.Status.SUCCESS


class ClearActiveBarrel(py_trees.behaviour.Behaviour):
    """Pop the now-handled barrel off PENDING_BARRELS and mark it handled."""

    def __init__(self, name: str = "ClearActiveBarrel"):
        super().__init__(name=name)
        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key(key=bb.PENDING_BARRELS, access=py_trees.common.Access.WRITE)
        self.bb.register_key(key=bb.HANDLED_BARRELS, access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        self._ros_logger: RcutilsLogger = kwargs["node"].get_logger()

    def update(self) -> py_trees.common.Status:
        queue = self.bb.get(bb.PENDING_BARRELS)
        assert queue is not None, "ClearActiveBarrel: PENDING_BARRELS not found on blackboard"
        handled = self.bb.get(bb.HANDLED_BARRELS)
        assert handled is not None, "ClearActiveBarrel: HANDLED_BARRELS not found on blackboard"

        barrel: Barrel = queue.popleft()
        handled.add(barrel.id)
        self._ros_logger.info(
            f"cleared barrel {barrel.id} from queue; remaining={len(queue)} "
            f"handled={len(handled)}"
        )
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
        NavRetryThenSucceed(
            child=NavigateToBlackboardGoal(
                name="NavigateToBarrelDestination",
                goal_key=bb.BARREL_DESTINATION,
            ),
            queue_key=bb.PENDING_BARRELS,
            head_id_fn=lambda b: b.id,
            name="BarrelNavRetry",
        ),
    ])

    alert_if_leaking = py_trees.composites.Selector(
        name="AlertIfLeaking", memory=False,
    )
    leak_branch = py_trees.composites.Sequence(name="LeakBranch", memory=False)
    leak_branch.add_children([
        IsHeadBarrelLeaking(),
        Speak("Leaking barrel alert alert!", name="SpeakLeakAlert"),
    ])
    alert_if_leaking.add_children([
        leak_branch,
        py_trees.behaviours.Success(name="NoLeak"),
    ])

    seq = py_trees.composites.Sequence(name="GoAndInspectBarrel", memory=True)
    seq.add_children([
        ComputeBarrelDestination(),
        drive_or_recompute,
        TurnTowardsBarrel(),
        CheckBarrelLeak(),
        alert_if_leaking,
        ClearActiveBarrel(),
    ])
    return seq
