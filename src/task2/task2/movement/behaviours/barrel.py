"""GoAndInspectBarrel subtree."""

import math

import py_trees
from rclpy.impl.rcutils_logger import RcutilsLogger

from msg_types.srv import NearestWall

from task2.movement import blackboard as bb
from task2.movement.behaviours._nav import (
    NavigateToBlackboardGoal,
    SERVICE_TIMEOUT_SEC,
    build_nav_goal,
)
from task2.movement.behaviours._speak import Speak
from task2.movement.behaviours.conditions import IsHeadBarrelLeaking
from task2.movement.log_utils import log_throttled
from task2.movement.models import Barrel, Pose


_BARREL_FORWARD_M = 0.5
_BARREL_RIGHT_M = 0.3
_BARREL_YAW_OFFSET_RAD = 0.0
_BARREL_GOAL_MIN_WALL_DIST_M = 0.6


def _wrap_to_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class ComputeBarrelDestination(py_trees.behaviour.Behaviour):
    """Compute one (flip_x, flip_y) standoff goal for the head barrel.

    Returns FAILURE (so outer FailureIsSuccess can move on to the next side)
    when the barrel has no normal, when nearest_wall reports the goal is
    closer than _BARREL_GOAL_MIN_WALL_DIST_M to a wall, or on service
    failure/timeout.
    """

    def __init__(self, flip_x: bool, flip_y: bool, name: str | None = None):
        sx = "-" if flip_x else "+"
        sy = "-" if flip_y else "+"
        super().__init__(name=name or f"ComputeBarrelDestination[{sx}x,{sy}y]")
        self._flip_x = flip_x
        self._flip_y = flip_y

        self.bb = self.attach_blackboard_client(name=self.name)
        self.bb.register_key(key=bb.PENDING_BARRELS, access=py_trees.common.Access.READ)
        self.bb.register_key(key=bb.BARREL_DESTINATION, access=py_trees.common.Access.WRITE)

        self._future = None
        self._dest: tuple[float, float, float] | None = None
        self._skip = False
        self._start_time: float = 0.0

    def setup(self, **kwargs):
        self.node = kwargs["node"]
        self._ros_logger: RcutilsLogger = self.node.get_logger()
        self.client = self.node.create_client(NearestWall, "nearest_wall")

    def initialise(self):
        self._future = None
        self._dest = None
        self._skip = False
        self._start_time = self.node.get_clock().now().nanoseconds * 1e-9

        queue = self.bb.get(bb.PENDING_BARRELS)
        assert queue is not None, f"{self.name}: PENDING_BARRELS not found on blackboard"
        barrel: Barrel = queue[0]

        if barrel.normal is None:
            self._ros_logger.warning(
                f"{self.name}: barrel {barrel.id} has no normal; skipping side"
            )
            self._skip = True
            return

        nx, ny = barrel.normal.x, barrel.normal.y
        fwd = _BARREL_FORWARD_M * (-1.0 if self._flip_x else 1.0)
        rgt = _BARREL_RIGHT_M * (-1.0 if self._flip_y else 1.0)
        dest_x = barrel.point.x + fwd * nx + rgt * ny
        dest_y = barrel.point.y + fwd * ny - rgt * nx
        theta_to_center = math.atan2(
            -fwd * ny + rgt * nx,
            -fwd * nx - rgt * ny,
        )
        theta = _wrap_to_pi(theta_to_center + _BARREL_YAW_OFFSET_RAD)
        self._dest = (dest_x, dest_y, theta)

        if not self.client.service_is_ready():
            self._ros_logger.warning(
                f"{self.name}: nearest_wall service not ready; skipping side"
            )
            self._skip = True
            return

        req = NearestWall.Request()
        req.x = float(dest_x)
        req.y = float(dest_y)
        self._future = self.client.call_async(req)
        self._ros_logger.info(
            f"{self.name}: barrel {barrel.id} candidate "
            f"({dest_x:.2f}, {dest_y:.2f}, θ={theta:.2f}); querying nearest_wall"
        )

    def update(self) -> py_trees.common.Status:
        if self._skip:
            return py_trees.common.Status.FAILURE

        assert self._future is not None and self._dest is not None

        if not self._future.done() or self._future.result() is None:
            now = self.node.get_clock().now().nanoseconds * 1e-9
            elapsed = now - self._start_time
            if elapsed < SERVICE_TIMEOUT_SEC:
                log_throttled(
                    self._ros_logger, self.node, f"{self.name}.waiting", "debug",
                    f"{self.name}: waiting for nearest_wall (elapsed={elapsed:.2f}s)",
                )
                return py_trees.common.Status.RUNNING
            self._ros_logger.warning(
                f"{self.name}: nearest_wall timed out after {elapsed:.2f}s; "
                f"skipping side"
            )
            return py_trees.common.Status.FAILURE

        resp: NearestWall.Response = self._future.result()  # type: ignore
        if not getattr(resp, "success", False):
            self._ros_logger.warning(
                f"{self.name}: nearest_wall returned failure; skipping side"
            )
            return py_trees.common.Status.FAILURE

        dest_x, dest_y, theta = self._dest
        dist = math.hypot(dest_x - resp.point_x, dest_y - resp.point_y)
        if dist < _BARREL_GOAL_MIN_WALL_DIST_M:
            self._ros_logger.info(
                f"{self.name}: side blocked — wall at ({resp.point_x:.2f}, "
                f"{resp.point_y:.2f}) is {dist:.2f}m from goal "
                f"(< {_BARREL_GOAL_MIN_WALL_DIST_M:.2f}m); skipping"
            )
            return py_trees.common.Status.FAILURE

        goal = build_nav_goal(Pose(dest_x, dest_y, theta))
        self.bb.set(bb.BARREL_DESTINATION, goal)
        self._ros_logger.info(
            f"{self.name}: BARREL_DESTINATION -> "
            f"({dest_x:.2f}, {dest_y:.2f}, θ={theta:.2f}); wall_dist={dist:.2f}m"
        )
        return py_trees.common.Status.SUCCESS


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


def _side(flip_x: bool, flip_y: bool) -> py_trees.decorators.FailureIsSuccess:
    sx = "-" if flip_x else "+"
    sy = "-" if flip_y else "+"
    side_seq = py_trees.composites.Sequence(
        name=f"BarrelSide[{sx}x,{sy}y]", memory=True,
    )
    side_seq.add_children([
        ComputeBarrelDestination(flip_x=flip_x, flip_y=flip_y),
        NavigateToBlackboardGoal(
            name=f"NavigateToBarrelDestination[{sx}x,{sy}y]",
            goal_key=bb.BARREL_DESTINATION,
        ),
    ])
    return py_trees.decorators.FailureIsSuccess(
        name=f"BarrelSideTolerant[{sx}x,{sy}y]",
        child=side_seq,
    )


def build() -> py_trees.composites.Sequence:
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
        _side(flip_x=False, flip_y=False),
        _side(flip_x=False, flip_y=True),
        _side(flip_x=True,  flip_y=False),
        _side(flip_x=True,  flip_y=True),
        CheckBarrelLeak(),
        alert_if_leaking,
        ClearActiveBarrel(),
    ])
    return seq
