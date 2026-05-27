"""Shared navigation helpers and thin nav-action wrappers.
"""

import math
from typing import Any, Callable

import py_trees
import py_trees_ros
import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.time import Time
from rclpy.impl.rcutils_logger import RcutilsLogger

from task2.movement.models import Pose, Point, Vector


STANDOFF = 0.5 # How far before target the goal should be set
SERVICE_TIMEOUT_SEC = 5.0
NAV_MAX_ATTEMPTS = 2


def build_nav_goal(pose: Pose) -> NavigateToPose.Goal:
    """Construct a NavigateToPose.Goal from a Pose in the map frame."""
    goal = NavigateToPose.Goal()
    ps = PoseStamped()
    ps.header.frame_id = "map"
    ps.pose.position.x = float(pose.x)
    ps.pose.position.y = float(pose.y)
    ps.pose.orientation.z = math.sin(pose.theta / 2.0)
    ps.pose.orientation.w = math.cos(pose.theta / 2.0)
    goal.pose = ps
    return goal


def standoff_goal_from_normal(point: Point, normal: Vector, standoff: float = STANDOFF) -> NavigateToPose.Goal:
    """Given a point and a normal vector, compute a standoff goal a bit back along the normal."""
    dest_x = point.x + standoff * normal.x
    dest_y = point.y + standoff * normal.y
    theta = math.atan2(-normal.y, -normal.x)
    return build_nav_goal(Pose(dest_x, dest_y, theta))


def standoff_goal_from_robot(robot: Point, target: Point) -> NavigateToPose.Goal:
    """Given the robot's current (x, y) and a target (x, y), compute a standoff goal a bit back from the target."""
    dx = target.x - robot.x
    dy = target.y - robot.y
    dist = math.hypot(dx, dy) or 1.0
    ux, uy = dx / dist, dy / dist
    dest_x = target.x - STANDOFF * ux
    dest_y = target.y - STANDOFF * uy
    theta = math.atan2(dy, dx)
    return build_nav_goal(Pose(dest_x, dest_y, theta))


def lookup_robot_xy(tf_buffer: tf2_ros.Buffer, logger: RcutilsLogger) -> tuple[float, float] | None:
    try:
        t = tf_buffer.lookup_transform("map", "base_link", Time())
    except Exception as exc:
        logger.warning(f"tf lookup map->base_link failed: {exc}")
        return None
    xy = (t.transform.translation.x, t.transform.translation.y)
    logger.debug(f"lookup_robot_xy: map->base_link = ({xy[0]:.2f}, {xy[1]:.2f})")
    return xy


def _log_pose(logger: RcutilsLogger, name: str, goal: NavigateToPose.Goal) -> None:
    """Log the (x, y, θ) pose from a NavigateToPose.Goal."""
    pos = goal.pose.pose.position
    q = goal.pose.pose.orientation
    theta = math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)
    logger.info(
        f"{name}: navigating to ({pos.x:.2f}, {pos.y:.2f}, θ={theta:.2f})"
    )


class LoggingNavWaypoint(py_trees_ros.action_clients.FromConstant):
    """FromConstant + 'navigating to pose ...' log on each fresh attempt."""

    def __init__(self, name: str, action_goal: NavigateToPose.Goal, **kwargs):
        super().__init__(
            name=name,
            action_type=NavigateToPose,
            action_name="navigate_to_pose",
            action_goal=action_goal,
            **kwargs,
        )

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._ros_logger: RcutilsLogger = kwargs["node"].get_logger()

    def initialise(self):
        super().initialise()
        _log_pose(self._ros_logger, self.name, self.blackboard.goal)

    def terminate(self, new_status):
        self._ros_logger.info(
            f"{self.name}: nav terminating with status={new_status.name}"
        )
        super().terminate(new_status)


class NavRetryThenSucceed(py_trees.decorators.Decorator):
    """Tolerate unreachable goals: after `max_attempts` child FAILUREs, return SUCCESS.

    Wraps a nav behaviour so the outer flow keeps going even when the computed
    destination is not reachable. Attempts are counted per-target: the counter
    resets when the head of the wrapped queue changes (i.e. a new person/barrel
    is being handled) or when the child reports SUCCESS.

    - child SUCCESS  -> reset counter, return SUCCESS
    - child RUNNING  -> return RUNNING
    - child FAILURE  -> increment counter
        - below threshold: return FAILURE (lets the outer sequence recompute
          and try again on the next tick — same behaviour as before)
        - at/over threshold: reset counter, return SUCCESS (give up gracefully)
    """

    def __init__(
        self,
        child: py_trees.behaviour.Behaviour,
        queue_key: str,
        head_id_fn: Callable[[Any], Any],
        max_attempts: int = NAV_MAX_ATTEMPTS,
        name: str = "NavRetryThenSucceed",
    ):
        super().__init__(name=name, child=child)
        self._queue_key = queue_key
        self._head_id_fn = head_id_fn
        self._max_attempts = max_attempts
        self._attempts = 0
        self._tracked_id: Any = None

        self._rbb = self.attach_blackboard_client(name=name)
        self._rbb.register_key(key=queue_key, access=py_trees.common.Access.READ)

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._ros_logger: RcutilsLogger = kwargs["node"].get_logger()

    def _current_id(self) -> Any:
        queue = self._rbb.get(self._queue_key)
        if not queue:
            return None
        return self._head_id_fn(queue[0])

    def update(self) -> py_trees.common.Status:
        current_id = self._current_id()
        if current_id != self._tracked_id:
            if self._attempts > 0:
                self._ros_logger.debug(
                    f"{self.name}: target changed "
                    f"({self._tracked_id} -> {current_id}); resetting attempts"
                )
            self._tracked_id = current_id
            self._attempts = 0

        status = self.decorated.status
        if status == py_trees.common.Status.SUCCESS:
            self._attempts = 0
            return py_trees.common.Status.SUCCESS
        if status == py_trees.common.Status.RUNNING:
            return py_trees.common.Status.RUNNING

        # FAILURE
        self._attempts += 1
        if self._attempts >= self._max_attempts:
            self._ros_logger.warning(
                f"{self.name}: nav failed {self._attempts}x for target "
                f"{current_id}; giving up and continuing flow"
            )
            self._attempts = 0
            return py_trees.common.Status.SUCCESS
        self._ros_logger.warning(
            f"{self.name}: nav failed (attempt {self._attempts}/"
            f"{self._max_attempts}) for target {current_id}; will retry"
        )
        return py_trees.common.Status.FAILURE


class NavigateToBlackboardGoal(py_trees_ros.action_clients.FromBlackboard):
    """FromBlackboard + 'navigating to pose ...' log on each fresh attempt."""

    def __init__(self, name: str, goal_key: str, **kwargs):
        super().__init__(
            name=name,
            action_type=NavigateToPose,
            action_name="navigate_to_pose",
            key=goal_key,
            **kwargs,
        )
        self._goal_key = goal_key

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._ros_logger: RcutilsLogger = kwargs["node"].get_logger()

    def initialise(self):
        self._ros_logger.debug(
            f"{self.name}: reading goal from blackboard key '{self._goal_key}'"
        )
        super().initialise()
        if self.send_goal_future is not None:
            _log_pose(self._ros_logger, self.name, self.blackboard.goal)
        else:
            self._ros_logger.warning(
                f"{self.name}: send_goal_future is None — goal not dispatched"
            )

    def terminate(self, new_status):
        self._ros_logger.info(
            f"{self.name}: nav terminating with status={new_status.name}"
        )
        super().terminate(new_status)
